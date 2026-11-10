#!/usr/bin/env python3
"""ElevationMap 서브클래스: 공식 CuPy 저장소를 수정하지 않고 B2 확장만 얹는다.

elevation_mapping_cupy는 원본 그대로 두고, 아래 B2 handoff 3수정을 서브클래스로 구현한다.
- 수정1 protection zone: base 아래 박스 셀의 time_variance 증가를 건너뛴다(발밑 기억 보존).
- 수정2 옵션B: upstream 커널은 그대로 두고 add_points_kernel만 모서리 보존 변형으로 교체.
- 수정3 policy crop: base 위치+yaw 기준 로컬 heightmap을 GPU 맵에서 직접 잘라온다.
또 raw 레이어 일괄 복사(get_maps_with_names_ref)로 발행 복사를 한 번으로 줄인다.

확장 파라미터(enable_protection_zone 등)는 노드의 _set_b2_extension_params가 self.param에
동적으로 주입한다(upstream Parameter에는 필드를 추가하지 않는다).
"""
import math

import cupy as cp
import numpy as np
from elevation_mapping_cupy.elevation_mapping import ElevationMap

from b2_elevation_kernels import b2_add_points_kernel


class B2ElevationMap(ElevationMap):
    def __init__(self, param):
        super().__init__(param)
        # B2 확장용 캐시: protection zone 셀 오프셋, 정책 크롭 base-frame 격자.
        self._pz_dx = None
        self._pz_dy = None
        self._crop_cache = None  # (len_x, len_y, res) -> (nx, ny, bx, by) base-frame 좌표

    def compile_kernels(self):
        """vanilla 커널을 그대로 만든 뒤 add_points_kernel만 B2 변형으로 교체한다."""
        super().compile_kernels()
        # 수정2 옵션B: 원본 custom_kernels는 수정하지 않고 B2 복제 커널로 덮어쓴다.
        self.add_points_kernel = b2_add_points_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            self.param.sensor_noise_factor,
            self.param.mahalanobis_thresh,
            self.param.outlier_variance,
            self.param.wall_num_thresh,
            self.param.max_ray_length,
            self.param.cleanup_step,
            self.param.min_valid_distance,
            self.param.max_height_range,
            self.param.cleanup_cos_thresh,
            self.param.ramped_height_range_a,
            self.param.ramped_height_range_b,
            self.param.ramped_height_range_c,
            self.param.enable_edge_sharpen,
            self.param.enable_visibility_cleanup,
            self.param.enable_max_fusion,
            self.param.max_fusion_edge_thresh,
        )

    def _base_yaw(self):
        """base_rotation에서 yaw[rad] 추출."""
        R = self.base_rotation
        return float(cp.arctan2(R[1, 0], R[0, 0]))

    def _protection_zone_mask(self):
        """수정1: base 프레임(yaw만) 박스 안 셀 = True 인 (cell_n, cell_n) bool 마스크.

        맵은 world-축 정렬이라 셀 오프셋(맵 중심 기준)은 고정. 매 호출 시 현재
        yaw로만 회전시켜 base 프레임 좌표를 얻는다. 맵 중심 ≈ base xy.
        """
        if self._pz_dx is None:
            off = (cp.arange(self.cell_n, dtype=self.data_type)
                   - (self.cell_n - 1) / 2.0) * self.resolution
            self._pz_dy = off.reshape(self.cell_n, 1)  # rows = Y (axis 1)
            self._pz_dx = off.reshape(1, self.cell_n)  # cols = X (axis 2)
        yaw = self._base_yaw()
        c = math.cos(yaw)
        s = math.sin(yaw)
        x_base = c * self._pz_dx + s * self._pz_dy
        y_base = -s * self._pz_dx + c * self._pz_dy
        return (cp.abs(x_base) <= self.param.protection_zone_x) & (
            cp.abs(y_base) <= self.param.protection_zone_y
        )

    def update_variance(self):
        """Adds the time variance to the valid cells.

        수정1(protection zone): base 아래 박스 안 셀은 time_variance 증가를
        건너뛰어 발밑 기억이 시간 경과로 무효화되지 않게 한다.
        """
        inc = self.param.time_variance * self.elevation_map[2]
        if self.param.enable_protection_zone:
            inc = inc * (~self._protection_zone_mask())
        self.elevation_map[1] += inc

    def get_policy_crop(self, base_x, base_y, base_z, yaw, length_x, length_y, resolution):
        """수정3: 정책 입력용 로컬 heightmap 크롭을 GPU 맵에서 직접 잘라온다.

        base_link 위치 + yaw(roll/pitch 제거) 기준의 length_x(앞뒤)×length_y(좌우)
        직사각 영역을 resolution 격자로 최근접 샘플링한다. 맵이 world-축 정렬이므로
        yaw 회전된 격자를 world 셀 인덱스로 변환해 gather 한다(모서리 보존을 위해
        bilinear 금지, 최근접만).

        Returns:
            H        (nx, ny) float32: 셀 높이 − base_z (무효 셀 0)
            M        (nx, ny) float32: 유효 1 / 무효 0
            H_filled (nx, ny) float32: H의 무효 셀을 최근접 유효값으로 채움(선형보간 금지)
            축: axis0 = 전방(+x), axis1 = 좌측(+y)
        """
        nx = int(round(length_x / resolution))
        ny = int(round(length_y / resolution))

        key = (nx, ny, float(resolution))
        if self._crop_cache is None or self._crop_cache[0] != key:
            xb = (cp.arange(nx, dtype=self.data_type) - (nx - 1) / 2.0) * resolution
            yb = (cp.arange(ny, dtype=self.data_type) - (ny - 1) / 2.0) * resolution
            XB, YB = cp.meshgrid(xb, yb, indexing="ij")  # (nx, ny)
            self._crop_cache = (key, XB, YB)
        _, XB, YB = self._crop_cache

        c = math.cos(yaw)
        s = math.sin(yaw)
        cx = float(self.center[0])
        cy = float(self.center[1])
        # base-frame 격자 → world 좌표
        wx = base_x + c * XB - s * YB
        wy = base_y + s * XB + c * YB
        # world → 맵 셀 인덱스 (col=x=axis2, row=y=axis1)
        col = cp.rint((wx - cx) / self.resolution + (self.cell_n - 1) / 2.0).astype(cp.int32)
        row = cp.rint((wy - cy) / self.resolution + (self.cell_n - 1) / 2.0).astype(cp.int32)
        in_bounds = (col >= 0) & (col < self.cell_n) & (row >= 0) & (row < self.cell_n)
        colc = cp.clip(col, 0, self.cell_n - 1)
        rowc = cp.clip(row, 0, self.cell_n - 1)

        with self.map_lock:
            elev = self.elevation_map[0][rowc, colc]
            is_valid = self.elevation_map[2][rowc, colc]
        valid = in_bounds & (is_valid > 0.5)
        H = cp.where(valid, elev - base_z, cp.asarray(0.0, dtype=self.data_type))
        M = valid.astype(self.data_type)

        H_np = cp.asnumpy(H).astype(np.float32)
        M_np = cp.asnumpy(M).astype(np.float32)
        valid_np = M_np > 0.5

        # 최근접 유효값 채움(선형보간 금지). 셀 수가 작아 CPU scipy로 충분.
        H_filled = H_np.copy()
        if valid_np.any() and not valid_np.all():
            from scipy import ndimage as _ndi
            _, (ir, ic) = _ndi.distance_transform_edt(
                ~valid_np, return_indices=True
            )
            H_filled = H_np[ir, ic]
        return H_np, M_np, H_filled

    def get_maps_with_names_ref(self, names, data):
        """Raw 높이·유효성·분산을 같은 잠금/stream에서 준비해 한 번에 CPU로 복사한다.

        기존 단층 출력과 같은 NaN, center.z, 축 변환을 적용한다.
        다른 출력층은 기존 plugin 갱신 순서와 단층 경로를 유지한다.
        """
        if not names:
            return
        raw_layers = {"elevation", "is_valid", "variance"}
        if not all(name in raw_layers for name in names):
            for name, target in zip(names, data):
                self.get_map_with_name_ref(name, target)
            return
        with self.map_lock:
            layers = []
            for name in names:
                if name == "elevation":
                    layer = self.get_elevation()
                elif name == "variance":
                    layer = self.get_variance()
                else:
                    layer = self.elevation_map[2, 1:-1, 1:-1]
                layers.append(self._transform_to_grid_map_coordinate_convention(layer))
            # 생산과 복사를 같은 current stream에 순서대로 배치한다.
            packed = cp.stack(layers).astype(cp.float32, copy=False)
            cp.asnumpy(packed, out=data)
