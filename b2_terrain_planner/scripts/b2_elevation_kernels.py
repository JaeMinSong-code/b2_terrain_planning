#!/usr/bin/env python3
"""B2 전용 add_points_kernel 변형.

elevation_mapping_cupy 공식 저장소는 원본 그대로 사용한다. B2가 필요로 하는
수정2(옵션B) 모서리 보존 max-fusion 분기만 이 복제 커널에 담는다. preamble(map_utils)은
upstream에서 그대로 import하므로 커널 operation 문자열 외의 중복은 없다.

upstream 커널이 바뀌면 이 복제본을 갱신한다(원본은 수정하지 않는다).
"""
import string

import cupy as cp
from elevation_mapping_cupy.kernels.custom_kernels import map_utils


def b2_add_points_kernel(
    resolution,
    width,
    height,
    sensor_noise_factor,
    mahalanobis_thresh,
    outlier_variance,
    wall_num_thresh,
    max_ray_length,
    cleanup_step,
    min_valid_distance,
    max_height_range,
    cleanup_cos_thresh,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
    enable_edge_shaped=True,
    enable_visibility_cleanup=True,
    enable_max_fusion=False,
    max_fusion_edge_thresh=0.03,
):
    add_points_kernel = cp.ElementwiseKernel(
        in_params="raw U center_x, raw U center_y, raw U R, raw U t, raw U map, raw U norm_map",
        out_params="raw U p, raw T newmap, raw T visibility",
        preamble=map_utils(
            resolution,
            width,
            height,
            sensor_noise_factor,
            min_valid_distance,
            max_height_range,
            ramped_height_range_a,
            ramped_height_range_b,
            ramped_height_range_c,
        ),
        operation=string.Template(
            """
            U rx = p[i * 3];
            U ry = p[i * 3 + 1];
            U rz = p[i * 3 + 2];
            U x = transform_p(rx, ry, rz, R[0], R[1], R[2], t[0]);
            U y = transform_p(rx, ry, rz, R[3], R[4], R[5], t[1]);
            U z = transform_p(rx, ry, rz, R[6], R[7], R[8], t[2]);
            U v = point_noise(rx, ry, rz);
            int idx = get_idx(x, y, center_x[0], center_y[0]);
            bool valid = is_valid(x, y, z, t[0], t[1], t[2]);
            p[i * 3] = idx;
            p[i * 3 + 1] = valid;
            p[i * 3 + 2] = is_inside(idx);
            if (!valid) { return; }

            if (is_inside(idx)) {
                    U map_h = map[get_map_idx(idx, 0)];
                    U map_v = map[get_map_idx(idx, 1)];
                    U num_points = newmap[get_map_idx(idx, 4)];
                    if (abs(map_h - z) > (map_v * ${mahalanobis_thresh})) {
                        atomicAdd(&newmap[get_map_idx(idx, 5)], 1.0);
                    }
                    else {
                        if (${enable_edge_shaped} && (num_points > ${wall_num_thresh}) && (z < map_h - map_v * ${mahalanobis_thresh} / num_points)) {
                          // continue;
                        }
                        else if (${enable_max_fusion} && (z > map_h + ${max_fusion_edge_thresh})) {
                            // 수정2 옵션B(모서리 보존): 새 측정이 기존보다 뚜렷이 높으면
                            // Kalman 평균으로 뭉개지 말고 새 값으로 덮어쓰고 variance를 새 측정 노이즈로 초기화.
                            atomicAdd(&newmap[get_map_idx(idx, 0)], z);
                            atomicAdd(&newmap[get_map_idx(idx, 1)], v);
                            atomicAdd(&newmap[get_map_idx(idx, 2)], 1.0);
                        }
                        else {
                            T new_h = (map_h * v + z * map_v) / (map_v + v);
                            T new_v = (map_v * v) / (map_v + v);
                            atomicAdd(&newmap[get_map_idx(idx, 0)], new_h);
                            atomicAdd(&newmap[get_map_idx(idx, 1)], new_v);
                            atomicAdd(&newmap[get_map_idx(idx, 2)], 1.0);
                        }
                        // visibility cleanup
                    }
            }
            if (${enable_visibility_cleanup}) {
                float ray_x, ray_y, ray_z;
                float ray_length = ray_vector(t[0], t[1], t[2], x, y, z, ray_x, ray_y, ray_z);
                ray_length = fminf(ray_length, (float)${max_ray_length});
                float end_x = t[0] + ray_x * ray_length;
                float end_y = t[1] + ray_y * ray_length;
                float end_z = t[2] + ray_z * ray_length;
                float start_grid_x = (
                    (t[0] - center_x[0]) / ${resolution} + 0.5f * (${width} - 1) + 0.5f
                );
                float start_grid_y = (
                    (t[1] - center_y[0]) / ${resolution} + 0.5f * (${height} - 1) + 0.5f
                );
                float end_grid_x = (
                    (end_x - center_x[0]) / ${resolution} + 0.5f * (${width} - 1) + 0.5f
                );
                float end_grid_y = (
                    (end_y - center_y[0]) / ${resolution} + 0.5f * (${height} - 1) + 0.5f
                );
                float grid_dx = end_grid_x - start_grid_x;
                float grid_dy = end_grid_y - start_grid_y;
                int cell_x = (int)floorf(start_grid_x);
                int cell_y = (int)floorf(start_grid_y);
                int end_cell_x = (int)floorf(end_grid_x);
                int end_cell_y = (int)floorf(end_grid_y);
                int step_x = (grid_dx > 0.0f) - (grid_dx < 0.0f);
                int step_y = (grid_dy > 0.0f) - (grid_dy < 0.0f);
                float delta_x = step_x == 0 ? 1.0e30f : fabsf(1.0f / grid_dx);
                float delta_y = step_y == 0 ? 1.0e30f : fabsf(1.0f / grid_dy);
                float boundary_x = cell_x + (step_x > 0 ? 1.0f : 0.0f);
                float boundary_y = cell_y + (step_y > 0 ? 1.0f : 0.0f);
                float next_x = step_x == 0 ? 1.0e30f : (boundary_x - start_grid_x) / grid_dx;
                float next_y = step_y == 0 ? 1.0e30f : (boundary_y - start_grid_y) / grid_dy;

                for (int step = 0; step < ${width} + ${height}; ++step) {
                    float entry;
                    if (next_x < next_y) {
                        entry = next_x;
                        next_x += delta_x;
                        cell_x += step_x;
                    }
                    else if (next_y < next_x) {
                        entry = next_y;
                        next_y += delta_y;
                        cell_y += step_y;
                    }
                    else {
                        entry = next_x;
                        next_x += delta_x;
                        next_y += delta_y;
                        cell_x += step_x;
                        cell_y += step_y;
                    }
                    if (entry >= 1.0f || (cell_x == end_cell_x && cell_y == end_cell_y)) {break;}
                    if (
                        cell_x <= 0 || cell_x >= ${width} - 1
                        || cell_y <= 0 || cell_y >= ${height} - 1
                    ) {continue;}

                    int nidx = cell_y * ${width} + cell_x;
                    float exit = fminf(fminf(next_x, next_y), 1.0f);
                    float sample = 0.5f * (entry + exit);
                    U nz = t[2] + (end_z - t[2]) * sample;

                    U nmap_h = map[get_map_idx(nidx, 0)];
                    U nmap_v = map[get_map_idx(nidx, 1)];
                    U nmap_valid = map[get_map_idx(nidx, 2)];
                    // Time layer
                    U non_updated_t = map[get_map_idx(nidx, 4)];
                    // If invalid, do upper bound check, then skip
                    if (nmap_valid < 0.5) {
                      atomic_min_float(&visibility[get_map_idx(nidx, 2)], nz);
                      continue;
                    }
                    // If updated recently, skip
                    if (non_updated_t < 0.5) {continue;}

                    if (nmap_h > nz + 0.01 - fminf(nmap_v, 1.0f) * 0.05) {
                        // If ray and norm is vertical, skip
                        U norm_x = norm_map[get_map_idx(nidx, 0)];
                        U norm_y = norm_map[get_map_idx(nidx, 1)];
                        U norm_z = norm_map[get_map_idx(nidx, 2)];
                        float product = inner_product(ray_x, ray_y, ray_z, norm_x, norm_y, norm_z);
                        if (fabs(product) < ${cleanup_cos_thresh}) {continue;}
                        U num_points = newmap[get_map_idx(nidx, 3)];
                        if (num_points > ${wall_num_thresh} && non_updated_t < 1.0) {continue;}

                        // Finally, this cell is penetrated by the ray.
                        atomicAdd(
                            &visibility[get_map_idx(nidx, 0)],
                            ${cleanup_step}/(ray_length / ${max_ray_length})
                        );
                        atomicAdd(&visibility[get_map_idx(nidx, 1)], 1.0);
                        atomic_min_float(&visibility[get_map_idx(nidx, 2)], nz);
                    }
                }
            }
            """
        ).substitute(
            mahalanobis_thresh=mahalanobis_thresh,
            outlier_variance=outlier_variance,
            wall_num_thresh=wall_num_thresh,
            resolution=resolution,
            width=width,
            height=height,
            max_ray_length=max_ray_length,
            cleanup_step=cleanup_step,
            cleanup_cos_thresh=cleanup_cos_thresh,
            enable_edge_shaped=int(enable_edge_shaped),
            enable_visibility_cleanup=int(enable_visibility_cleanup),
            enable_max_fusion=int(enable_max_fusion),
            max_fusion_edge_thresh=max_fusion_edge_thresh,
        ),
        name="add_points_kernel",
    )
    return add_points_kernel
