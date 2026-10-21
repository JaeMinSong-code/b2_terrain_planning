"""맵 입력의 좌표·시간·점군 변환. ROS 런타임과 분리하여 회귀 검증한다."""

from dataclasses import dataclass
import numpy as np


def unit_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,):
        raise ValueError('quaternion must have four components')
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-8:
        raise ValueError("invalid quaternion")
    return q / norm


def rotation(q):
    x, y, z, w = unit_quaternion(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def quaternion_product(a, b):
    x, y, z, w = unit_quaternion(a)
    u, v, s, t = unit_quaternion(b)
    return unit_quaternion([w*u+x*t+y*s-z*v, w*v-x*s+y*t+z*u,
                            w*s+x*v-y*u+z*t, w*t-x*u-y*v-z*s])


def slerp(a, b, fraction):
    a, b = unit_quaternion(a), unit_quaternion(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    if dot > 0.9995:
        return unit_quaternion(a + fraction * (b - a))
    theta = np.arccos(np.clip(dot, -1, 1))
    return (np.sin((1 - fraction) * theta) * a + np.sin(fraction * theta) * b) / np.sin(theta)


@dataclass
class Pose:
    stamp: float
    parent: str
    child: str
    position: np.ndarray
    quaternion: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float)
        self.quaternion = unit_quaternion(self.quaternion)
        if (not self.parent or not self.child or self.parent == self.child
                or self.position.shape != (3,) or not np.isfinite(self.position).all()
                or not np.isfinite(self.stamp) or self.stamp <= 0):
            raise ValueError("invalid odometry pose/frame/stamp")


def interpolate(poses, stamp, max_gap):
    for pose in poses:
        if abs(pose.stamp - stamp) < 1e-7:
            return pose
    for left, right in zip(poses, list(poses)[1:]):
        if left.stamp < stamp < right.stamp:
            if (right.stamp - left.stamp > max_gap
                    or (left.parent, left.child) != (right.parent, right.child)):
                raise ValueError("odometry gap or frame change")
            ratio = (stamp - left.stamp) / (right.stamp - left.stamp)
            return Pose(stamp, left.parent, left.child,
                        left.position * (1-ratio) + right.position * ratio,
                        slerp(left.quaternion, right.quaternion, ratio))
    raise ValueError("waiting for bracketing odometry (no extrapolation)")


def decode_xyz(msg):
    """FLOAT32/FLOAT64, endian, organized row padding을 해석해 XYZ float32로 변환."""
    fields = {field.name: field for field in msg.fields}
    formats, offsets = [], []
    endian = "<"
    if msg.is_bigendian:
        endian = ">"
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError(f"unsupported point field: {name}")
        kind = "f4"
        if field.datatype == 8:
            kind = "f8"
        if field.offset < 0 or field.offset + np.dtype(kind).itemsize > msg.point_step:
            raise ValueError("point field exceeds point_step")
        formats.append(endian + kind)
        offsets.append(field.offset)
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.height * msg.row_step:
        raise ValueError("invalid pointcloud data/row_step")
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=formats,
                         offsets=offsets, itemsize=msg.point_step))
    data = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step))
    xyz = np.column_stack([data[n].reshape(-1) for n in ('x', 'y', 'z')]).astype('<f4')
    return xyz[np.isfinite(xyz).all(axis=1)]


def flat_ground_height(points_world, base_position, half_extent, min_points=20):
    """평지 시험용: 지도 영역 안 몸통보다 20 cm 아래인 최신 점군의 지면 중앙값."""
    points = np.asarray(points_world, dtype=float)
    base = np.asarray(base_position, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or base.shape != (3,)
            or not np.isfinite(base).all() or not np.isfinite(half_extent)
            or half_extent <= 0):
        raise ValueError('invalid plane geometry')
    valid = (np.isfinite(points).all(axis=1)
             & (np.abs(points[:, 0]-base[0]) <= half_extent)
             & (np.abs(points[:, 1]-base[1]) <= half_extent)
             & (points[:, 2] < base[2]-.2))
    heights = points[valid, 2]
    if len(heights) < min_points:
        raise ValueError('not enough ground points in current cloud')
    return float(np.median(heights))
