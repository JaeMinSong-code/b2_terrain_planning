"""URDF 순기구학과 몸체 박스·발 구의 지도 입력 제거 영역."""

from collections import deque
import xml.etree.ElementTree as ET

import numpy as np


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * (skew @ skew)


class FootKinematics:
    """B2 URDF의 base_link→foot 체인을 그대로 평가한다."""

    def __init__(self, path):
        joints = {joint.find('child').get('link'): joint
                  for joint in ET.parse(path).getroot().findall('joint')}
        self.chains = []
        for leg in ('FL', 'FR', 'RL', 'RR'):
            chain = []
            child = leg + '_foot'
            while child != 'base_link':
                joint = joints[child]
                origin = joint.find('origin')
                transform = np.eye(4)
                if origin is not None:
                    transform[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
                    roll, pitch, yaw = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
                    transform[:3, :3] = (axis_rotation([0, 0, 1], yaw)
                                         @ axis_rotation([0, 1, 0], pitch)
                                         @ axis_rotation([1, 0, 0], roll))
                axis = None
                if joint.get('type') in ('revolute', 'continuous'):
                    axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
                elif joint.get('type') != 'fixed':
                    raise ValueError('unsupported foot joint: ' + joint.get('name'))
                chain.append((transform, axis))
                child = joint.find('parent').get('link')
            chain.reverse()
            if sum(axis is not None for _, axis in chain) != 3:
                raise ValueError('B2 foot chain must have three joints')
            self.chains.append(chain)

    def feet(self, joints):
        joints = np.asarray(joints, dtype=float).reshape(4, 3)
        if not np.isfinite(joints).all():
            raise ValueError('invalid measured joints')
        feet = []
        for chain, angles in zip(self.chains, joints):
            transform = np.eye(4)
            index = 0
            for origin, axis in chain:
                transform = transform @ origin
                if axis is not None:
                    motion = np.eye(4)
                    motion[:3, :3] = axis_rotation(axis, angles[index])
                    transform = transform @ motion
                    index += 1
            feet.append(transform[:3, 3])
        return np.asarray(feet)


def measured_joints(message, appendage_ids):
    """관절 전용 appendage 배열을 ID로 정렬한다. jt_pos_enc의 자세 성분은 사용하지 않는다."""
    ids = list(message.appendage_ids)
    counts = list(message.appendage_joint_counts)
    values = list(message.appendage_actual_joints)
    if (len(ids) != len(counts) or len(ids) != len(set(ids))
            or any(n <= 0 for n in counts) or sum(counts) != len(values)):
        raise ValueError('invalid appendage joint layout')
    mapped = {}
    offset = 0
    for name, count in zip(ids, counts):
        mapped[name] = values[offset:offset + count]
        offset += count
    if any(name not in mapped or len(mapped[name]) != 3 for name in appendage_ids):
        raise ValueError('missing measured leg joints')
    result = np.asarray([mapped[name] for name in appendage_ids], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('nonfinite measured leg joints')
    return result


class JointHistory:
    """수신 시각 보간. 다음 관절 표본을 기다리지 않고 신선한 직전 측정을 사용한다."""

    def __init__(self):
        self.samples = deque(maxlen=200)

    def append(self, stamp, joints):
        if self.samples and stamp <= self.samples[-1][0]:
            self.samples.clear()
        self.samples.append((stamp, joints.copy()))

    def at(self, stamp, max_age):
        previous = None
        for current in self.samples:
            if current[0] == stamp:
                return current[1]
            if current[0] > stamp:
                if previous is None or stamp - previous[0] > max_age:
                    return None
                gap = current[0] - previous[0]
                if gap <= max_age:
                    ratio = (stamp - previous[0]) / gap
                    return previous[1] * (1 - ratio) + current[1] * ratio
                break
            previous = current
        if previous is not None and 0 <= stamp - previous[0] <= max_age:
            return previous[1]
        return None


def keep_external_points(points_body, lower, upper, feet, foot_radius):
    """몸통 좌표의 박스 또는 발 구 내부·경계의 점을 제외한다."""
    inside = np.all((points_body >= lower) & (points_body <= upper), axis=1)
    for foot in feet:
        delta = points_body - foot
        inside |= np.einsum('ij,ij->i', delta, delta) <= foot_radius ** 2
    return ~inside
