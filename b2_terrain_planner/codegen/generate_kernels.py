#!/usr/bin/env python3
"""TAMOLS 표본의 잔차·제약·Jacobian을 독립 C 함수로 생성한다.

CasADi는 재생성 도구에서만 사용한다. z는 pose(6), velocity(6),
acceleration(6), FL/FR/RL/RR 접촉점(12), 지지 여유(1)이다.
p는 mass(1), column-major inertia(9), leg-major hips(12), height,
mu, leg_min, leg_max, body-frame command(vx, vy, yaw_rate),
기준 다리 벡터(12), 실제 모델 모드(1), pitch·수직 길이 비용의 제곱근(2), pitch/roll 한계(1), 다리별 최소·최대 도달 거리(4+4)이다.
"""

import argparse
import json
import os
from pathlib import Path
import re

import casadi as ca
import numpy as np


def rotation_rates(phi, velocity, acceleration):
    yaw, pitch, roll = phi[0], phi[1], phi[2]
    cy, sy = ca.cos(yaw), ca.sin(yaw)
    cp, sp = ca.cos(pitch), ca.sin(pitch)
    cr, sr = ca.cos(roll), ca.sin(roll)
    rotation = ca.vertcat(
        ca.horzcat(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        ca.horzcat(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        ca.horzcat(-sp, cp * sr, cp * cr))
    transform = ca.vertcat(ca.horzcat(-sp, 0, 1),
                          ca.horzcat(sr * cp, cr, 0),
                          ca.horzcat(cr * cp, -sr, 0))
    pd, rd = velocity[1], velocity[2]
    derivative = ca.vertcat(
        ca.horzcat(-cp * pd, 0, 0),
        ca.horzcat(cr * rd * cp - sr * sp * pd, -sr * rd, 0),
        ca.horzcat(-sr * rd * cp - cr * sp * pd, -cr * rd, 0))
    return rotation, transform @ velocity, transform @ acceleration + derivative @ velocity


def make_kernel(name, contacts):
    z = ca.SX.sym('z', 31)
    p = ca.SX.sym('p', 54)
    q, velocity, acceleration = z[:6], z[6:12], z[12:18]
    feet = ca.reshape(z[18:30], 3, 4)
    mass, inertia = p[0], ca.reshape(p[1:10], 3, 3)
    hips = ca.reshape(p[10:22], 3, 4)
    height, mu, leg_min, leg_max, command = p[22], p[23], p[24], p[25], p[26:29]
    rotation, omega, alpha = rotation_rates(q[3:], velocity[3:], acceleration[3:])
    momentum = rotation @ (inertia @ alpha + ca.cross(omega, inertia @ omega))
    gia = ca.DM([0, 0, -9.81]) - acceleration[:3]
    heading = ca.vertcat(ca.horzcat(ca.cos(q[3]), ca.sin(q[3])),
                        ca.horzcat(-ca.sin(q[3]), ca.cos(q[3])))
    residuals = [np.sqrt(20) * (heading @ velocity[:2] - command[:2]),
                 2 * (velocity[3] - command[2]), .1 * acceleration[:3],
                 .1 * alpha, p[42] * q[4], np.sqrt(30) * q[5]]
    constraints, lower, upper = [], [], []

    def bound(value, lo, hi):
        constraints.append(value)
        lower.append(lo)
        upper.append(hi)

    bound(gia[2], -ca.inf, -.1)
    for axis in (0, 1):
        bound(gia[axis] + mu * gia[2] / np.sqrt(2), -ca.inf, 0)
        bound(-gia[axis] + mu * gia[2] / np.sqrt(2), -ca.inf, 0)
    for axis in (4, 5):
        bound(q[axis], -p[44], p[44])
    for axis in range(3):
        bound(acceleration[axis], -15, 15)
    if len(contacts) == 2:
        i, j = contacts
        edge = feet[:, j] - feet[:, i]
        mi = ca.cross(q[:3] - feet[:, i], gia) - momentum / mass
        mj = ca.cross(q[:3] - feet[:, j], gia) - momentum / mass
        bound(ca.dot(edge, mi), 0, 0)
        bound(ca.cross(edge, mi)[2], 0, ca.inf)
        bound(ca.cross(-edge, mj)[2], 0, ca.inf)
    else:
        for index, i in enumerate(contacts):
            j = contacts[(index + 1) % len(contacts)]
            h = contacts[(index + 2) % len(contacts)]
            edge = feet[:, j] - feet[:, i]
            moment = ca.cross(q[:3] - feet[:, i], gia) - momentum / mass
            bound(ca.dot(edge, moment) - z[30], -ca.inf, 0)
            bound(ca.cross(edge, feet[:, h] - feet[:, j])[2], .01, ca.inf)
    for leg_index in contacts:
        leg = q[:3] + rotation @ hips[:, leg_index] - feet[:, leg_index]
        lower_reach = (1 - p[41]) * leg_min + p[41] * p[45 + leg_index]
        upper_reach = (1 - p[41]) * leg_max + p[41] * p[49 + leg_index]
        bound(ca.sumsqr(leg), lower_reach**2, upper_reach**2)
        bound(q[2] - feet[2, leg_index], .2, ca.inf)
        target = ((1 - p[41]) * ca.vertcat(0, 0, height) +
                  p[41] * rotation @ p[29 + 3 * leg_index:32 + 3 * leg_index])
        residuals.extend([np.sqrt(3) * (leg - target),
                          p[43] * (leg[2] - target[2])])
    # 월드 수직속도 규제(p[53]=sqrt(vz_weight)): 하강 runaway 억제. 잔차 맨 끝에 둔다.
    residuals.append(p[53] * velocity[2])
    residual = ca.vertcat(*residuals)
    constraint = ca.vertcat(*constraints)
    outputs = [residual, ca.densify(ca.jacobian(residual, z)), constraint,
               ca.densify(ca.jacobian(constraint, z)), ca.vertcat(*lower), ca.vertcat(*upper)]
    return ca.Function(name, [z, p], outputs,
                       ['z', 'p'], ['r', 'jr', 'g', 'jg', 'lower', 'upper'])


def main():
    if ca.__version__ != '3.7.2':
        raise RuntimeError('재현 가능한 생성에는 CasADi 3.7.2가 필요합니다')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'generated')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    kernels = [make_kernel('tamols_diagonal_a', [0, 3]),
               make_kernel('tamols_diagonal_b', [2, 1]),
               make_kernel('tamols_full_support', [0, 2, 3, 1])]
    os.chdir(args.output)
    generator = ca.CodeGenerator('tamols_kernels.c', {'with_header': True})
    for kernel in kernels:
        generator.add(kernel)
        z = ca.SX.sym('z', 31)
        p = ca.SX.sym('p', 54)
        outputs = kernel(z, p)
        generator.add(ca.Function(kernel.name() + '_values', [z, p],
                                  [outputs[i] for i in (0, 2, 4, 5)]))
        generator.add(ca.Function(kernel.name() + '_constraint_jacobian', [z, p],
                                  [outputs[i] for i in (2, 3, 4, 5)]))
    generator.generate()
    # 저장소 스타일 계약: 생성기의 null-input 기본값 표현도 명시적 분기로 쓴다.
    generated = Path('tamols_kernels.c')
    pattern = re.compile(r'^(\s*)(\w+)=arg\[(\d+)\]\? (arg\[\d+\]\[\d+\]) : 0;$', re.M)

    def explicit_branch(match):
        indent, variable, index, value = match.groups()
        return (f'{indent}if (arg[{index}]) {{\n{indent}  {variable}={value};\n'
                f'{indent}}} else {{\n{indent}  {variable}=0;\n{indent}}}')

    text = pattern.sub(explicit_branch, generated.read_text())
    if '?' in text:
        raise RuntimeError('unsupported conditional expression in generated C')
    generated.write_text(text)
    header = Path('tamols_kernels.h')
    header.write_text('/**\n * @file tamols_kernels.h\n * @brief CasADi가 생성한 잔차·제약·미분 C API.\n * @version 0.0.5\n * @update 2026-10-14: 다리별 관절 제한 거리를 제약 매개변수로 추가함.\n * @update 2026-10-14: 자세 비용·pitch/roll 한계를 외부 매개변수로 분리함.\n */\n' + header.read_text())
    for path in (generated, header):
        path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines()) + '\n')
    manifest = {'casadi_version' : ca.__version__, 'z_size': 31, 'parameter_size': 54,
                'evaluation_modes': ['full', 'values', 'constraint_jacobian'],
                'kernels': {f.name(): {'residuals': f.size1_out(0),
                                      'constraints': f.size1_out(2)} for f in kernels}}
    Path('manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
