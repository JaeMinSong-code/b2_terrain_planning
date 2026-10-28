"""분리된 ROS domain 188에서 진행 중 스윙의 토픽 궤적 고정을 검증한다."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.set_message import set_message_fields
from b2_terrain_planner.msg import LocomotionState, LocomotionPlan
from geometry_msgs.msg import TwistStamped
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, String

parser = argparse.ArgumentParser()
parser.add_argument('workspace', type=Path)
parser.add_argument('states', type=Path)
parser.add_argument('--output', type=Path, default=Path('/tmp/committed-swing-ros'))
args = parser.parse_args()
assert os.environ.get('ROS_DOMAIN_ID') == '188'
args.output.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('committed_swing_probe')
state = LocomotionState()
record = next(v for _, v in json.loads(args.states.read_text()) if all(v['contact_estimated']))
set_message_fields(state, record)
state.gait_clock_valid = True
state.gait_epoch_ns = 123
state.gait_time = 0.0
state.active_plan_stamp_ns = 0
state.gait_paused = False
state.contact_confirmed = [True] * 4
height = sum(f.z for f in state.foot_position) / 4
terrain = GridMap()
terrain.header.frame_id = 'odom'
terrain.info.resolution = .05
terrain.info.length_x = 4.0
terrain.info.length_y = 4.0
terrain.info.pose.orientation.w = 1.0
terrain.layers = ['elevation', 'variance']
terrain.data = [Float32MultiArray(data=[height] * 6400), Float32MultiArray(data=[0.0] * 6400)]
sp = node.create_publisher(LocomotionState, '/commit_test/state', qos_profile_sensor_data)
mp = node.create_publisher(GridMap, '/commit_test/map', qos_profile_sensor_data)
cp = node.create_publisher(TwistStamped, '/commit_test/command', 1)
plans, solves = [], []
def accept_plan(message):
    plans.append(message)
    if message.parent_plan_stamp_ns == state.active_plan_stamp_ns:
        state.active_plan_stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec

node.create_subscription(LocomotionPlan, '/commit_test/plan', accept_plan, 10)
node.create_subscription(String, '/terrain_planner_node/solve_status',
                         lambda m: solves.append(json.loads(m.data)), 10)
command = TwistStamped()
command.header.frame_id = 'base_heading'

def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        state.header.stamp = node.get_clock().now().to_msg()
        terrain.header.stamp = state.header.stamp
        command.header.stamp = state.header.stamp
        sp.publish(state)
        mp.publish(terrain)
        cp.publish(command)
        rclpy.spin_once(node, timeout_sec=.01)
        time.sleep(.01)

def point(plan, leg, absolute_time):
    index = round((absolute_time - plan.gait_time) / plan.dt)
    assert abs(plan.gait_time + index * plan.dt - absolute_time) < 1e-8
    return plan.foot_position[4 * index + leg]

def distance(a, b):
    return math.sqrt(sum((getattr(a, axis) - getattr(b, axis))**2 for axis in 'xyz'))

cmd = [str(args.workspace / 'install/b2_terrain_planner/lib/b2_terrain_planner/terrain_planner_node'),
       '--ros-args', '-p', 'state_topic:=/commit_test/state', '-p', 'map_topic:=/commit_test/map',
       '-p', 'plan_topic:=/commit_test/plan', '-p', 'command_topic:=/commit_test/command',
       '-p', 'plan_rate_hz:=10.0', '-p', 'time_limit:=0.1']
with (args.output / 'node.log').open('w') as log:
    process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    try:
        pump(1.5)
        assert plans, solves
        original = plans[-1]
        state.gait_time = .2
        state.contact_estimated = [True, False, False, True]
        state.contact_confirmed = [True, False, False, True]
        for leg in (1, 2):
            state.foot_position[leg] = point(original, leg, .2)
        pump(.5)
        baseline = plans[-1]
        assert baseline.gait_time == .2, solves[-3:]
        command.twist.linear.x = -.15
        command.twist.linear.y = .03
        pump(.5)
        changed = plans[-1]
        assert changed.gait_time == .2 and solves[-1]['published'], solves[-3:]
        error = max(distance(point(original, leg, t), point(changed, leg, t))
                    for leg in (1, 2) for t in (.2, .24, .3, .36, .4))
        future = max(distance(point(baseline, leg, .8), point(changed, leg, .8)) for leg in (0, 3))
        assert error < 1e-6, error
        assert future < 1e-6, future
        assert abs(solves[-1]['command'][0]) < 1e-12, solves[-1]
        state.gait_time = .22
        pump(.4)
        assert plans[-1].gait_time == .22, solves[-3:]
        error2 = max(distance(point(changed, leg, t), point(plans[-1], leg, t))
                     for leg in (1, 2) for t in (.24, .3, .36, .4))
        assert error2 < 1e-6, error2
        state.gait_time = .4
        state.gait_paused = True
        before_landing = len(plans)
        pump(.15)
        assert not any(p.gait_time >= .4 for p in plans[before_landing:]), \
            "unconfirmed landing published a new support plan"
        state.contact_estimated = [True] * 4
        state.contact_confirmed = [True] * 4
        for leg in (1, 2):
            state.foot_position[leg] = point(changed, leg, .4)
        pump(.6)
        assert plans[-1].gait_time == .4, solves[-3:]
        assert abs(solves[-1]['command'][0] + .15) < 1e-12, solves[-1]
        future = max(distance(point(baseline, leg, .8), point(plans[-1], leg, .8)) for leg in (0, 3))
        assert future > 1e-4, future
        assert plans[-1].handover_gait_time == state.gait_time, plans[-1].handover_gait_time
        assert plans[-1].parent_plan_stamp_ns > 0
        result = {'passed': True, 'paused_handover_gait_time': plans[-1].handover_gait_time,
                  'active_curve_change_m': error,
                  'next_replan_curve_change_m': error2, 'future_target_change_m': future,
                  'plans': len(plans)}
        (args.output / 'result.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        process.terminate()
        process.wait(timeout=5)
        (args.output / 'solve.json').write_text(json.dumps(solves, indent=2))
        node.destroy_node()
        rclpy.shutdown()
