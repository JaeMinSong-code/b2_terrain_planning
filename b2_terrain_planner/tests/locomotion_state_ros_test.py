#!/usr/bin/env python3
"""별도 ROS domain에서 실제 계획 노드의 상태 입력·만료 계약을 검증한다.

사용: python3 locomotion_state_ros_test.py /path/to/terrain_planner_node
실제 WBC/모터 명령은 발행하지 않는다. 토픽은 /state_contract_test 아래에 격리한다.
"""
import copy
import json
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from b2_terrain_planner.msg import LocomotionPlan, LocomotionState
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, String


def main():
    rclpy.init()
    node = rclpy.create_node("locomotion_state_contract_test")
    plans = []
    pipeline = []
    status_subscription = node.create_subscription(
        String, "/state_contract_test/pipeline", lambda m: pipeline.append(json.loads(m.data)), 100)
    state_pub = node.create_publisher(
        LocomotionState, "/state_contract_test/state", qos_profile_sensor_data)
    map_pub = node.create_publisher(GridMap, "/state_contract_test/map", qos_profile_sensor_data)
    subscription = node.create_subscription(
        LocomotionPlan, "/state_contract_test/plan", plans.append, 10)
    state = LocomotionState()
    state.header.frame_id = "odom"
    state.base_orientation.w = 1.0
    # B2 FK의 넓은 실제 스탠스. hip 바로 아래 명목 발 위치와 구별한다.
    state.com_position.x = -0.0012454836197385707
    state.com_position.y = 0.0025266552409980002
    state.com_position.z = 0.49859984054669726
    feet = [(0.3166114894, 0.191730002), (0.3166114894, -0.191643014),
            (-0.3403885106, 0.191730002), (-0.3403885106, -0.191729998)]
    for leg, (x, y) in enumerate(feet):
        state.foot_position[leg].x = x
        state.foot_position[leg].y = y
        state.contact_estimated[leg] = True
    terrain = GridMap()
    terrain.header.frame_id = "odom"
    terrain.info.resolution = 0.1
    terrain.info.length_x = 2.0
    terrain.info.length_y = 2.0
    terrain.info.pose.orientation.w = 1.0
    terrain.layers = ["elevation"]
    terrain.data = [Float32MultiArray(data=[0.0] * 400)]
    support_patch = "--support-patch" in sys.argv[2:]
    if support_patch:
        terrain.info.resolution = 0.05
        terrain.info.length_x = 4.0
        terrain.info.length_y = 4.0
        # 앞쪽 절반 미관측, 뒤쪽은 발 기준보다 32 mm 낮은 실측 재현 입력.
        heights = []
        for i in range(80 * 80):
            height = -0.032
            if i % 80 < 40:
                height = float("nan")
            heights.append(height)
        terrain.data[0].data = heights

    def pump(seconds, state_message=None, fresh_state=True, send_map=True):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send_map:
                terrain.header.stamp = node.get_clock().now().to_msg()
                map_pub.publish(terrain)
            if state_message is not None:
                if fresh_state:
                    state_message.header.stamp = node.get_clock().now().to_msg()
                state_pub.publish(state_message)
            rclpy.spin_once(node, timeout_sec=0.02)

    args = [sys.argv[1], "--ros-args",
            "-p", "state_topic:=/state_contract_test/state",
            "-p", "map_topic:=/state_contract_test/map",
            "-p", "plan_topic:=/state_contract_test/plan",
            "-r", "~/pipeline_status:=/state_contract_test/pipeline",
            "-p", "publish_markers:=false", "-p", "command:=[0.0,0.0,0.0]",
            "-p", "plan_rate_hz:=5.0", "-p", "time_limit:=3.0",
            "-p", "state_max_age:=4.0", "-p", "map_max_age:=4.0"]
    if support_patch:
        args += ["-p", "support_patch.enabled:=true", "-p", "support_patch.half_width:=0.30"]
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    with tempfile.TemporaryFile(mode="w+") as log:
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, env=environment)
        try:
            pump(0.7)
            for invalid in ("frame", "nan", "quaternion", "old", "future"):
                bad = copy.deepcopy(state)
                bad.header.stamp = node.get_clock().now().to_msg()
                if invalid == "frame":
                    bad.header.frame_id = "unrelated_map"
                elif invalid == "nan":
                    bad.foot_position[0].x = float("nan")
                elif invalid == "quaternion":
                    bad.base_orientation.w = 0.0
                elif invalid == "old":
                    bad.header.stamp.sec -= 10
                elif invalid == "future":
                    bad.header.stamp.sec += 10
                pump(0.4, bad, fresh_state=False)
                assert not plans, f"invalid state accepted: {invalid}"

            # 첫 유효 상태 하나만 반복 발행: 초기값 전달과 중복 계획 차단을 함께 검증한다.
            state.header.stamp = node.get_clock().now().to_msg()
            pump(3.5, state, fresh_state=False)
            assert len(plans) == 1, f"expected one plan, received {len(plans)}"
            plan = plans[0]
            assert plan.header == state.header
            for axis in ("x", "y", "z"):
                assert abs(getattr(plan.base_pose[0].position, axis) -
                           getattr(state.com_position, axis)) < 1e-6
            for leg in range(4):
                assert abs(plan.foot_position[leg].x - feet[leg][0]) < 1e-6
                assert abs(plan.foot_position[leg].y - feet[leg][1]) < 1e-6
            older = copy.deepcopy(state)
            older.header.stamp.nanosec = max(0, older.header.stamp.nanosec - 1)
            pump(0.2, older, fresh_state=False)
            assert len(plans) == 1, "duplicate/out-of-order state replanned"

            # 상태 만료: 지도만 계속 갱신해도 마지막 상태로 새 계획을 만들지 않는다.
            pump(4.2)
            count = len(plans)
            pump(0.5)
            assert len(plans) == count, "stale state replanned"
            # 지도 만료: 새 상태가 와도 옛 지도에 새 계획 시각을 붙이지 않는다.
            pump(4.2, state, send_map=False)
            count = len(plans)
            pump(0.6, state, send_map=False)
            assert len(plans) == count, "stale map replanned"
            reasons = {event["reason"] for event in pipeline}
            assert "state_not_fresh" in reasons
            assert "map_not_fresh" in reasons
            published = [event for event in pipeline if event["event"] == "published"]
            assert published, "missing actual publish event"
            event = published[0]
            assert event["last_published_stamp_ns"] == event["state_stamp_ns"]
            assert event["last_publish_ns"] >= event["solve_end_ns"] >= event["solve_start_ns"] > 0
            count = len(plans)
            state.gait_clock_valid = True
            state.gait_epoch_ns = node.get_clock().now().nanoseconds
            state.gait_time = 0.0
            state.gait_paused = True
            pump(0.4, state)
            assert len(plans) == count
            assert pipeline[-1]["reason"] == "support_unconfirmed"
            state.gait_paused = False
            pump(0.4, state)
            assert len(plans) == count
            assert pipeline[-1]["reason"] == "support_unconfirmed"
            assert pipeline[-1]["leg"] >= 0
            assert process.poll() is None, "planner exited unexpectedly"
            print("PASS: initial CoM/feet, stamp, invalid/duplicate/reversed state, state/map expiry")
            if support_patch:
                print("PASS: plan generated with unknown front terrain and 32 mm rear height mismatch")
        except Exception:
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_subscription(status_subscription)
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
