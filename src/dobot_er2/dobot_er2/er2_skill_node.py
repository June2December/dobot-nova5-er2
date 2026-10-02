#!/usr/bin/env python3
"""Execute last ER 2 pick/place on a real Nova 5 via dobot_bringup_v3.

Default is dry-run (TF + log only). Set execute_robot:=true after EnableRobot.
Keep current TCP rpy from GetPose. Positions are converted camera → base_link → mm.
"""
import json
import re
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from dobot_msgs_v3.srv import GetPose, MovL, SpeedFactor, Sync, ToolDOExecute


def quat_to_rot(qx, qy, qz, qw):
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5 or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def parse_pose_blob(text):
    nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', text or '')
    if len(nums) < 6:
        raise RuntimeError('GetPose 파싱 실패: {}'.format(text))
    return [float(v) for v in nums[:6]]


class Er2SkillNode(Node):
    def __init__(self):
        super().__init__('er2_skill_node')
        self.declare_parameter('execute_robot', False)
        # manual: 팔만 이동, 그리퍼는 사람이 버튼. continue 서비스로 다음 단계.
        # tool_do: J6 ToolDOExecute (아직 안 되면 manual 쓰세요).
        self.declare_parameter('gripper_mode', 'manual')
        self.declare_parameter('continue_timeout_s', 180.0)
        self.declare_parameter('speed_ratio', 10)
        self.declare_parameter('approach_z_m', 0.08)
        self.declare_parameter('grip_z_m', 0.03)
        self.declare_parameter('tool_do_index', 1)
        self.declare_parameter('gripper_close', 1)
        self.declare_parameter('gripper_open', 0)
        self.declare_parameter('target_frame', 'base_link')
        self._pick = None
        self._place = None
        self._busy = False
        self._continue = threading.Event()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/er2/pick_pose', self._on_pick, 10)
        self.create_subscription(PoseStamped, '/er2/place_pose', self._on_place, 10)
        self.pub_status = self.create_publisher(String, '/er2/skill_status', 10)
        self.create_service(Trigger, '/er2/execute', self._on_execute)
        self.create_service(Trigger, '/er2/continue', self._on_continue)
        self.cli_speed = self.create_client(SpeedFactor, '/dobot_bringup_v3/srv/SpeedFactor')
        self.cli_move = self.create_client(MovL, '/dobot_bringup_v3/srv/MovL')
        self.cli_sync = self.create_client(Sync, '/dobot_bringup_v3/srv/Sync')
        self.cli_pose = self.create_client(GetPose, '/dobot_bringup_v3/srv/GetPose')
        self.cli_tool = self.create_client(ToolDOExecute, '/dobot_bringup_v3/srv/ToolDOExecute')
        self.get_logger().info(
            'er2_skill ready execute_robot={} gripper_mode={}'.format(
                self.get_parameter('execute_robot').value,
                self.get_parameter('gripper_mode').value))

    def _on_continue(self, request, response):
        self._continue.set()
        response.success = True
        response.message = 'continue'
        return response

    def _wait_gripper(self, action):
        """action is 'close' or 'open'."""
        mode = str(self.get_parameter('gripper_mode').value).strip().lower()
        if mode == 'tool_do':
            status = int(self.get_parameter('gripper_close' if action == 'close' else 'gripper_open').value)
            self._tool(status)
            return
        hint = '그리퍼를 {} 한 뒤: ros2 service call /er2/continue std_srvs/srv/Trigger'.format(
            '닫고' if action == 'close' else '열고')
        self.get_logger().warn(hint)
        self.pub_status.publish(String(data=hint))
        self._continue.clear()
        timeout = float(self.get_parameter('continue_timeout_s').value)
        if not self._continue.wait(timeout):
            raise RuntimeError('그리퍼 대기 시간 초과. /er2/continue 를 호출하세요.')

    def _on_pick(self, msg):
        self._pick = msg

    def _on_place(self, msg):
        self._place = msg

    def _on_execute(self, request, response):
        if self._busy:
            response.success = False
            response.message = '이미 실행 중입니다.'
            return response
        if self._pick is None or self._place is None:
            response.success = False
            response.message = '/er2/plan 을 먼저 호출하세요.'
            return response
        self._busy = True
        thread = threading.Thread(target=self._run_job, daemon=True)
        thread.start()
        response.success = True
        response.message = 'started'
        return response

    def _run_job(self):
        try:
            summary = self._execute()
            self.pub_status.publish(String(data=json.dumps(summary, ensure_ascii=False)))
            self.get_logger().info(str(summary))
        except Exception as exc:
            self.get_logger().error(str(exc))
            self.pub_status.publish(String(data=str(exc)))
        finally:
            self._busy = False

    def _to_base_mm(self, pose_stamped):
        target = str(self.get_parameter('target_frame').value)
        tf = self.tf_buffer.lookup_transform(
            target, pose_stamped.header.frame_id, rclpy.time.Time())
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = quat_to_rot(q.x, q.y, q.z, q.w)
        p = np.array([
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            pose_stamped.pose.position.z,
        ])
        p_base = rot @ p + np.array([t.x, t.y, t.z])
        return p_base * 1000.0

    def _call(self, client, request, timeout=20.0):
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('서비스 없음: {}'.format(client.srv_name))
        future = client.call_async(request)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError(client.srv_name)
            time.sleep(0.05)
        result = future.result()
        if result is None:
            raise RuntimeError('{} 응답 없음'.format(client.srv_name))
        return result

    def _movl(self, xyz_mm, rpy, label):
        req = MovL.Request()
        req.x, req.y, req.z = [float(v) for v in xyz_mm]
        req.rx, req.ry, req.rz = [float(v) for v in rpy]
        req.param_value = []
        self.get_logger().info('MovL {} x={:.1f} y={:.1f} z={:.1f}'.format(label, req.x, req.y, req.z))
        self._call(self.cli_move, req)
        self._call(self.cli_sync, Sync.Request())

    def _tool(self, status):
        req = ToolDOExecute.Request()
        req.index = int(self.get_parameter('tool_do_index').value)
        req.status = int(status)
        self._call(self.cli_tool, req)
        time.sleep(0.4)

    def _execute(self):
        pick_mm = self._to_base_mm(self._pick)
        place_mm = self._to_base_mm(self._place)
        approach = float(self.get_parameter('approach_z_m').value) * 1000.0
        grip = float(self.get_parameter('grip_z_m').value) * 1000.0
        summary = {
            'pick_base_mm': [round(float(v), 1) for v in pick_mm],
            'place_base_mm': [round(float(v), 1) for v in place_mm],
            'execute_robot': bool(self.get_parameter('execute_robot').value),
        }
        if not bool(self.get_parameter('execute_robot').value):
            summary['note'] = 'dry-run. execute_robot:=true 이면 실기가 움직입니다.'
            return summary
        pose_req = GetPose.Request()
        pose_req.user = 0
        pose_req.tool = 0
        pose_res = self._call(self.cli_pose, pose_req)
        rpy = parse_pose_blob(pose_res.pose)[3:6]
        speed = SpeedFactor.Request()
        speed.ratio = int(self.get_parameter('speed_ratio').value)
        self._call(self.cli_speed, speed)
        pick_app = np.array(pick_mm)
        pick_app[2] += approach
        pick_grip = np.array(pick_mm)
        pick_grip[2] += grip
        place_app = np.array(place_mm)
        place_app[2] += approach
        place_grip = np.array(place_mm)
        place_grip[2] += grip
        self._movl(pick_app, rpy, 'pick_approach')
        self._movl(pick_grip, rpy, 'pick')
        self._wait_gripper('close')
        self._movl(pick_app, rpy, 'pick_lift')
        self._movl(place_app, rpy, 'place_approach')
        self._movl(place_grip, rpy, 'place')
        self._wait_gripper('open')
        self._movl(place_app, rpy, 'place_retreat')
        summary['done'] = True
        summary['rpy'] = rpy
        summary['gripper_mode'] = str(self.get_parameter('gripper_mode').value)
        return summary


def main(args=None):
    rclpy.init(args=args)
    node = Er2SkillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
