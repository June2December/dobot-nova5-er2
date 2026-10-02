#!/usr/bin/env python3
"""D405 RGB-D + Gemini Robotics ER 2 → pick/place pixels → camera-frame 3D.

Does not talk to Isaac Sim. Robot motion is a separate node (er2_skill_node).
"""
import json

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from dobot_er2.er2_client import (
    PICK_PLACE_TOOL,
    function_calls,
    infer_pick_place,
    make_client,
    new_observation_id,
    norm_to_pixel,
    parse_json_maybe,
    point_yx,
    response_text,
)


def image_msg_to_bgr(msg):
    if msg.encoding in ('bgr8', '8UC3'):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
    if msg.encoding in ('rgb8',):
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    raise ValueError('unsupported color encoding {}'.format(msg.encoding))


def depth_msg_to_m(msg):
    if msg.encoding in ('16UC1', 'mono16'):
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        return raw.astype(np.float32) / 1000.0
    if msg.encoding in ('32FC1',):
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    raise ValueError('unsupported depth encoding {}'.format(msg.encoding))


def bgr_to_png(bgr):
    ok, buf = cv2.imencode('.png', bgr)
    if not ok:
        raise RuntimeError('png encode failed')
    return buf.tobytes()


def sample_depth_m(depth_m, u, v, radius=4):
    h, w = depth_m.shape
    u0 = max(0, u - radius)
    v0 = max(0, v - radius)
    u1 = min(w, u + radius + 1)
    v1 = min(h, v + radius + 1)
    patch = depth_m[v0:v1, u0:u1]
    valid = patch[(patch > 0.05) & (patch < 2.0)]
    if valid.size < 5:
        return None
    return float(np.median(valid))


def unproject(u, v, z_m, fx, fy, cx, cy):
    return np.array([
        (u - cx) / fx * z_m,
        (v - cy) / fy * z_m,
        z_m,
    ], dtype=np.float64)


class Er2BridgeNode(Node):
    def __init__(self):
        super().__init__('er2_bridge_node')
        self.declare_parameter('model', 'gemini-robotics-er-2-preview')
        self.declare_parameter(
            'instruction',
            'Pick up the cup and place it 8cm to the right on the same table. '
            'pick is the cup body, not the table. place is empty table.')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic', '/camera/color/camera_info')
        qos = qos_profile_sensor_data
        self._color = None
        self._depth = None
        self._info = None
        self._last_plan = None
        self.create_subscription(Image, self.get_parameter('color_topic').value, self._on_color, qos)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self._on_depth, qos)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, self._on_info, qos)
        self.pub_pick = self.create_publisher(PoseStamped, '/er2/pick_pose', 10)
        self.pub_place = self.create_publisher(PoseStamped, '/er2/place_pose', 10)
        self.pub_overlay = self.create_publisher(Image, '/er2/overlay', 10)
        self.pub_status = self.create_publisher(String, '/er2/status', 10)
        self.create_service(Trigger, '/er2/plan', self._on_plan)
        self.get_logger().info('er2_bridge ready. call /er2/plan after the camera is streaming')

    def _on_color(self, msg):
        self._color = msg

    def _on_depth(self, msg):
        self._depth = msg

    def _on_info(self, msg):
        self._info = msg

    def _on_plan(self, request, response):
        try:
            plan = self._plan_once()
            self._last_plan = plan
            response.success = True
            response.message = json.dumps(plan, ensure_ascii=False)
            self.pub_status.publish(String(data=response.message))
        except Exception as exc:
            self.get_logger().error(str(exc))
            response.success = False
            response.message = str(exc)
            self.pub_status.publish(String(data=response.message))
        return response

    def _plan_once(self):
        if self._color is None or self._depth is None or self._info is None:
            raise RuntimeError('카메라 토픽이 아직 없습니다. cup_pose 런치를 먼저 켜세요.')
        bgr = image_msg_to_bgr(self._color)
        depth_m = depth_msg_to_m(self._depth)
        info = self._info
        h, w = bgr.shape[:2]
        obs_id = new_observation_id()
        instruction = str(self.get_parameter('instruction').value)
        prompt = (
            'You control a real Dobot Nova 5 tabletop arm via pick_place. '
            'The image is from a RealSense D405 looking at the table. '
            'Select the cup (or the main graspable object) and a clear place point on the table. '
            'Only visible RGB is available. Do not use world coordinates. '
            'All image coordinates are normalized to 0..1000 in [y,x] order. '
            'Issue at most ONE function call for this observation. '
            'pick must be on the object, not the table. place must be empty table. '
            'Observation ID: {}\nGoal: {}'
        ).format(obs_id, instruction)
        api = make_client()
        model = str(self.get_parameter('model').value)
        response, elapsed = infer_pick_place(
            api, model, bgr_to_png(bgr), prompt, tools=[PICK_PLACE_TOOL])
        text = response_text(response)
        calls = function_calls(response)
        if not calls:
            raise RuntimeError('ER 2가 pick_place를 호출하지 않았습니다: {}'.format(text[:400]))
        call = calls[0]
        if getattr(call, 'name', '') != 'pick_place':
            raise RuntimeError('허용되지 않은 도구: {}'.format(call.name))
        args = call.arguments
        if isinstance(args, str):
            args = parse_json_maybe(args)
        pick_yx = point_yx(args['pick'])
        place_yx = point_yx(args['place'])
        model_obs = str(args.get('observation_id', ''))
        if model_obs and model_obs != obs_id:
            self.get_logger().warn('observation_id mismatch model={} local={}'.format(model_obs, obs_id))
        fx, fy = float(info.k[0]), float(info.k[4])
        cx, cy = float(info.k[2]), float(info.k[5])
        pick_uv = norm_to_pixel(pick_yx, w, h)
        place_uv = norm_to_pixel(place_yx, w, h)
        pick_z = sample_depth_m(depth_m, pick_uv[0], pick_uv[1])
        place_z = sample_depth_m(depth_m, place_uv[0], place_uv[1])
        if pick_z is None or place_z is None:
            raise RuntimeError('pick/place 픽셀에 유효 depth가 없습니다. pick={} place={}'.format(pick_uv, place_uv))
        pick_xyz = unproject(pick_uv[0], pick_uv[1], pick_z, fx, fy, cx, cy)
        place_xyz = unproject(place_uv[0], place_uv[1], place_z, fx, fy, cx, cy)
        header = self._color.header
        pick_pose = self._xyz_pose(header, pick_xyz)
        place_pose = self._xyz_pose(header, place_xyz)
        self.pub_pick.publish(pick_pose)
        self.pub_place.publish(place_pose)
        overlay = bgr.copy()
        cv2.circle(overlay, pick_uv, 8, (0, 0, 255), 2)
        cv2.putText(overlay, 'pick', (pick_uv[0] + 10, pick_uv[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.circle(overlay, place_uv, 8, (0, 255, 0), 2)
        cv2.putText(overlay, 'place', (place_uv[0] + 10, place_uv[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        self.pub_overlay.publish(self._bgr_msg(header, overlay))
        plan = {
            'observation_id': obs_id,
            'model': model,
            'elapsed_s': round(elapsed, 2),
            'text': text,
            'pick_yx': pick_yx,
            'place_yx': place_yx,
            'pick_uv': list(pick_uv),
            'place_uv': list(place_uv),
            'pick_xyz_cam_m': [round(float(v), 4) for v in pick_xyz],
            'place_xyz_cam_m': [round(float(v), 4) for v in place_xyz],
        }
        self.get_logger().info('ER2 plan {}'.format(json.dumps(plan, ensure_ascii=False)))
        return plan

    def _xyz_pose(self, header, xyz):
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        return msg

    def _bgr_msg(self, header, bgr):
        msg = Image()
        msg.header = header
        msg.height = bgr.shape[0]
        msg.width = bgr.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = Er2BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
