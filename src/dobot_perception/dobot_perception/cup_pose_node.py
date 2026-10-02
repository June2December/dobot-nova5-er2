#!/usr/bin/env python3
"""D405 RGB-D → YOLO cup → depth 6D pose.

Position is the 3D centroid of valid depth pixels in the box.
Orientation is the PCA long axis of that cloud (cup standing axis),
stabilized across frames to avoid 180° flips.
"""

import math
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Header, String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker

# Latest Ultralytics SOTA (2026), extra-large COCO detect weights.
DEFAULT_MODEL_NAME = 'yolo26x.pt'


def rpy_from_rot(rm):
    ry = math.asin(max(-1.0, min(1.0, -rm[2, 0])))
    rx = math.atan2(rm[2, 1], rm[2, 2])
    rz = math.atan2(rm[1, 0], rm[0, 0])
    return rx, ry, rz


def quat_to_rot(qx, qy, qz, qw):
    """Unit quaternion (x,y,z,w) → 3x3 rotation."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def rot_to_quat(rm):
    tr = float(np.trace(rm))
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rm[2, 1] - rm[1, 2]) / s
        qy = (rm[0, 2] - rm[2, 0]) / s
        qz = (rm[1, 0] - rm[0, 1]) / s
    elif rm[0, 0] > rm[1, 1] and rm[0, 0] > rm[2, 2]:
        s = math.sqrt(1.0 + rm[0, 0] - rm[1, 1] - rm[2, 2]) * 2.0
        qw = (rm[2, 1] - rm[1, 2]) / s
        qx = 0.25 * s
        qy = (rm[0, 1] + rm[1, 0]) / s
        qz = (rm[0, 2] + rm[2, 0]) / s
    elif rm[1, 1] > rm[2, 2]:
        s = math.sqrt(1.0 + rm[1, 1] - rm[0, 0] - rm[2, 2]) * 2.0
        qw = (rm[0, 2] - rm[2, 0]) / s
        qx = (rm[0, 1] + rm[1, 0]) / s
        qy = 0.25 * s
        qz = (rm[1, 2] + rm[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rm[2, 2] - rm[0, 0] - rm[1, 1]) * 2.0
        qw = (rm[1, 0] - rm[0, 1]) / s
        qx = (rm[0, 2] + rm[2, 0]) / s
        qy = (rm[1, 2] + rm[2, 1]) / s
        qz = 0.25 * s
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    return qx / n, qy / n, qz / n, qw / n


def numpy_to_image(header, array, encoding):
    msg = Image()
    msg.header = header
    msg.height = int(array.shape[0])
    msg.width = int(array.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    if encoding == 'bgr8':
        msg.step = msg.width * 3
        if array.dtype != np.uint8:
            array = array.astype(np.uint8)
        if array.ndim == 2:
            array = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
    elif encoding == '16UC1':
        msg.step = msg.width * 2
        if array.dtype != np.uint16:
            array = array.astype(np.uint16)
    else:
        raise ValueError(encoding)
    msg.data = array.tobytes()
    return msg


def _normalize(v):
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        return v * 0.0
    return v / n


def _frame_from_z(z_axis, prev_rot=None, toward_camera=True):
    """Build a right-handed rotation whose third column is z_axis."""
    z_axis = _normalize(z_axis)
    if float(np.linalg.norm(z_axis)) < 1e-9:
        z_axis = np.array([0.0, 0.0, -1.0])

    if prev_rot is not None:
        prev_z = prev_rot[:, 2]
        if float(np.dot(z_axis, prev_z)) < 0.0:
            z_axis = -z_axis
        x_axis = prev_rot[:, 0] - z_axis * float(np.dot(prev_rot[:, 0], z_axis))
        if float(np.linalg.norm(x_axis)) < 1e-6:
            x_axis = prev_rot[:, 1] - z_axis * float(np.dot(prev_rot[:, 1], z_axis))
        x_axis = _normalize(x_axis)
        y_axis = _normalize(np.cross(z_axis, x_axis))
        x_axis = _normalize(np.cross(y_axis, z_axis))
        rot = np.stack([x_axis, y_axis, z_axis], axis=1)
        alpha = 0.35
        blended = (1.0 - alpha) * prev_rot + alpha * rot
        u, _, vt2 = np.linalg.svd(blended)
        rot = u @ vt2
        if np.linalg.det(rot) < 0:
            u[:, -1] *= -1
            rot = u @ vt2
        return rot

    if toward_camera and z_axis[2] > 0:
        z_axis = -z_axis
    if toward_camera and abs(z_axis[2]) < 0.25 and z_axis[1] > 0:
        z_axis = -z_axis
    z_axis = _normalize(z_axis)
    x_ref = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(z_axis, x_ref))) > 0.9:
        x_ref = np.array([0.0, 1.0, 0.0])
    x_axis = _normalize(np.cross(x_ref, z_axis))
    y_axis = _normalize(np.cross(z_axis, x_axis))
    return np.stack([x_axis, y_axis, z_axis], axis=1)


def pca_axes(pts):
    """Return SVD axes of a centered cloud: rows of vt, longest first."""
    centered = pts - pts.mean(axis=0)
    _, s, vt = np.linalg.svd(centered, full_matrices=False)
    return vt, s


def pca_orientation(pts, prev_rot=None, which='max'):
    """z = longest (max) or shortest (min) principal axis."""
    vt, _ = pca_axes(pts)
    z_axis = vt[0] if which != 'min' else vt[-1]
    return _frame_from_z(z_axis, prev_rot, toward_camera=True)


def view_orientation(centroid, prev_rot=None):
    """z = ray from camera origin through the object (optical +Z-ish)."""
    return _frame_from_z(centroid, prev_rot, toward_camera=False)


def fit_plane_ransac(pts, iters=80, thresh=0.008, min_inliers=30):
    """Return unit plane normal or None. n·(p-p0)=0."""
    npts = int(pts.shape[0])
    if npts < 3:
        return None
    rng = np.random.default_rng(0)
    best_n, best_count = None, 0
    for _ in range(iters):
        idx = rng.choice(npts, 3, replace=False)
        p0, p1, p2 = pts[idx]
        nvec = np.cross(p1 - p0, p2 - p0)
        ln = float(np.linalg.norm(nvec))
        if ln < 1e-9:
            continue
        nvec = nvec / ln
        dist = np.abs((pts - p0) @ nvec)
        count = int(np.sum(dist < thresh))
        if count > best_count:
            best_count = count
            best_n = nvec
    if best_n is None or best_count < min_inliers:
        return None
    return best_n


def plane_orientation(pts, prev_rot=None):
    """z = RANSAC plane normal in the ROI (table or cup face)."""
    nvec = fit_plane_ransac(pts)
    if nvec is None:
        vt, _ = pca_axes(pts)
        nvec = vt[-1]
    return _frame_from_z(nvec, prev_rot, toward_camera=True)


def source_models_dir():
    return os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', 'models'))


def resolve_model_path(model):
    """Resolve a weights name/path to an absolute file path."""
    model = str(model).strip() or DEFAULT_MODEL_NAME
    if os.path.isabs(model) and os.path.isfile(model):
        return model
    if os.path.isfile(model):
        return os.path.abspath(model)
    name = os.path.basename(model)
    candidates = [os.path.join(source_models_dir(), name)]
    try:
        share = get_package_share_directory('dobot_perception')
        candidates.insert(0, os.path.join(share, 'models', name))
    except Exception:
        pass
    # Symlink-install: package module may live under install/.../site-packages.
    # Also check workspace src path relative to AMENT_PREFIX_PATH.
    ws = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)[0]
    if ws:
        candidates.append(os.path.join(
            os.path.dirname(ws), 'src', 'dobot_perception', 'models', name))
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    dest_dir = source_models_dir()
    os.makedirs(dest_dir, exist_ok=True)
    return os.path.join(dest_dir, name)


class CupPoseNode(Node):
    def __init__(self):
        super().__init__('cup_pose_node')
        self.declare_parameter('serial', '')
        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('conf', 0.35)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('show_window', True)
        self.declare_parameter('publish_camera', True)
        self.declare_parameter('target_classes', ['cup', 'bottle', 'wine glass', 'bowl'])
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('debug_image_path', '/tmp/cup_pose_debug.png')
        self.declare_parameter('model', DEFAULT_MODEL_NAME)
        self.declare_parameter('orient_mode', 'pca_max')
        # Cylinder glyph stands along +Z of this frame (table/world up in MoveIt).
        # Empty string: fall back to optical -Y (up in the image).
        self.declare_parameter('cylinder_up_frame', 'base_link')

        self.frame_id = self.get_parameter('frame_id').value
        self.show_window = bool(self.get_parameter('show_window').value)
        self.publish_camera = bool(self.get_parameter('publish_camera').value)
        self.conf = float(self.get_parameter('conf').value)
        self.debug_image_path = self.get_parameter('debug_image_path').value
        self.target_classes = [
            str(x).lower() for x in self.get_parameter('target_classes').value
        ]
        self._prev_rot = None
        self._miss_frames = 0
        self.orient_mode = str(self.get_parameter('orient_mode').value).strip().lower()
        self.get_logger().info('orient_mode={}'.format(self.orient_mode))

        from ultralytics import YOLO
        model_path = resolve_model_path(self.get_parameter('model').value)
        device = self.get_parameter('device').value
        model_dir = os.path.dirname(model_path) or '.'
        os.makedirs(model_dir, exist_ok=True)
        # Keep YOLO artifact dirs out of the workspace root.
        self._yolo_project = os.path.join('/tmp', 'dobot_perception_yolo')
        os.makedirs(self._yolo_project, exist_ok=True)

        self.get_logger().info('loading YOLO {} on {}'.format(model_path, device))
        cwd = os.getcwd()
        try:
            os.chdir(model_dir)
            # Load by basename so Ultralytics downloads next to models/ if missing.
            self.yolo = YOLO(os.path.basename(model_path))
        finally:
            os.chdir(cwd)
        self.device = device
        name_to_id = {v: k for k, v in self.yolo.names.items()}
        self.class_ids = []
        for name in self.target_classes:
            if name in name_to_id:
                self.class_ids.append(name_to_id[name])
        if not self.class_ids:
            raise RuntimeError('no target classes in model: {}'.format(self.target_classes))
        self.get_logger().info('detecting classes {}'.format(
            [(i, self.yolo.names[i]) for i in self.class_ids]))

        qos = qos_profile_sensor_data
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', qos)
        self.pub_depth = self.create_publisher(
            Image, '/camera/aligned_depth_to_color/image_raw', qos)
        self.pub_info = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        self.pub_debug = self.create_publisher(Image, '/perception/debug_image', qos)
        self.pub_pose = self.create_publisher(PoseStamped, '/perception/cup_pose', 10)
        self.pub_conf = self.create_publisher(Float32, '/perception/cup_confidence', 10)
        self.pub_label = self.create_publisher(String, '/perception/cup_label', 10)
        self.pub_dist = self.create_publisher(Float32, '/perception/cup_distance', 10)
        self.pub_offset = self.create_publisher(
            Vector3Stamped, '/perception/cup_offset_camera', 10)
        self.pub_status = self.create_publisher(String, '/perception/cup_status', 10)
        self.pub_marker = self.create_publisher(Marker, '/perception/cup_marker', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._start_camera()
        self.saved_debug = False
        if self.show_window:
            cv2.namedWindow('D405 cup pose', cv2.WINDOW_NORMAL)
        self.timer = self.create_timer(1.0 / 15.0, self.tick)
        self.get_logger().info('D405 cup pose running (serial {})'.format(self.serial))

    def _start_camera(self):
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        serial = str(self.get_parameter('serial').value)
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if serial:
            cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        last_err = None
        for attempt in range(1, 6):
            try:
                profile = self.pipeline.start(cfg)
                break
            except RuntimeError as exc:
                last_err = exc
                msg = str(exc)
                self.get_logger().warn(
                    'camera open failed ({}/5): {}'.format(attempt, msg))
                if 'busy' in msg.lower() or '16' in msg:
                    time.sleep(1.0)
                    continue
                raise
        else:
            raise RuntimeError(
                'D405 open failed after retries: {}. '
                'Another process is using the camera '
                '(realsense-viewer, depth_stream.py, previous ros2 launch). '
                'Check with: fuser -v /dev/video0'.format(last_err))

        dev = profile.get_device()
        self.serial = dev.get_info(rs.camera_info.serial_number)
        self.depth_scale = dev.first_depth_sensor().get_depth_scale()
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intr = color_stream.get_intrinsics()
        self.align = rs.align(rs.stream.color)
        self.camera_info = CameraInfo()
        self.camera_info.width = self.intr.width
        self.camera_info.height = self.intr.height
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.k = [
            self.intr.fx, 0.0, self.intr.ppx,
            0.0, self.intr.fy, self.intr.ppy,
            0.0, 0.0, 1.0,
        ]
        self.camera_info.p = [
            self.intr.fx, 0.0, self.intr.ppx, 0.0,
            0.0, self.intr.fy, self.intr.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.camera_info.d = list(self.intr.coeffs[:5])
        self.get_logger().info(
            'opened {}  {}x{}  depth_scale={:.6f}  fx={:.1f}'.format(
                self.serial, width, height, self.depth_scale, self.intr.fx))

    def _pose_from_depth(self, depth_m, box):
        x1, y1, x2, y2 = box
        w, h = x2 - x1, y2 - y1
        sx1 = int(x1 + 0.2 * w)
        sy1 = int(y1 + 0.2 * h)
        sx2 = int(x2 - 0.2 * w)
        sy2 = int(y2 - 0.2 * h)
        if sx2 <= sx1 or sy2 <= sy1:
            sx1, sy1, sx2, sy2 = x1, y1, x2, y2
        roi = depth_m[sy1:sy2, sx1:sx2]
        ys, xs = np.where((roi > 0.05) & (roi < 2.0))
        if len(xs) < 30:
            return None
        us = xs + sx1
        vs = ys + sy1
        zs = roi[ys, xs]
        pts = np.stack([
            (us - self.intr.ppx) / self.intr.fx * zs,
            (vs - self.intr.ppy) / self.intr.fy * zs,
            zs,
        ], axis=1)
        centroid = np.median(pts, axis=0)
        try:
            mode = self.orient_mode
            if mode in ('pca', 'pca_max'):
                rot = pca_orientation(pts, self._prev_rot, which='max')
            elif mode == 'pca_min':
                rot = pca_orientation(pts, self._prev_rot, which='min')
            elif mode in ('view', 'view_ray'):
                rot = view_orientation(centroid, self._prev_rot)
            elif mode in ('plane', 'plane_ransac'):
                rot = plane_orientation(pts, self._prev_rot)
            elif mode in ('identity', 'none'):
                rot = np.eye(3)
            else:
                self.get_logger().warn(
                    'unknown orient_mode {}, using pca_max'.format(mode),
                    throttle_duration_sec=5.0)
                rot = pca_orientation(pts, self._prev_rot, which='max')
        except Exception:
            rot = self._prev_rot if self._prev_rot is not None else np.eye(3)
        self._prev_rot = rot
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in centroid]
        qx, qy, qz, qw = rot_to_quat(rot)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose, rot, int(np.median(us)), int(np.median(vs))

    def tick(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError as exc:
            self.get_logger().warn('frame timeout: {}'.format(exc))
            return
        frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return
        bgr = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self.depth_scale
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self.frame_id)

        if self.publish_camera:
            self.camera_info.header = header
            self.pub_info.publish(self.camera_info)
            self.pub_color.publish(numpy_to_image(header, bgr, 'bgr8'))
            self.pub_depth.publish(numpy_to_image(header, depth_raw, '16UC1'))

        results = self.yolo.predict(
            bgr,
            verbose=False,
            conf=self.conf,
            device=self.device,
            classes=self.class_ids,
            save=False,
            save_txt=False,
            save_crop=False,
            project=self._yolo_project,
            name='predict',
            exist_ok=True,
        )
        debug = bgr.copy()
        best = None
        if results and results[0].boxes is not None and len(results[0].boxes):
            boxes = results[0].boxes
            confs = boxes.conf.cpu().numpy()
            clss = boxes.cls.cpu().numpy().astype(int)
            xyxy = boxes.xyxy.cpu().numpy().astype(int)
            idx = int(np.argmax(confs))
            best = {
                'conf': float(confs[idx]),
                'cls': int(clss[idx]),
                'box': xyxy[idx].tolist(),
            }

        pose_msg = None
        if best is not None:
            x1, y1, x2, y2 = best['box']
            label = self.yolo.names[best['cls']]
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 220, 0), 2)
            estimated = self._pose_from_depth(depth_m, best['box'])
            if estimated is not None:
                self._miss_frames = 0
                pose, rot, cu, cv_ = estimated
                px, py, pz = pose.position.x, pose.position.y, pose.position.z
                dist = math.sqrt(px * px + py * py + pz * pz)
                # optical frame: +X right, +Y down, +Z forward
                yaw_cam = math.degrees(math.atan2(px, pz))
                pitch_cam = math.degrees(math.atan2(-py, pz))
                rx, ry, rz = rpy_from_rot(rot)
                lr = 'RIGHT' if px >= 0 else 'LEFT'
                ud = 'DOWN' if py >= 0 else 'UP'

                pose_msg = PoseStamped(header=header, pose=pose)
                self.pub_pose.publish(pose_msg)
                self.pub_conf.publish(Float32(data=best['conf']))
                self.pub_label.publish(String(data=label))
                self.pub_dist.publish(Float32(data=float(dist)))
                off = Vector3Stamped()
                off.header = header
                off.vector.x, off.vector.y, off.vector.z = px, py, pz
                self.pub_offset.publish(off)

                status = (
                    'object={label} conf={conf:.0%} | '
                    'distance={dist:.3f}m ({dist_mm:.0f}mm) | '
                    'cup_xyz_in_camera=[{px:+.3f}, {py:+.3f}, {pz:+.3f}] m '
                    '(CUP pose in camera frame, NOT camera pose; '
                    'X right / Y down / Z forward) | '
                    'from_camera_center: {lr} {dx:.0f}mm, {ud} {dy:.0f}mm, '
                    'forward {dz:.0f}mm | '
                    'view_angle: yaw={yaw:+.1f}deg pitch={pitch:+.1f}deg | '
                    'orient={orient} | '
                    'object_rpy=[{rx:.1f}, {ry:.1f}, {rz:.1f}] deg'
                ).format(
                    label=label, conf=best['conf'], dist=dist, dist_mm=dist * 1000.0,
                    px=px, py=py, pz=pz, lr=lr, ud=ud,
                    dx=abs(px) * 1000.0, dy=abs(py) * 1000.0, dz=pz * 1000.0,
                    yaw=yaw_cam, pitch=pitch_cam, orient=self.orient_mode,
                    rx=math.degrees(rx), ry=math.degrees(ry), rz=math.degrees(rz),
                )
                self.pub_status.publish(String(data=status))
                self._publish_tf(header, pose)
                self._publish_marker(header, pose)

                line1 = '{} {:.0%}  dist={:.3f}m'.format(label, best['conf'], dist)
                line2 = 'cup@cam [{:+.3f} {:+.3f} {:+.3f}]'.format(px, py, pz)
                line3 = '{} {:.0f}mm  {} {:.0f}mm  fwd {:.0f}mm'.format(
                    lr, abs(px) * 1000.0, ud, abs(py) * 1000.0, pz * 1000.0)
                line4 = 'orient={}  yaw {:+.1f}  pitch {:+.1f}'.format(
                    self.orient_mode, yaw_cam, pitch_cam)
                cv2.putText(debug, line1, (x1, max(20, y1 - 28)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2, cv2.LINE_AA)
                cv2.putText(debug, line2, (x1, max(40, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)
                cv2.putText(debug, line3, (x1, min(debug.shape[0] - 24, y2 + 18)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)
                cv2.putText(debug, line4, (x1, min(debug.shape[0] - 8, y2 + 36)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)
                self._draw_axes(debug, pose.position, rot)
                cv2.circle(debug, (cu, cv_), 4, (0, 0, 255), -1)
                self.get_logger().info(status, throttle_duration_sec=1.0)
            else:
                self._miss_frames += 1
                miss = '{} {:.0%} detected but no valid depth'.format(
                    label, best['conf'])
                self.pub_status.publish(String(data=miss))
                cv2.putText(debug, miss, (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 180, 255), 1, cv2.LINE_AA)
        else:
            self._miss_frames += 1
            miss = 'no target in view (looking for cup/bottle/wine glass/bowl)'
            self.pub_status.publish(String(data=miss))
            cv2.putText(debug, miss, (16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 255), 2, cv2.LINE_AA)

        if self._miss_frames > 10:
            self._prev_rot = None

        self.pub_debug.publish(numpy_to_image(header, debug, 'bgr8'))
        if pose_msg is not None and not self.saved_debug:
            cv2.imwrite(self.debug_image_path, debug)
            self.saved_debug = True
            self.get_logger().info('wrote debug image {}'.format(self.debug_image_path))
        if self.show_window:
            cv2.imshow('D405 cup pose', debug)
            cv2.waitKey(1)

    def _draw_axes(self, img, origin, rot, length=0.05):
        axes = {
            (0, 0, 255): rot[:, 0] * length,
            (0, 255, 0): rot[:, 1] * length,
            (255, 0, 0): rot[:, 2] * length,
        }
        o = np.array([origin.x, origin.y, origin.z], dtype=np.float64)

        def project(p):
            if p[2] <= 1e-6:
                return None
            u = int(self.intr.fx * p[0] / p[2] + self.intr.ppx)
            v = int(self.intr.fy * p[1] / p[2] + self.intr.ppy)
            return u, v

        p0 = project(o)
        if p0 is None:
            return
        for color, vec in axes.items():
            p1 = project(o + vec)
            if p1 is not None:
                cv2.line(img, p0, p1, color, 2)

    def _publish_tf(self, header, pose):
        tf = TransformStamped()
        tf.header = header
        tf.child_frame_id = 'cup'
        tf.transform.translation.x = pose.position.x
        tf.transform.translation.y = pose.position.y
        tf.transform.translation.z = pose.position.z
        tf.transform.rotation = pose.orientation
        self.tf.sendTransform(tf)

    def _world_up_in_frame(self, frame_id):
        """+Z of cylinder_up_frame in frame_id, or None if that TF is missing."""
        up_frame = str(self.get_parameter('cylinder_up_frame').value).strip()
        if not up_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, up_frame, rclpy.time.Time())
            q = tf.transform.rotation
            rot = quat_to_rot(q.x, q.y, q.z, q.w)
            up = rot @ np.array([0.0, 0.0, 1.0])
            if float(np.linalg.norm(up)) > 1e-6:
                return _normalize(up)
        except Exception:
            return None
        return None

    def _publish_marker(self, header, pose):
        """Cup glyph. Not the orient_mode rotation.

        ROS cylinders use pose Z as height. view/plane/identity put object Z
        along the camera ray, so copying that pose made MoveIt look at the cap.
        With base_link: stand the cylinder on world +Z. Without it (camera-only
        RViz): a sphere at the centroid, so the camera XY grid is not mistaken
        for a table the cup should sit on.
        """
        m = Marker()
        m.header = header
        m.ns = 'cup'
        m.id = 0
        m.action = Marker.ADD
        m.pose.position = pose.position
        m.color.r = 0.1
        m.color.g = 0.8
        m.color.b = 0.2
        m.color.a = 0.45
        up = self._world_up_in_frame(header.frame_id)
        if up is None:
            m.type = Marker.SPHERE
            m.pose.orientation.w = 1.0
            m.scale.x = 0.07
            m.scale.y = 0.07
            m.scale.z = 0.07
        else:
            m.type = Marker.CYLINDER
            rot = _frame_from_z(up, prev_rot=None, toward_camera=False)
            qx, qy, qz, qw = rot_to_quat(rot)
            m.pose.orientation.x = qx
            m.pose.orientation.y = qy
            m.pose.orientation.z = qz
            m.pose.orientation.w = qw
            m.scale.x = 0.07
            m.scale.y = 0.07
            m.scale.z = 0.10
        self.pub_marker.publish(m)

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CupPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
