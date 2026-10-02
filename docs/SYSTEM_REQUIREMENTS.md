# 시스템 요구사항

**프로젝트:** Dobot Nova 5 테이블탑 데모 (`dobot_ws`)  
**문서 ID:** SRS-DOBOT-001  
**버전:** 0.2  
**날짜:** 2026-09-08  
**상태:** 1차(카메라) 통과 · 2차(실기+그리퍼) 진행 전

무엇을 시연할지, 돌리려면 무엇이 필요한지. VLA / Isaac은 범위 밖.

---

## 1. 목적

최종 시연은 **Nova 5 실기**가 테이블 위 컵을 보고, 팔이 움직이고, J6 그리퍼가 열고 닫히는 것이다. 카메라는 쓸 것이고, CR16 fake + D405는 그 카메라 파이프라인의 1차 검증이었다.

| 단계 | 목표 | 상태 |
|---|---|---|
| 1 | D405로 컵 검출·거리·RViz 표시. 자세 축은 실험. | **통과** (실기 없음, CR16 모델은 시각화용) |
| 2 | PC ↔ Nova 5 컨트롤러 ROS 2 연결. RViz에 실기 관절이 나오고, Plan/Execute로 팔이 움직인다. J6 그리퍼 열고 닫기. RViz에 그리퍼 메시가 없어도 됨. | **다음** |
| 3 | 컵 위치를 실기 목표로 바꿔 접근·집기. 카메라 캘리브, 위아래 접근. | 아직 아님 |

1차에서 확인한 것: YOLO 박스 + depth median 위치는 쓸 만하다. PCA/`orient_mode`로 컵이 어떻게 서 있는지를 안정적으로 재구성하진 못했다. 집기 Z는 나중에 테이블 법선(위→아래)이지 `pca_max`가 아니다.

2차는 인식과 붙이지 않는다. 컨트롤러에 직접 꽂지 않고, **로봇 LAN → `dobot_bringup_v3` → ROS 2 서비스**로 팔과 그리퍼를 본다.

---

## 2. 범위

### 1차에서 넣은 것 (유지)

- D405 RGB-D 캡처 (color에 맞춘 depth)
- YOLO26x로 `cup`, `bottle`, `wine glass`, `bowl` 검출
- depth 중심점 + 선택적 자세 → `camera_color_optical_frame`의 `PoseStamped`
- 디버그 이미지, TF `cup`, RViz 마커
- CR16 MoveIt fake + 임시 카메라 TF (시각화만)

### 2차에서 넣을 것

- `dobot_bringup_v3`로 Nova 5 실기 연결
- `dobot_moveit` + `nova5_moveit`으로 RViz 동기 모션
- J6 그리퍼 Tool DO 또는 Modbus

### 지금 제외 (2차 이후)

- 컵 포즈 → MoveIt 집기 실행
- 캘리브된 카메라–베이스 외부파라미터 (장착 TF는 눈대중)
- Isaac Sim / VLA 정책
- 다중 물체 추적 ID
- 벤더 기본값 이상의 안전 PLC / 협동 속도 제한

---

## 3. 시스템 맥락

```
[D405 USB]
    │  librealsense
    ▼
cup_pose_node (YOLO26x + depth PCA)
    │  /perception/cup_pose
    │  /perception/debug_image
    │  TF: camera_color_optical_frame → cup
    ▼
RViz2  ←  cr16_moveit fake demo  ←  static TF base_link → camera
```

런치 진입점 두 개:

| 런치 | 뜨는 것 |
|---|---|
| `dobot_perception cup_pose.launch.py` | 카메라 노드 + 선택적 RViz (로봇 없음) |
| `dobot_perception moveit_cup_pose.launch.py` | CR16 MoveIt fake + 카메라 TF + 카메라 노드 + RViz |

---

## 4. 하드웨어 요구사항

### 4.1 연산 (최소)

| 항목 | 요구 |
|---|---|
| OS | Ubuntu 22.04 LTS |
| CPU | x86_64, 8코어 이상 권장 |
| RAM | 최소 16 GB, 권장 32 GB |
| GPU | CUDA 가능한 NVIDIA GPU (데모는 고사양 50번대 카드 기준). `device:=cpu`는 가능하나 전시 프레임레이트용은 아님 |
| 디스크 | 여유 20 GB (Humble + 워크스페이스 + `yolo26x.pt` 약 114 MB) |
| 디스플레이 | RViz / OpenCV 디버그 창용 |

### 4.2 로봇

| 항목 | 요구 |
|---|---|
| 팔 | **실기 Dobot Nova 5**. 1차 RViz 시각화만 CR16 fake (`cr16_moveit`) |
| 제어 | 벤더 `DOBOT_6Axis_ROS2_V3`. PC는 컨트롤러에 TCP. `IP_address`, `DOBOT_TYPE=nova5` |
| 시뮬 | 1차: CR16 MoveIt fake. 2차 실기는 fake 없이 bringup + `dobot_moveit` |
| 실기 | 컨트롤러 LAN 도달. 유선 기본 `192.168.5.1`, 무선 `192.168.1.6`. 원격 TCP 모드 + EnableRobot |
| 엔드이펙터 | J6 그리퍼. 열림/닫힘 모델이면 Tool DO, 위치·악력이면 컨트롤러 Modbus. Nova5 URDF에는 그리퍼 관절 없음 (RViz 손가락 생략 가능) |

### 4.3 카메라

| 항목 | 요구 |
|---|---|
| 센서 | Intel RealSense **D405** |
| 인터페이스 | USB 3.x, 단독 점유 (`realsense-viewer` / 다른 파이프라인과 공유 금지) |
| 스트림 | Color + depth, 848×480 @ 30 FPS, depth를 color에 align |
| 장착 | `base_link` 기준 고정. 임시 TF 기본값: `x=0.50 m`, `y=0.0`, `z=0.75 m`, `roll=-2.50`, `yaw=-1.5708`. 실제 캘리브는 미정 |
| 장면 | 팔 앞 테이블. 물체는 카메라에서 0.05–2.0 m, 조명 충분할 것 |

### 4.4 대상 물체

YOLO26이 보는 COCO 식기 클래스: cup, bottle, wine glass, bowl.  
테이블 위에 세워 둔 컵이 주 데모 물체다.

---

## 5. 소프트웨어 요구사항

### 5.1 ROS / 미들웨어

| 항목 | 요구 |
|---|---|
| 배포판 | ROS 2 **Humble** |
| 워크스페이스 | `/opt/ros/humble` 위 `~/dobot_ws` overlay |
| 빌드 | `colcon`, `ament_python` + `ament_cmake` 패키지 |
| 런치 | `ros2launch` 파이썬 런치 파일 |
| 시각화 | `rviz2`, MoveIt MotionPlanning 플러그인 |
| 모션 플래닝 | MoveIt 2 (`cr16_moveit`) |
| TF | `tf2_ros` static publisher + 노드 broadcaster |
| DDS | 단일 PC는 Humble 기본 DDS. 이후 Isaac Sim / 멀티호스트면 CycloneDDS |

### 5.2 인식 스택

| 항목 | 요구 |
|---|---|
| 카메라 SDK | Intel `librealsense2` (`pyrealsense2`) |
| 검출기 | Ultralytics **YOLO26x** (`yolo26x.pt`), COCO 80 클래스 |
| Ultralytics | `>= 8.4` (YOLO26 가중치) |
| OpenCV | 디버그 오버레이용 `cv2` |
| NumPy | 점구름 중심 / PCA |
| CUDA | `device:=cuda:0`일 때 로컬 드라이버에 맞는 PyTorch CUDA 빌드 |

가중치는 `src/dobot_perception/models/yolo26x.pt`에 둔다 (share에 설치하지 않음). 없으면 Ultralytics GitHub 에셋에서 최초 1회 다운로드.

### 5.3 이 데모에서 쓰는 워크스페이스 패키지

| 패키지 | 역할 |
|---|---|
| `dobot_perception` | D405 + YOLO + 6D 포즈 노드와 런치 (1차) |
| `cr16_moveit` | 1차 RViz 스탠드인 |
| `nova5_moveit` | 2차 실기 MoveIt 모델 |
| `cra_description` | 메시. Nova5 URDF에 그리퍼 관절 없음 |
| `dobot_demo` | 벤더 모션 헬퍼 |
| `dobot_bringup_v3` | 실기 컨트롤러 TCP bringup (2차) |
| `dobot_moveit` | 실기 joint_states ↔ MoveIt |

---

## 6. 기능 요구사항

표시가 없으면 **해야 한다(SHALL)**. SHOULD / MAY만 별도 표기.

| ID | 요구사항 |
|---|---|
| FR-01 | 시스템은 D405를 열고 align된 RGB-D를 스트리밍해야 한다. 장치가 busy면 재시도한 뒤, 실패 시 원인을 로그에 남겨야 한다. |
| FR-02 | 시스템은 컬러 이미지에 YOLO26x를 돌려야 한다. 노드 틱은 약 15 Hz (카메라는 30 FPS일 수 있음). |
| FR-03 | 클래스 목록을 설정할 수 있어야 한다. 기본값 `cup,bottle,wine glass,bowl`. |
| FR-04 | 해당 클래스 중 **신뢰도가 가장 높은** 검출 하나만 써야 한다. |
| FR-05 | 3D 위치는 축소한 바운딩 박스 안, 유효 depth 픽셀(0.05–2.0 m)의 **median**이어야 한다. |
| FR-06 | 자세는 그 점구름의 PCA로 추정해야 하며, 시간축 부호 잠금으로 Z축이 180°로 깜빡이지 않아야 한다. |
| FR-07 | 포즈는 `/perception/cup_pose`에 `geometry_msgs/PoseStamped`로, 프레임은 `camera_color_optical_frame` (X 오른쪽, Y 아래, Z 앞)이어야 한다. |
| FR-08 | TF `camera_color_optical_frame` → `cup`을 브로드캐스트해야 한다. |
| FR-09 | 박스·거리·축이 그려진 디버그 이미지를 `/perception/debug_image`에 퍼블리시해야 한다. |
| FR-10 | MoveIt 런치는 static TF `base_link` → `camera_color_optical_frame`을 퍼블리시해서 컵이 로봇 장면에 보이게 해야 한다. 장착값은 런치 인자여야 한다. |
| FR-11 | MoveIt 런치는 CR16 fake MoveIt을 RViz 없이 띄운 뒤, 로봇+이미지+마커가 있는 RViz를 **하나만** 띄워야 한다. |
| FR-12 | 검출 전용 런치는 로봇을 생략해도 되고, OpenCV 창을 띄워도 된다 (MAY). |
| FR-13 | GPU 디바이스, 신뢰도, 클래스 목록, 모델 경로는 런치/노드 파라미터여야 한다. |

### 2차 (실기 + 그리퍼). 컵 인식과 아직 연결하지 않음.

| ID | 요구사항 |
|---|---|
| FR-30 | PC가 Nova 5 컨트롤러에 ping 되고 `dobot_bringup_v3`가 TCP로 붙어야 한다. |
| FR-31 | `EnableRobot` 이후 `/joint_states`가 실기 각을 반영하고, RViz Nova 5 모델이 따라와야 한다. |
| FR-32 | MoveIt Plan and Execute로 작은 관절/카르테시안 이동이 **실기에서** 보여야 한다. |
| FR-33 | J6 그리퍼를 ROS 2 서비스로 열고 닫을 수 있어야 한다. 방식은 실물 그리퍼에 따라 Tool DO(A) 또는 컨트롤러 Modbus(B). |
| FR-34 | RViz에 그리퍼 손가락이 안 열려도 된다. 실물 동작이면 2차 통과. |

### 3차 이후. 자세 실험 기록은 `docs/grasp_methods/`.

| ID | 의도 |
|---|---|
| FR-20 | 캘리브된 카메라 장착으로 컵 포즈를 `base_link`로 변환 |
| FR-21 | MoveIt으로 접근 / 파지. 접근 Z는 PCA 장축이 아니라 테이블 법선(위→아래). |
| FR-22 | 인식 위치와 2차에서 검증한 그리퍼를 한 시퀀스로 연결 |
| FR-23 | 파지 높이는 컵 바닥이 아니라 몸통/립. 그리퍼가 테이블까지 내려가 닫히면 안 된다. |

---

## 7. 비기능 요구사항

| ID | 요구사항 |
|---|---|
| NFR-01 지연 | CUDA일 때 검출+포즈는 RViz에서 상호작용 가능해야 한다 (SHOULD, 대략 10 Hz 이상). |
| NFR-02 안정 | 물체가 계속 보이면 컵 TF Z축이 몇 프레임마다 뒤집히면 안 된다. |
| NFR-03 강건 | 미검출 시 크래시하지 않고 status 문자열을 내야 한다. depth가 없으면 포즈를 내지 않아야 한다. |
| NFR-04 단독 점유 | D405는 프로세스 하나만 점유해야 한다 (MAY → 사실상 필수 제약). |
| NFR-05 재현 | `colcon build`와 `source install/local_setup.bash` 이후 추가 `PYTHONPATH` 없이 `ros2 launch dobot_perception ...`가 런치, RViz 설정, 노드를 찾아야 한다. |
| NFR-06 안전 | 2차 실기는 속도 스케일 낮게, 작업공간 비우고, Enable 전에 비상정지 위치를 확인한다. 1차 기본은 계속 fake MoveIt. |

---

## 8. 인터페이스

### 8.1 토픽 (`cup_pose_node`)

| 토픽 | 타입 | 방향 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 출력 |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | 출력 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 출력 |
| `/perception/debug_image` | `sensor_msgs/Image` | 출력 |
| `/perception/cup_pose` | `geometry_msgs/PoseStamped` | 출력 |
| `/perception/cup_confidence` | `std_msgs/Float32` | 출력 |
| `/perception/cup_label` | `std_msgs/String` | 출력 |
| `/perception/cup_distance` | `std_msgs/Float32` | 출력 |
| `/perception/cup_offset_camera` | `geometry_msgs/Vector3Stamped` | 출력 |
| `/perception/cup_status` | `std_msgs/String` | 출력 |
| `/perception/cup_marker` | `visualization_msgs/Marker` | 출력 |

### 8.2 TF

| 부모 | 자식 | 출처 |
|---|---|---|
| `base_link` | `camera_color_optical_frame` | `static_transform_publisher` (MoveIt 런치만) |
| `camera_color_optical_frame` | `cup` | 포즈가 유효할 때 `cup_pose_node` |

### 8.3 런치 인자 (MoveIt + 인식)

`device`, `show_window`, `conf`, `model`, `classes`,  
`cam_x`, `cam_y`, `cam_z`, `cam_roll`, `cam_pitch`, `cam_yaw`

---

## 9. 가정과 제약

1. 이 개정에서 지원하는 호스트는 Ubuntu 22.04 + Humble뿐이다.
2. 카메라 optical 프레임은 ROS 표준 (Z가 앞).
3. 카메라 장착 TF는 핸드아이 캘리브 전까지 임시값이다. RViz에서 월드 기준 컵 위치는 근사다.
4. YOLO는 COCO 사전학습이며, 이 부스 데이터로 파인튜닝하지 않았다.
5. Ultralytics `runs/` 산출물은 워크스페이스 루트에 쓰면 안 된다. 추론 저장 경로는 `/tmp/dobot_perception_yolo`.
6. 큰 가중치는 `setup.py`의 `data_files`로 설치하지 않는다.

---

## 10. 인수 확인 (이 개정)

데모 PC에 D405가 연결된 상태에서 아래를 모두 만족하면 통과다.

1. `ros2 launch dobot_perception cup_pose.launch.py`가 카메라를 열고, 디버그 이미지에 컵 박스를 그린다.
2. 컵이 보이면 `/perception/cup_pose`에 유한한 포즈가 나온다.
3. TF `cup`이 있고, 계속 180°로 뒤집히지 않는다.
4. `ros2 launch dobot_perception moveit_cup_pose.launch.py`가 하나의 RViz에 CR16 + 카메라 프레임 + 컵 마커를 보여 준다.
5. `realsense-viewer`를 끈 뒤 다시 런치하면 카메라를 잡거나, busy라고 로그를 남긴다.

---

## 11. 실행 명령

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/local_setup.bash

# 인식만
ros2 launch dobot_perception cup_pose.launch.py

# CR16 fake MoveIt + 인식
ros2 launch dobot_perception moveit_cup_pose.launch.py
```

CPU 대체: `device:=cpu`  
RViz 끄기: `rviz:=false` (검출 전용 런치)

---

## 12. 개정 이력

| 버전 | 날짜 | 내용 |
|---|---|---|
| 0.2 | 2026-09-08 | 실기 목표를 Nova 5로 확정. 1차 카메라 통과, 2차 실기+그리퍼. |
| 0.1 | 2026-09-07 | 인식 + CR16 시각화 마일스톤 초안 |

---

## 13. 2차 실행 (Nova 5)

컨트롤러와 PC는 같은 망. 벤더 README 기준 유선 `192.168.5.1`, 무선 `192.168.1.6`.

```bash
export IP_address=192.168.5.1    # 실제 컨트롤러 IP
export DOBOT_TYPE=nova5

source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/local_setup.bash

# 1) 실기 TCP 드라이버 (컨트롤러에 연결)
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py

# 2) 원격 TCP 모드 확인 후 상시 정지 위치 확인, 그다음 활성화
ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot

# 3) RViz + MoveIt. 실기 /joint_states가 모델에 들어감
ros2 launch dobot_moveit dobot_moveit.launch.py
```

그리퍼는 실물을 보고 나눈다. 이 스택은 PC `/dev/ttyUSB0`로 J6에 직접 꽂지 않는다. 컨트롤러 TCP 서비스가 J6 핀 또는 툴 버스에 신호를 낸다.

**방식 A — Tool DO (열기/닫기)**

```bash
# 즉시 출력. index는 보통 툴 DO 1 또는 2, status 0/1
ros2 service call /dobot_bringup_v3/srv/ToolDOExecute dobot_msgs_v3/srv/ToolDOExecute "{index: 1, status: 1}"
ros2 service call /dobot_bringup_v3/srv/ToolDOExecute dobot_msgs_v3/srv/ToolDOExecute "{index: 1, status: 0}"
```

`ToolDO`는 큐에 넣고, `ToolDOExecute`는 바로 낸다. 캐비닛 DO는 `/dobot_bringup_v3/srv/DO` (J6 아님).

**방식 B — 컨트롤러 Modbus (위치/악력)**

```bash
ros2 service call /dobot_bringup_v3/srv/ModbusCreate dobot_msgs_v3/srv/ModbusCreate
ros2 service call /dobot_bringup_v3/srv/SetHoldRegs dobot_msgs_v3/srv/SetHoldRegs
```

레지스터 맵은 그리퍼 매뉴얼. 슬레이브 ID·주소는 추측하지 않는다.

2차 통과: 팔이 RViz와 실기에서 같이 움직이고, 그리퍼가 실물에서 열리고 닫힌다.
