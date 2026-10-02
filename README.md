# Dobot Nova 5 · Gemini Robotics ER 2 (ROS 2, 실기)

Isaac Sim / Franka 레포([GR-ER2](https://github.com/engiengi/physical-ai-system-design/tree/main/GR-ER2))와 **같은 역할 분리**를 Dobot에 맞춘 워크스페이스다.

```text
D405 RGB  →  Gemini Robotics ER 2  →  pick/place 픽셀
D405 depth →  카메라 3D
TF base_link →  Nova 5 MovL + Tool DO
```

모델은 관절을 안 돌린다. `pick_place`만 고른다. 움직임은 `dobot_bringup_v3`다.

| 이 레포 | GR-ER2 |
|---|---|
| `dobot_er2` | `src/app.py` 도구 루프 |
| `dobot_perception` | Isaac 카메라 대신 D405 |
| `dobot_bringup_v3` (벤더) | Franka `PickPlaceController` |

## 작업 노트북 설치

Ubuntu 22.04, ROS 2 Humble.

```bash
sudo apt update
sudo apt install ros-humble-moveit ros-humble-std-srvs
pip3 install -r requirements-er2.txt   # google-genai
```

벤더 스택은 공식 레포에서 받는다. 이 git에는 넣지 않았다.

```bash
mkdir -p ~/dobot_ws/src
cd ~/dobot_ws/src
# 이 레포를 클론했다면 src 안에 dobot_er2 / dobot_perception 이 이미 있다.
git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3.git
```

YOLO 가중치 `src/dobot_perception/models/yolo26x.pt` 는 git에 없다. 기존 PC에서 복사하거나 Ultralytics가 첫 실행 때 받게 한다.

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dobot_msgs_v3 dobot_bringup_v3 dobot_moveit nova5_moveit \
  dobot_rviz cra_description dobot_perception dobot_er2
source install/local_setup.bash
```

API 키:

```bash
cp .env.example ~/.config/dobot_er2.env
# GEMINI_API_KEY=... 를 채운다
export GEMINI_API_KEY=...
```

Preview 모델 `gemini-robotics-er-2-preview` 권한이 있어야 한다.  
[공식 ER 2](https://ai.google.dev/gemini-api/docs/robotics-overview)

## 테스트 순서 (한 칸씩)

실기는 **기본이 안 움직인다** (`execute_robot:=false`).

### A. 카메라 + ER 2만 (팔 꺼둠)

D405 USB, GPU 권장.

```bash
export GEMINI_API_KEY=...
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/local_setup.bash
ros2 launch dobot_er2 er2_camera.launch.py
```

다른 터미널:

```bash
ros2 service call /er2/plan std_srvs/srv/Trigger
ros2 topic echo /er2/status --once
```

`pick_xyz_cam_m` 이 나오면 모델+깊이는 된 것. 오버레이는 `/er2/overlay`.

### B. 팔만 (ER 2 없이)

[`docs/NOVA5_CHECKLIST.md`](docs/NOVA5_CHECKLIST.md) 1→7. ping, TCP 원격, Enable, MoveIt Execute.

```bash
export IP_address=192.168.5.1   # ping 된 IP
export DOBOT_TYPE=nova5
```

### C. 그리퍼만

bringup + Enable 이후:

```bash
ros2 service call /dobot_bringup_v3/srv/ToolDOExecute dobot_msgs_v3/srv/ToolDOExecute "{index: 1, status: 1}"
ros2 service call /dobot_bringup_v3/srv/ToolDOExecute dobot_msgs_v3/srv/ToolDOExecute "{index: 1, status: 0}"
```

안 닫히면 `index: 2`. 그래도 안 되면 캐비닛 DO가 아니라 그리퍼가 Modbus인지 실물을 본다.

### D. 한 번에 집기 (실기)

카메라 TF는 아직 눈대중이다. 첫 Execute는 속도 10, 사람 손 대기.

터미널 1 — 컨트롤러:

```bash
export IP_address=192.168.5.1
export DOBOT_TYPE=nova5
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/local_setup.bash
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

터미널 2 — 활성화:

```bash
ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError
ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot "{load: 0.0}"
```

터미널 3 — 카메라+모델+스킬 (처음엔 dry-run):

```bash
export GEMINI_API_KEY=...
ros2 launch dobot_er2 er2_nova5.launch.py execute_robot:=false
```

터미널 4:

```bash
ros2 service call /er2/plan std_srvs/srv/Trigger
ros2 service call /er2/execute std_srvs/srv/Trigger
```

그리퍼는 기본이 **수동**. 팔이 pick에 멈추면 컨트롤러에서 닫고:

```bash
ros2 service call /er2/continue std_srvs/srv/Trigger
```

place에 멈추면 열고 다시 continue. ROS ToolDO를 쓰려면 `gripper_mode:=tool_do`.

`/er2/skill_status` 에 `pick_base_mm` 이 나오면 TF까지 된 것. 숫자가 테이블 위인지 확인한 뒤에만:

```bash
ros2 launch dobot_er2 er2_nova5.launch.py execute_robot:=true speed_ratio:=10 gripper_mode:=manual
```

다시 plan → execute. 이때 실기가 움직인다.

카메라 장착이 눈대중이면 컵 옆 허공을 찌를 수 있다. 그때는 `cam_x` 등 런치 인자를 고친다.

## 패키지

| 패키지 | 역할 |
|---|---|
| `dobot_er2` | ER 2 호출, depth 역투영, MovL/ToolDO 스킬 |
| `dobot_perception` | D405 + YOLO (위치 검증용, ER 2는 RGB를 직접 봄) |
| `DOBOT_6Axis_ROS2_V3` | 공식 실기 드라이버 |

`docs/` 는 1차 카메라 실험 기록과 Nova 5 오전 체크리스트다.
