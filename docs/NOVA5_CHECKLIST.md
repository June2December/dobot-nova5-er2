# Nova 5 오전 체크 (실기 팔)

한 칸 실패하면 다음으로 가지 않는다. 카메라·그리퍼는 이 목록에 없다.

Gazebo는 안 켠다. RViz가 실기 자세를 그리는 화면이다.

---

## 0. 노트북 (출발 전 또는 현장 처음)

- Ubuntu 22.04, ROS 2 Humble
- `sudo apt install ros-humble-moveit`
- USB에서 `build` / `install` / `log` 빼고 워크스페이스 복사
- 인식 패키지는 카메라·CUDA 없으면 빌드에서 뺀다

```bash
cd ~/dobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dobot_msgs_v3 dobot_bringup_v3 dobot_moveit nova5_moveit \
  dobot_rviz cra_description
source install/local_setup.bash
```

빌드 에러 나면 여기서 멈춘다.

---

## 1. 망

컨트롤러 화면에서 IP를 적는다.

| 연결 | 컨트롤러 IP (벤더 기본) | 노트북 예 |
|---|---|---|
| 유선 | `192.168.5.1` | `192.168.5.100` 같은 같은 대역 고정 IP |
| 무선 | `192.168.1.6` | `192.168.1.x` |

```bash
ip -4 addr
ping -c 3 192.168.5.1
```

ping 실패 → 케이블/와이파이/노트북 IP. ROS 아직 안 켠다.

---

## 2. 컨트롤러 (화면에서 손으로)

1. 비상정지 위치 확인. 작업공간 비움.
2. 로봇이 앱/티칭 전용이 아니라 **TCP/IP 원격(2차 개발)** 인지 확인.
3. 에러 떠 있으면 클리어. 나중에 ROS `ClearError`로도 가능.
4. 팔이 테이블·벽에 박히지 않은 자세인지 눈으로 확인.

---

## 3. 환경 변수 (노트북 터미널)

IP는 **1에서 ping 된 그 주소**.

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/local_setup.bash
export IP_address=192.168.5.1
export DOBOT_TYPE=nova5
echo "$IP_address $DOBOT_TYPE"
```

`DOBOT_TYPE`이 `nova5`가 아니면 RViz 모델이 안 맞는다.

---

## 4. 드라이버

**터미널 A**

```bash
ros2 launch dobot_bringup_v3 dobot_bringup_ros2.launch.py
```

로그에 `connection succeeded:29999,30003` 이 나와야 한다.  
`Connection failed!!!` 또는 IP가 `None`이면 `$IP_address` 다시.

**터미널 B** (같은 source + export)

```bash
ros2 service list | grep EnableRobot
ros2 topic echo /joint_states_robot --once
```

서비스가 있고 관절 숫자 6개가 나오면 연결됨. 안 나오면 4에서 멈춤.

---

## 5. 에러 있으면 클리어 후 활성화

아직 팔은 안 움직인다. 모터만 켠다.

```bash
ros2 service call /dobot_bringup_v3/srv/ClearError dobot_msgs_v3/srv/ClearError
ros2 service call /dobot_bringup_v3/srv/EnableRobot dobot_msgs_v3/srv/EnableRobot "{load: 0.0}"
```

컨트롤러가 활성화(Enable)로 바뀌는지 본다. 거부하면 2번 TCP 모드·에러·비상정지.

---

## 6. RViz (실기 자세가 보여야 함)

**터미널 C**

```bash
ros2 launch dobot_moveit dobot_moveit.launch.py
```

확인:

- 모델이 Nova 5인지
- Motion Planning → Planning Group: `nova5_group`
- 화면 속 팔 자세 = 앞의 실기 자세

안 따라가면 `/joint_states`를 한 번 본다.

```bash
ros2 topic echo /joint_states --once
```

---

## 7. 작게 움직이기 (이때 실기 + RViz)

속도·가속도 스케일 **0.1** 근처.

1. RViz에서 마커를 **조금만** 옮긴다. 큰 스윙 금지.
2. **Plan** → 경로가 테이블을 안 뚫는지 본다.
3. **Execute** → 실기가 움직이고 RViz가 따라가면 팔 쪽 통과.

안 움직이면 Execute 로그, 컨트롤러 에러, Enable 상태를 본다. 다시 Plan만 하고 Execute를 안 눌렀는지도 본다.

---

## 8. 끄기

Execute 중이 아닐 때.

```bash
ros2 service call /dobot_bringup_v3/srv/DisableRobot dobot_msgs_v3/srv/DisableRobot
```

런치 터미널은 Ctrl+C. 비상시에는 컨트롤러 비상정지 먼저.

---

## 아직 안 함 (7번 다음에)

그리퍼: `ToolDOExecute` index 1 또는 2.  
ER 2 집기: 저장소 루트 `README.md` 절 D. `execute_robot:=false` 로 좌표부터 확인.
