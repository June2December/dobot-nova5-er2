# identity — 자세를 안 추정

## 실행

```bash
ros2 launch dobot_perception cup_pose.launch.py orient_mode:=identity
```

별칭: `none`.

## 원리

\[
R = I_3
\]

컵 TF의 축이 **카메라 optical과 나란하다.** 위치 \(p\)만 의미가 있다. RViz 축은 물체를 설명하지 않고, “여기에 점이 있다”만 본다.

거리가 목표면 이 모드로 축 깜빡임을 제거한 채 위치만 확인하면 된다.

## 볼 것

파란 축이 카메라 +Z와 같게 고정. 컵을 돌려도 축은 안 돈다. 마커 원통도 카메라에 정렬되어 컵과 안 맞을 수 있다. 정상이다.
