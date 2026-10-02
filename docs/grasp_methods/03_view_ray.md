# view — 카메라에서 컵으로 가는 시선

## 실행

```bash
ros2 launch dobot_perception cup_pose.launch.py orient_mode:=view
```

별칭: `view_ray`.

## 원리

물체 기하를 안 본다. Z를 **카메라 원점 → 컵 중심**으로 둔다.

\[
z = \frac{p}{\|p\|},\quad p=\mathrm{median}(X)
\]

optical에서 \(p_z>0\)이면 \(z\)는 대체로 **렌즈 앞(+Z)** 이다. 핸드아이로 위에서 내려오면 이 방향이 “카메라가 보는 쪽 = 접근 방향”에 가깝다.

점구름 모양·손잡이·누운 컵과 무관해서 **축이 잘 안 뒤집힌다.** 대신 “컵이 어떻게 서 있는지”는 전혀 모른다.

## 수식

핀홀 \(p\)는 위치와 같고, 회전만

\[
z=\hat{p},\quad
x = \mathrm{normalize}(e \times z),\quad
y = z \times x
\]

\(e=(1,0,0)\) 또는 \(z\)와 거의 평행이면 \(e=(0,1,0)\).

이전 프레임 잠금은 pca와 같다. `toward_camera=False`라 첫 프레임에서 \(z_z>0\)을 뒤집지 않는다.

## 볼 것

파란 축이 항상 카메라에서 컵으로 꽂히면 정상. 컵을 기울여도 축은 시선만 따라간다.
