# pca_max — 점구름 장축 (기존 기본값)

## 실행

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/local_setup.bash
ros2 launch dobot_perception cup_pose.launch.py orient_mode:=pca_max
```

인자: 없음. 생략해도 이 모드. `orient_mode:=pca` 동일.

## 원리

박스 안 3D 점 \(X\in\mathbb{R}^{N\times 3}\)의 **공분산에서 분산이 가장 큰 방향**을 물체 Z로 둔다. 옆에서 본 길쭉한 컵이면 높이와 비슷하다.

점들을 평균으로 옮긴다. (RViz 원점이 아니다. 컵 모양만 본다.)

\[
\tilde{X}_i = X_i - \bar{X}
\]

SVD:

\[
\tilde{X} = U\Sigma V^\top
\]

\(V\)의 첫 열(코드에선 `vt[0]`)이 최대 분산 방향 \(v_1\).

\[
z = \pm v_1
\]

부호는 이전 프레임과 내적이 양수가 되게, 첫 프레임은 카메라 쪽(\(z_z<0\))을 선호.

나머지 축: \(x\)는 어제 \(x\)를 새 \(z\) 평면에 투영, \(y=z\times x\).

## 수식 요약

공분산 \(C=\frac{1}{N}\tilde{X}^\top\tilde{X}\)의 최대 고유벡터 = \(v_1\). SVD의 \(V\)와 같다.

## 볼 것

- 옆에서 본 컵: 파란 축이 컵 높이 방향이면 성공에 가깝다.
- 위에서 본 컵: 파란 축이 테이블에 누워 있으면 이 모드의 실패 모드다.
- 손잡이 머그: 장축이 손잡이 쪽으로 갈 수 있다.

## 한계

거리와 무관. 집기 접근 방향으로 쓰면 안 된다.
