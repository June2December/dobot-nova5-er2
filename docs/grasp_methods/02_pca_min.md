# pca_min — 점구름 단축

## 실행

```bash
ros2 launch dobot_perception cup_pose.launch.py orient_mode:=pca_min
```

## 원리

`pca_max`와 같은 SVD인데 **가장 얇은 방향**을 Z로 쓴다.

위에서 내려다본 원통 컵: 가로·세로로 테두리가 넓고, **높이 방향 두께가 가장 얇다.** 그때 \(v_3\)(최소 분산)이 서 있는 축에 가깝다.

\[
z = \pm v_3 \quad (V\text{의 마지막 열, } \texttt{vt[-1]})
\]

옆에서 보면 최소 분산이 테이블 법선(얇은 깊이)이 되어, max와 **역할이 반대**가 된다.

## 수식

동일하게 \(\tilde{X}=U\Sigma V^\top\), 특이값 \(\sigma_1\ge\sigma_2\ge\sigma_3\), \(z \parallel V_{\cdot 3}\).

## 볼 것

카메라를 컵 **위**에 두고 max / min을 번갈아 켠다. 위에서 min이 더 서 있는 축처럼 보이면 이 모드가 그 시점에 맞다.
