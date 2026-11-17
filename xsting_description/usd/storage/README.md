<div align="center">

<p><strong>XSTING MK2 · SIMULATION ASSETS</strong></p>
<h1>수납 충돌 모델</h1>
<p>12칸 바스킷의 정밀 접촉 형상, 퍽 낙하 검사와 센서 구성</p>
<p><a href="../../../README.md">프로젝트 홈</a> &nbsp; · &nbsp; <a href="../../README.md">로봇 모델</a></p>

</div>

---

**바로 가기** &nbsp; [퍽과 검사 장면](#section-1) · [재생성](#section-2) · [카메라 및 초음파 방식 거리 센서](#section-3)

`../xsting_storage_training.usda`는 이동형 학습 로봇, `../xsting_storage_manipulation.usda`는
고정 베이스 팔 조작 로봇입니다. 기존 파일은 그대로 두고 몸체 충돌만 교체합니다.
두 프리셋은 원래 USD와 `storage_colliders.usdc`가 함께 있어야 합니다.

- 원본 `base_link.stl`에서 닫힌 바스킷 메시(74,970 삼각형)를 추출했습니다.
- 몸체 전체 convex decomposition을 끄고, 바스킷 / 층판 4개 / 앞쪽 팔 마운트 / 전면 RealSense를 분리했습니다.
- 바스킷은 원본 삼각형을 유지하는 PhysX SDF를 사용합니다. 해상도 1024,
  최대 축 기준 명목 격자 간격 약 0.213 mm입니다. 이는 절대 접촉 오차 보증값은 아닙니다.
- 바스킷과 퍽의 contact offset은 각각 0.2 mm, rest offset은 0입니다.
- 아래부터 실제 층판 4장을 각각 외곽 크기 약 265.55×265.55×9 mm의 박스 충돌로 처리했습니다.
  바닥 z는 base 기준 약 0.022 / 36.722 / 99.222 / 151.722 mm입니다.
  `Floor1`은 기존 URDF 설명의 하단 추가 와플판에 해당합니다. 구멍은 막히며,
  층판 사이 배선·전자부품은 이 간략 모델의 별도 충돌 형상에 포함하지 않습니다.
- 앞으로 돌출된 RealSense는 `StorageColliders/FrontRealSense` 별도 convex hull입니다.
  원본 CAD 본체 외곽을 추출했으며 크기는 X×Y×Z 약 25.05×89.714×25.00 mm,
  base 기준 X 범위 136.943~161.993 mm입니다. PhysX hull vertex limit은 256입니다.
  렌즈 구멍·내부 전자부품은 막힌 외곽으로 근사합니다. 기존 질량·관성은 유지합니다.
- 팔 마운트는 convex decomposition이며, 팔·다리는 기존 모델을 사용합니다.
- SDF 접촉용으로 GPU PhysX를 사용하세요. 테스트 장면은 GPU dynamics, TGS, 240 Hz로 설정했습니다.
  대규모 병렬 환경의 처리량과 VRAM 사용량은 별도 측정이 필요합니다.

<a id="section-1"></a>

## 퍽과 검사 장면

`inspect_square.usda`: 30×30×60 mm 사각 퍽 12개.
`inspect_round.usda`: 지름 30×높이 60 mm 원형 퍽 12개.
각 파일을 열고 Play하면 계산된 지지 높이보다 5 mm 위에서 퍽을 놓습니다.
원형 충돌 메시는 32각 기둥으로, 원 대비 최대 반경 오차는 약 0.072 mm입니다.
퍽 질량은 밀도 1000 kg/m³를 가정한 시험값(사각 54 g, 원형 약 42.4 g)입니다.
실제 학습에서는 실측 질량·마찰·치수 공차로 조정해야 합니다.

수납부의 중심축 단면 폭은 약 32 mm여서 30 mm 퍽은 한쪽당 약 1 mm의 여유를 가집니다.
`pallet_inner_1`, `pallet_inner_4`의 바닥 돌출부는 기존 슬롯 프레임보다 약 1 mm 높습니다.
슬롯 프레임을 바닥 높이로 그대로 쓰지 않고, 원본 메시와 퍽의 첫 접촉 높이를 계산했습니다.

`geometry_validation.json`은 원본 바스킷 삼각형과 FCL box/cylinder의 24개 검사 결과입니다.
수직 삽입 경로는 0.5 mm 간격으로 검사하고, 지지 높이는 이분 탐색했습니다.
이는 기하학 검사이며 실제 PhysX 검사 결과는 `runtime_validation.json`에 별도로 기록합니다.
GPU PhysX에서 각 형태 12개, 5 mm 높이 낙하, 240 Hz × 720 step(3초)을 실행했습니다.
24개 모두 수납부에 유지되었고, 최종 자세를 원본 삼각형과 대조한 최대 접촉 침투량은 0.051 mm였습니다.
앞쪽 두 칸은 돌출부 때문에 약 3.3~3.8° 기울어져 안착했습니다. 이 수치는 해당 시험 결과로,
모든 자세·속도에서 동일한 오차를 보증하지 않습니다.
좁은 틈을 이용한 기울임 삽입, 고속 충돌, 흔들리는 베이스 및 마찰 추정은 아직 별도 검증 대상입니다.

<a id="section-2"></a>

## 재생성

OpenUSD, trimesh, numpy, scipy, python-fcl이 있는 Python 환경에서 저장소 루트(`~/mk2_ws/src/xSting`)를 기준으로 실행합니다.

```bash
python xsting_description/scripts/build_storage_usd.py
```

Isaac Sim에서 검사 장면을 연 뒤 Script Editor로 `scripts/check_storage_runtime.py`를
실행하면 두 장면을 차례로 시험합니다(현재 stage를 교체). 그 다음 외부 Python에서
`scripts/validate_storage_runtime.py`를 실행해 원본 메시 대비 침투량을 검증합니다.
재생성하면 이전 runtime 결과는 새 모델을 검증한 결과로 간주하지 말고 다시 시험하세요.

이 스크립트는 원본 URDF/STL을 수정하지 않습니다. 원본 CAD가 바뀌면 바스킷 분리 조건과 검사 결과를 다시 확인하세요.
기존 cuMotion XRDF는 구 기반 계획용 근사입니다. 이번 SDF 개선은 PhysX 접촉 형상에 적용되며,
cuMotion이 바스킷 내부까지 정밀 삽입 계획을 지원한다고 검증한 것은 아닙니다.

참고: [NVIDIA SDF 충돌 지원](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/rigid_bodies_articulations/collision.html).

<a id="section-3"></a>

## 카메라 및 초음파 방식 거리 센서

두 storage 프리셋과 검사 장면에 센서 prim을 포함했습니다. 원본 URDF의 장착 프레임을 따릅니다.

| 센서 이름 | 출력 | 설정 |
|---|---|---|
| `front_rgb`, `left_rgb`, `right_rgb` | RGB | 640×480, 30 Hz, 수평 FOV 69° |
| `front_depth`, `left_depth`, `right_depth` | 광축 방향 거리(m) | 640×480, 30 Hz, 수평 FOV 87° |
| `arm_rgb` | 팔 장착 RGB | 640×360, 30 Hz, 기존 Arducam 보정 P를 1/3로 축소 |
| `ultrasonic_front` | 콘 안의 최단 거리(m) | 0.02~4 m, 전체 각도 15°, 25개 ray, 읽기 15 Hz |
| `lidar_2d` | 360° 수평 2D 거리 배열(m) | 0.145~12 m, 360°, 720개 ray, 읽기 10 Hz (`s.read_lidar()`) |

RealSense 카메라의 내부 파라미터는 수평 FOV와 정사각 픽셀을 가정한 명목값입니다.
깊이는 이상적인 렌더 거리이며 스테레오 매칭·적외선 패턴·실기 노이즈는 모사하지 않습니다.
팔 카메라는 rectified P를 사용하므로 원본 왜곡 계수는 다시 적용하지 않습니다.
초음파는 PhysX raycast 기반 기하학 근사이며 음파 반사·재질·다중경로는 모사하지 않습니다.
센서 파라미터는 `../../config/isaac_sensors.json`에 기록합니다.

Isaac Sim 6.1에서 장면을 연 뒤 Script Editor로 `scripts/start_isaac_sensors.py`를 실행하고 Play합니다.
렌더 제품은 요청한 카메라만 할당합니다(기본 전면 RGB). 다른 카메라도 아래처럼 읽을 수 있습니다.
첫 호출 직후에는 렌더 워밍업 때문에 `None`일 수 있으므로 여러 프레임 뒤 다시 읽으세요.

```python
import builtins
s = builtins.xsting_sensors
rgb = s.read_camera('front_rgb')        # H×W×3 uint8
depth = s.read_camera('front_depth')    # H×W×1 float32, metres
arm = s.read_camera('arm_rgb')
distance = s.read_ultrasonic()         # dict; range_m=inf means no hit
s.disable_camera('front_depth')       # release an unused render product
```

장면을 다시 열면 런타임 스크립트도 다시 실행하세요. 저장된 prim만으로 ROS 토픽이 생기지는 않으며,
이번 센서 출력은 Python 학습 환경용입니다. ROS 영상/Range publisher는 별도 연결 대상입니다.
`scripts/check_isaac_sensors.py`는 카메라 7개 출력, 0.5 m 거리 표적, RealSense 본체 6방향 충돌 ray를
검사하며 결과를 `sensor_validation.json`에 기록합니다. 임시 시험 표적은 session layer에서 제거합니다.
현재 검사에서 카메라 7개 출력과 RealSense 본체 6방향 충돌 검출이 통과했으며,
깊이 카메라 3개와 초음파 방식 센서 모두 0.5 m 표적을 0.5 m로 측정했습니다.
본체 충돌은 convex cooking 근사이므로 원본 CAD 표면과 모든 지점이 정확히 일치한다는 의미는 아닙니다.
