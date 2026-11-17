# DOPE 사각기둥 퍽 모델 (26-11-14)

빨강/초록 사각기둥 퍽(3×3×6 cm)의 3D 위치와 자세를 카메라 사진 한 장으로 추정하는 DOPE 모델입니다.

## 들어 있는 것
| 파일 | 설명 |
|---|---|
| model/square_puck_dope_e60.pth | 모델 가중치 (60 epoch, DOPE 표준 꼭짓점 순서로 변환됨). 깃에는 없음 → 아래 "모델 받기" 참고 |
| train_header.txt | 학습 설정 |
| datagen/gen_dope_synth.py | Isaac Sim 합성 데이터 생성 스크립트 |
| requirements.txt | 학습/추론 파이썬 패키지 |
| config/dope_square_puck.yaml | 치수, 추론 설정 |
| code/infer_dope.py | 사진 폴더에 모델 실행 → 결과 그림 + json |
| code/eval_dope.py | 검출 함수와 합성 val 평가 |
| code/convert_to_dope_order.py | 학습 폴더의 원본 .pth → 표준 순서 변환 (이미 변환된 모델은 다시 할 필요 없음) |
| code/utils_xsting.diff | 학습 때 DOPE 코드에 넣은 수정 (jpg 읽기, 물체 중심 crop, 노이즈 세기) |

## 모델 받기
가중치(192MB)는 GitHub 용량 제한 때문에 레포가 아니라 Release에 올려 두었습니다.
```bash
mkdir -p model
wget -O model/square_puck_dope_e60.pth \
  https://github.com/hsmint/xSting/releases/download/dope-square-puck-v1/square_puck_dope_e60.pth
```

## 학습 정보
- Isaac Sim 합성 사진 약 13,000장
  - 서 있는 퍽 10,000장 + 누운 퍽 5,000장 중 train 분할
  - 옆면 태그: 전부 있음 / 일부만 있음 / 없음을 고르게 섞음
- DOPE (NVlabs Deep_Object_Pose), batch 8, lr 1e-4, 60 epoch, RTX 5060에서 약 20시간

## 사용 방법
```bash
source ~/xsting_ws/dope/.venv/bin/activate
cd code
python infer_dope.py --weights ../model/square_puck_dope_e60.pth \
  --images 사진폴더 --out 결과폴더 --fx 924.28 --fy 924.28 --cx 640 --cy 360
```
- fx, fy, cx, cy는 실제 카메라 값을 넣으세요. 위 값은 시뮬레이션 카메라(1280×720) 값입니다.
- 결과 json의 `location_cm`은 카메라 좌표(OpenCV: x 오른쪽, y 아래, z 앞) 기준이고 단위는 cm입니다.

## 꼭 알아야 할 점
1. **빨강과 초록을 구분하지 않습니다.** 한 클래스("사각기둥 퍽")로 학습했습니다. 색은 YOLO 결과나 퍽 중심 픽셀 색으로 구분하세요.
2. **사진을 줄이지 마세요.** 1280×720 원본 그대로 넣어야 합니다. DOPE 기본값처럼 높이 400으로 줄이면 퍽이 너무 작아져 잘 못 찾습니다.
3. **DOPE 기본 `inference.py`는 쓰지 마세요.**
   - 이 버전은 꼭짓점 9개가 모두 보여야 결과를 냅니다. 작은 퍽은 대부분 버려집니다.
   - `infer_dope.py`는 9개 중 6개 이상이면 자세를 구합니다(sigma 1).
4. **퍽은 90°마다 같은 모양입니다.** 그래서 긴 축(z) 둘레 회전은 90° 단위로 구분할 수 없습니다. 긴 축의 방향과 위치는 정상입니다.

## 간단한 성능 (합성 val 80장, 참고용)
| 항목 | 값 |
|---|---|
| 검출률 | 약 80% (거의 다 보이는 퍽 기준) |
| 위치 오차 중앙값 | 약 1.2 cm |

- 놓치는 경우는 주로 1 m보다 먼 퍽, 사진 가장자리에서 잘린 퍽, 바짝 붙은 두 퍽입니다.
- 실제 카메라 사진으로는 아직 검증하지 않았습니다.

## Isaac ROS DOPE로 쓰려면
1. Isaac ROS의 `isaac_ros_dope/scripts/dope_converter.py`로 이 .pth를 ONNX로 변환합니다. 변환된 모델은 DOPE 표준 순서라 그대로 쓸 수 있습니다.
2. 객체 치수는 `[3.0, 3.0, 6.0]`(cm)입니다.
3. 네트워크 입력 크기는 카메라 해상도에 맞게 크게 잡으세요(예: 1280×720).
4. Isaac ROS 디코더가 꼭짓점이 몇 개 있어야 결과를 내는지 실제 로봇에서 확인하세요.
