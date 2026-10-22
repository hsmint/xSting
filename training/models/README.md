# 배포 모델
실제 모델은 아직 없습니다. arm/v001/ 아래 arm.onnx, manifest.json, evaluation.json을 함께 저장하세요.
manifest 필수 항목은 training/validate_model.py 참조. 검증 명령:

```bash
python3 training/validate_model.py training/models/arm/v001/manifest.json training/models/arm/v001/arm.onnx
```

이 명령은 메타데이터와 파일 해시만 검증합니다. ONNX 실행·수치 일치·성공률 검증은 RL 담당 작업입니다.
