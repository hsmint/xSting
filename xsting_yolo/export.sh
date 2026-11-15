#!/usr/bin/env bash
# YOLO .pt → ONNX 내보내기
# 사용법: ./export.sh <이미지 수: 1|2> <모델: n|s> [대상: sign_v2|rg|all] [해상도: 1280x720|640x480] [fp16]
#   대상을 빼면 all(model/yolo-v8*), 해상도를 빼면 1280x720, fp16을 붙이면 내부를 FP16으로 변환 (입출력은 FP32)
#   예) ./export.sh 1 n sign_v2 640x480 fp16 → model/yolo_sign_v2/yolo-v8n/best_b1_640x480_fp16.onnx
#   예) ./export.sh 1 n rg 640x480 fp16      → model/yolo_sep/rg/yolo-v8n/best_b1_640x480_fp16.onnx
#   예) ./export.sh 2 n 640x480              → model/yolo-v8n/best_b2_640x480.onnx
set -e

USAGE="사용법: $0 <이미지 수: 1|2> <모델: n|s> [대상: sign_v2|rg|all] [해상도: 1280x720|640x480] [fp16]"
BATCH=$1
SIZE=$2
TARGET="all"
RES=1280x720
FP16=""

# 세 번째부터의 인자는 순서 상관없이 대상, 해상도 또는 fp16
for ARG in "${@:3}"; do
    case "$ARG" in
        sign_v2|rg|all)   TARGET=$ARG ;;
        1280x720|640x480) RES=$ARG ;;
        fp16)             FP16=fp16 ;;
        *)                echo "$USAGE"; exit 1 ;;
    esac
done

if [[ "$BATCH" != "1" && "$BATCH" != "2" ]] || [[ "$SIZE" != "n" && "$SIZE" != "s" ]]; then
    echo "$USAGE"
    exit 1
fi

# 해상도별 모델 입력 크기 (높이 너비)
# 720은 32의 배수가 아니라서 736으로 내보낸다 (위아래 8픽셀씩 패딩)
case "$RES" in
    1280x720) IMGSZ="736 1280" ;;
    640x480)  IMGSZ="480 640" ;;
esac

# 스크립트가 있는 폴더에서 실행
cd "$(dirname "$0")"
case "$TARGET" in
    sign_v2) DIR="model/yolo_sign_v2/yolo-v8${SIZE}" ;;
    rg)      DIR="model/yolo_sep/rg/yolo-v8${SIZE}" ;;
    all)     DIR="model/yolo-v8${SIZE}" ;;
esac

if [[ ! -f "$DIR/best.pt" ]]; then
    echo "모델 가중치가 없습니다: $DIR/best.pt"
    exit 1
fi

if [[ -n "$FP16" ]]; then
    .venv/bin/python export_onnx.py --model "$DIR/best.pt" --batch "$BATCH" --imgsz $IMGSZ --fp16
    OUT="$DIR/best_b${BATCH}_${RES}_fp16.onnx"
else
    .venv/bin/python export_onnx.py --model "$DIR/best.pt" --batch "$BATCH" --imgsz $IMGSZ
    OUT="$DIR/best_b${BATCH}_${RES}.onnx"
fi

# 배치·해상도·정밀도마다 파일 이름을 달리해서 서로 덮어쓰지 않게 한다
mv "$DIR/best.onnx" "$OUT"
echo "완료: $OUT"
