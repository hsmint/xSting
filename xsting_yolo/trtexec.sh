#!/usr/bin/env bash
# ONNX → TensorRT 엔진 빌드 (trtexec)
# 사용법: ./trtexec.sh <이미지 수: 1|2> <모델: n|s> [대상: sign_v2|rg|all] [해상도: 1280x720|640x480] [fp16]
#   대상을 빼면 all(model/yolo-v8*), 해상도를 빼면 1280x720, fp16을 붙이면 FP16 ONNX를 쓴다
#   예) ./trtexec.sh 1 n sign_v2 640x480 fp16 → model/yolo_sign_v2/yolo-v8n/best_b1_640x480_fp16.engine
#   예) ./trtexec.sh 1 n rg 640x480 fp16      → model/yolo_sep/rg/yolo-v8n/best_b1_640x480_fp16.engine
#   예) ./trtexec.sh 2 s 640x480              → model/yolo-v8s/best_b2_640x480.engine
# export.sh로 먼저 같은 인자의 ONNX를 만들어 둬야 한다.
set -e

USAGE="사용법: $0 <이미지 수: 1|2> <모델: n|s> [대상: sign_v2|rg|all] [해상도: 1280x720|640x480] [fp16]"
BATCH=$1
SIZE=$2
TARGET="all"
RES=1280x720
SUFFIX=""

# 세 번째부터의 인자는 순서 상관없이 대상, 해상도 또는 fp16
for ARG in "${@:3}"; do
    case "$ARG" in
        sign_v2|rg|all)   TARGET=$ARG ;;
        1280x720|640x480) RES=$ARG ;;
        fp16)             SUFFIX=_fp16 ;;
        *)                echo "$USAGE"; exit 1 ;;
    esac
done

if [[ "$BATCH" != "1" && "$BATCH" != "2" ]] || [[ "$SIZE" != "n" && "$SIZE" != "s" ]]; then
    echo "$USAGE"
    exit 1
fi

# 스크립트가 있는 폴더에서 실행
cd "$(dirname "$0")"
case "$TARGET" in
    sign_v2) DIR="model/yolo_sign_v2/yolo-v8${SIZE}" ;;
    rg)      DIR="model/yolo_sep/rg/yolo-v8${SIZE}" ;;
    all)     DIR="model/yolo-v8${SIZE}" ;;
esac
NAME="${DIR}/best_b${BATCH}_${RES}${SUFFIX}"

if [[ ! -f "$NAME.onnx" ]]; then
    echo "ONNX가 없습니다: $NAME.onnx"
    echo "먼저 실행: ./export.sh $*"
    exit 1
fi

# 엔진 빌드 후 trtexec가 속도 측정까지 하고 결과를 출력한다
trtexec --onnx="$NAME.onnx" --saveEngine="$NAME.engine"

echo "완료: $NAME.engine"
