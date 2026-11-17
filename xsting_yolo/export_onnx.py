"""YOLO .pt 모델을 ONNX로 변환한다."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def get_args():
    """변환할 모델과 입력 크기를 명령행에서 받는다."""
    parser = argparse.ArgumentParser()

    # 입력 모델 (.pt 경로)
    parser.add_argument("--model", default="model/yolo-v8n/best.pt")

    # 모델 입력 형태 (배치, 이미지 크기: 한 값이면 정사각형, 두 값이면 높이 너비)
    parser.add_argument("--batch", type=int, default=1)
    parser.add_argument("--imgsz", type=int, nargs="+", default=[640])

    # 켜면 모델 내부를 FP16으로 변환한다 (입출력은 FP32 그대로)
    parser.add_argument("--fp16", action="store_true")
    return parser.parse_args()


def to_onnx(pt, batch, imgsz, fp16):
    """.pt 파일을 같은 폴더에 .onnx로 내보내고 경로를 돌려준다."""
    model = YOLO(pt)

    # quantize=16: FP16 변환 (예전 half=True와 같은 뜻)
    # CPU로 내보내야 입출력을 FP32로 남기고 내부만 FP16으로 바꾼다.
    # GPU로 내보내면 입력까지 FP16이 되어 추론 코드의 전처리를 고쳐야 한다.
    out = model.export(
        format="onnx",
        batch=batch,
        imgsz=imgsz,
        simplify=True,
        quantize=16 if fp16 else None,
        device="cpu",
    )
    return Path(out)


def main():
    args = get_args()

    onnx = to_onnx(args.model, args.batch, args.imgsz, args.fp16)
    print(f"ONNX: {onnx}")


if __name__ == "__main__":
    main()
