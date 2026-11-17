"""학습된 DOPE 가중치(.pth)를 DOPE/Isaac ROS 표준 꼭짓점 순서로 바꿔 저장

  python convert_to_dope_order.py IN.pth OUT.pth [--check IMG.jpg]

왜 필요한가
- 우리 합성 데이터(gen_dope_synth.py)의 꼭짓점 순서: 0(+x,+y,+z) 1(-x,+y,+z) 2(-x,-y,+z) 3(+x,-y,+z) 4~7 은 z 를 - 로
- DOPE(Cuboid3d)/Isaac ROS DOPE 의 순서:          0(+x,-y,+z) 1(-x,-y,+z) 2(-x,+y,+z) 3(+x,+y,+z) 4~7 은 z 를 - 로
- 그대로 쓰면 PnP 가 거울상으로 맞춰져 자세가 틀림 -> 신경망 출력 채널 순서만 [3,2,1,0,7,6,5,4,8] 로 바꾸면 해결
  (재학습 불필요. 물체 좌표계 x,y,z 와 치수 [3,3,6] 은 그대로)
- 결과 파일은 'module.' 접두어를 없앤 일반 가중치 -> inference.py 에서 --parallel 없이 사용
"""
import argparse, sys, os
from collections import OrderedDict
import torch

PERM = [3, 2, 1, 0, 7, 6, 5, 4, 8]                       # 새 채널 j = 기존 채널 PERM[j]
AFF = [c for v in PERM[:8] for c in (2 * v, 2 * v + 1)]  # affinity 는 꼭짓점마다 (x, y) 2채널
LAST = "12"   # create_stage 의 마지막 conv 이름 (첫 단계 m1_*: "8")
FIRST = "0"


def convert(sd):
    sd = OrderedDict((k[7:] if k.startswith("module.") else k, v.clone()) for k, v in sd.items())
    for s in range(1, 7):
        last = "8" if s == 1 else LAST
        for br, perm in (("2", PERM), ("1", AFF)):          # m*_2 = belief(9), m*_1 = affinity(16)
            for p in ("weight", "bias"):
                k = f"m{s}_{br}.{last}.{p}"
                assert sd[k].shape[0] == len(perm), (k, sd[k].shape)
                sd[k] = sd[k][perm]
        if s >= 2:                                            # 다음 단계 입력 = cat(belief9, aff16, feat128)
            inperm = PERM + [9 + c for c in AFF] + list(range(25, 153))
            for br in ("2", "1"):
                k = f"m{s}_{br}.{FIRST}.weight"
                assert sd[k].shape[1] == 153, (k, sd[k].shape)
                sd[k] = sd[k][:, inperm]
    return sd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inp"); ap.add_argument("out")
    ap.add_argument("--check", help="변환 전후 출력이 채널 순서만 다른지 확인할 사진")
    a = ap.parse_args()
    src = torch.load(a.inp, map_location="cpu")
    dst = convert(src)
    torch.save(dst, a.out)
    print("saved", a.out)
    if a.check:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "Deep_Object_Pose", "common"))
        from models import DopeNetwork
        from PIL import Image
        import torchvision.transforms as T
        x = T.Compose([T.Resize(400), T.ToTensor(), T.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225))])(
            Image.open(a.check).convert("RGB")).unsqueeze(0).cuda()
        n0 = DopeNetwork().cuda().eval(); n0.load_state_dict(OrderedDict((k[7:] if k.startswith("module.") else k, v) for k, v in src.items()))
        n1 = DopeNetwork().cuda().eval(); n1.load_state_dict(dst)
        with torch.no_grad():
            b0, a0 = n0(x); b1, a1 = n1(x)
        eb = (b1[-1][0] - b0[-1][0][PERM]).abs().max().item()
        ea = (a1[-1][0] - a0[-1][0][AFF]).abs().max().item()
        print(f"check: belief max diff {eb:.2e}, affinity max diff {ea:.2e} (0 에 가까우면 정상)")


if __name__ == "__main__":
    main()
