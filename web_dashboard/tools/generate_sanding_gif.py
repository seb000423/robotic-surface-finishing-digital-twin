#!/usr/bin/env python3
"""Before/After 비교용 '제거량 히트맵' 애니메이션 GIF 생성기.

원본 차량 사진(public/images/bmw_z4.png)에서 차체 외형만 마스킹해, 부위별로 다른
제거 깊이(최대 3.0μm, 평균 ~1.9μm)를 히트맵으로 입히고 앞→뒤로 샌딩이 진행되는
모습을 애니메이션으로 만든다. 제거량 분포는 난수 블롭이라 부위마다 다르게 나온다.

사용:
  python3 tools/generate_sanding_gif.py [--seed N] [--in IN.png] [--out OUT.gif]
기본 출력: public/images/sanding_heatmap.gif
"""
import argparse
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

MAX_UM = 3.0    # 최대 제거 깊이
MEAN_UM = 1.9   # 평균 제거 깊이(차체 영역)

# 히트맵 컬러맵 앵커 (낮음=파랑 → 높음=빨강), 값은 0..255
_ANCH = [
    (0.00, (0, 0, 180)),
    (0.25, (0, 170, 220)),
    (0.50, (0, 200, 70)),
    (0.70, (240, 220, 0)),
    (0.85, (245, 140, 0)),
    (1.00, (220, 0, 0)),
]


def colormap(t):
    """t: 임의 shape 배열(0..1) → (..., 3) 0..255 색."""
    t = np.clip(np.asarray(t, np.float32), 0, 1)
    out = np.zeros(t.shape + (3,), np.float32)
    for i in range(len(_ANCH) - 1):
        t0, c0 = _ANCH[i]
        t1, c1 = _ANCH[i + 1]
        m = (t >= t0) & (t <= t1)
        if not m.any():
            continue
        u = ((t[m] - t0) / (t1 - t0))[:, None]
        out[m] = np.array(c0, np.float32) * (1 - u) + np.array(c1, np.float32) * u
    return out


# 자동차를 감싸는 사각형(이 사진 전용, 375x282) = GrabCut 초기 영역. (x, y, w, h)
_CAR_RECT = (33, 40, 327, 216)


def build_car_mask(rgb_u8):
    """GrabCut 으로 자동차 외형(테두리)을 따서 차체 전체를 1, 나무 바닥을 0으로.
    색만으론 밝은 바닥과 흰 차체가 안 나뉘므로, 사각형 초기화 GrabCut(색 GMM+공간
    평활)으로 실루엣을 분리한 뒤 가장 큰 덩어리만 남기고 살짝 페더링한다."""
    import cv2
    import scipy.ndimage as ndi
    H, W = rgb_u8.shape[:2]
    bgr = cv2.cvtColor(rgb_u8, cv2.COLOR_RGB2BGR)
    gc = np.zeros((H, W), np.uint8)
    bgd = np.zeros((1, 65), np.float64)
    fgd = np.zeros((1, 65), np.float64)
    cv2.grabCut(bgr, gc, _CAR_RECT, bgd, fgd, 7, cv2.GC_INIT_WITH_RECT)
    m = (gc == cv2.GC_FGD) | (gc == cv2.GC_PR_FGD)

    m = ndi.binary_fill_holes(m)
    lab, n = ndi.label(m)                       # 가장 큰 덩어리(차체)만 유지
    if n > 1:
        sizes = ndi.sum(np.ones_like(lab), lab, index=range(1, n + 1))
        m = lab == (int(np.argmax(sizes)) + 1)
    m = ndi.binary_closing(m, iterations=2)
    m = ndi.binary_fill_holes(m)
    m = ndi.binary_erosion(m, iterations=1)     # 1px 안쪽으로 → 바닥 헤일로 방지

    mf = Image.fromarray((m * 255).astype(np.uint8), "L").filter(ImageFilter.GaussianBlur(1.0))
    return np.asarray(mf, np.float32) / 255.0


def depth_field(H, W, mask, seed):
    """난수 가우시안 블롭 합으로 0..MAX_UM 제거량 필드 생성(평균≈MEAN_UM)."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    field = np.zeros((H, W), np.float32)
    for _ in range(10):
        cx = rng.uniform(0, W)
        cy = rng.uniform(0.30 * H, H)
        sx = rng.uniform(0.08 * W, 0.30 * W)
        sy = rng.uniform(0.08 * H, 0.30 * H)
        amp = rng.uniform(0.4, 1.0)
        field += amp * np.exp(-(((xx - cx) / sx) ** 2 + ((yy - cy) / sy) ** 2))

    sel = mask > 0.5
    if sel.sum() < 10:
        sel = np.ones_like(mask, bool)
    fmin, fmax = field[sel].min(), field[sel].max()
    f = np.clip((field - fmin) / (fmax - fmin + 1e-9), 0, 1)

    # 평균을 target(=MEAN/MAX)으로 맞추는 감마 보정
    target = MEAN_UM / MAX_UM
    g = 1.0
    for _ in range(60):
        cur = float((f[sel] ** g).mean())
        if abs(cur - target) < 1e-3 or cur <= 0:
            break
        g *= np.log(target + 1e-9) / np.log(cur + 1e-9)
        g = min(max(g, 0.1), 6.0)
    return (f ** g) * MAX_UM


def draw_legend(im):
    d = ImageDraw.Draw(im)
    W, H = im.size
    bw, bh, x0, y0 = 96, 8, 10, H - 16
    for k in range(bw):
        c = colormap(np.array([[k / (bw - 1)]]))[0, 0]
        d.line([(x0 + k, y0), (x0 + k, y0 + bh)], fill=tuple(int(v) for v in c))
    d.rectangle([x0, y0, x0 + bw, y0 + bh], outline=(255, 255, 255))
    try:
        font = ImageFont.load_default()
    except Exception:
        font = None
    d.text((x0, y0 - 11), "Removal 0 - 3 um", fill=(255, 255, 255), font=font)
    return im


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pub = os.path.normpath(os.path.join(here, "..", "public", "images"))
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--in", dest="inp", default=os.path.join(pub, "bmw_z4.png"))
    ap.add_argument("--out", dest="out", default=os.path.join(pub, "sanding_heatmap.gif"))
    args = ap.parse_args()

    img = Image.open(args.inp).convert("RGB")
    rgb = np.asarray(img, np.float32)
    H, W, _ = rgb.shape

    mask = build_car_mask(np.asarray(img))
    depth = depth_field(H, W, mask, args.seed)
    heat = colormap(depth / MAX_UM)              # HxWx3 (0..255)

    sel = mask > 0.5
    xx = np.arange(W)[None, :] * np.ones((H, 1), np.float32)
    xmin, xmax = float(xx[sel].min()), float(xx[sel].max())
    edge = max(8.0, (xmax - xmin) * 0.18)
    BLEND = 0.85                       # 차체를 확실히 덮음(배경 바닥은 그대로 → 구분)
    car_a = mask[..., None]

    # 배경과 또렷이 구분되게 차 실루엣 외곽선(항상 표시)
    edge_a = (((mask > 0.12) & (mask < 0.9)).astype(np.float32))[..., None] * 0.5
    OUTLINE = np.array([20, 28, 46], np.float32)

    def compose(reveal, shimmer=1.0):
        a = BLEND * car_a * np.clip(reveal[..., None], 0, 1) * shimmer
        out = rgb * (1 - a) + heat * a
        out = out * (1 - edge_a) + OUTLINE * edge_a       # 외곽선
        return np.clip(out, 0, 255).astype(np.uint8)

    frames = []
    F_rev, F_hold = 22, 12
    for i in range(F_rev):                        # 앞→뒤 샌딩 진행
        p = i / (F_rev - 1)
        bnd = xmin + p * ((xmax - xmin) + edge)
        reveal = np.clip((bnd - xx) / edge, 0, 1) * mask
        frames.append(compose(reveal))
    for j in range(F_hold):                        # 완료 후 은은한 셔머 + 정지
        sh = 0.92 + 0.08 * np.sin(j / F_hold * 2 * np.pi)
        frames.append(compose(mask, shimmer=sh))

    # 범례는 대시보드 우측 HTML 색상바로 표시(여기선 굽지 않음)
    pil = [Image.fromarray(f).convert("P", palette=Image.ADAPTIVE, colors=256)
           for f in frames]
    pil[0].save(args.out, save_all=True, append_images=pil[1:],
                duration=70, loop=0, disposal=2, optimize=True)
    cov = float((depth[sel]).mean())
    print(f"[gen] saved {args.out}  ({len(pil)} frames, {W}x{H})")
    print(f"[gen] depth μm  max={depth[sel].max():.2f}  mean={cov:.2f}  (target mean {MEAN_UM})")


if __name__ == "__main__":
    main()
