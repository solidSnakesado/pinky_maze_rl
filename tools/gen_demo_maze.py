"""
tools/gen_demo_maze.py - 종이벽돌 미로 월드(SDF)와 무늬 그림 생성

역할
  - maze_bricks.py 가 계산한 벽돌 배치로 핑키 시뮬용 SDF 월드를 만듬
  - 벽돌 무늬 그림 (빨강/파랑 x 면 3종)을 만듬
방법
  - 벽돌마다 상자(라이다/충돌용, 단색) + 윗면/옆면 4개에 무늬 평면
  - 무늬 평면은 바깥을 향하게 놓음 (핑키 world.py 의 albedo_map 평면은 한쪽 면만 있음)
실행법
  python3 tools/gen_demo_maze.py
  python3 tools/gen_demo_maze.py --out-dir /tmp/maze
"""

# ==================== 표준 라이브러리 ==============================
import argparse                                                                 # 명령행 인자
import warnings                                                                 # 짐벌락 경고 끄기
from pathlib                    import Path                                     # 경로 처리

# ==================== 외부 라이브러리 ==============================
import numpy as np                                                              # 벡터 계산
from PIL                        import Image, ImageDraw                         # 무늬 그림 생성
from scipy.spatial.transform    import Rotation                                 # 회전 -> rpy

# ==================== 이 저장소 ===================================
from maze_bricks import bricks, foam_walls                                      # 벽돌/폼보드 배치

# ==================== 출력 설정 ===================================
GAP         = 0.0005                                                            # 무늬 평면을 면에서 띄우는 거리
PX_MM       = 4                                                                 # 무늬 그림 해상도 (픽셀/mm)
COLORS      = {"U": ("red", (224, 50, 43)), "S": ("blue", (47, 98, 217))}       # 세로 빨강, 가로 파랑
FOAM_RGBA   = "0.93 0.93 0.91 1"                                                # 폼보드 색
PKG_URI     = "package://pinky_maze_bringup/worlds"                             # 설치 후 그림 경로
FACES       = {(190, 95): "big", (190, 50): "side", (95, 50): "end"}            # 면 크기(mm) -> 그림 종류

# ==================== 무늬 그림 ===================================
def texture(rgb, w_mm, h_mm):
    """작은 벽돌(32 x 16 mm, 줄눈 3 mm) 엇갈림 쌓기 무늬"""
    img     = Image.new("RGB", (w_mm * PX_MM, h_mm * PX_MM), (242, 242, 240))   # 줄눈 흰색
    draw    = ImageDraw.Draw(img)
    bw, bh, m = 32, 16, 3
    for row in range(-1, h_mm // bh + 2):
        shift = (bw // 2) * (row % 2)
        for col in range(-1, w_mm // bw + 2):
            x0, y0 = col * bw - shift + m / 2, row * bh + m / 2
            draw.rectangle([x0 * PX_MM, y0 * PX_MM, (x0 + bw - m) * PX_MM, (y0 + bh - m) * PX_MM], fill=rgb)
    return img

# ==================== SDF 작성 ====================================
def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)                                 # 숫자 목록 -> 문자열

def pose(xyz, rot=None):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")                                         # 짐벌락 경고 무시
        rpy = Rotation.from_matrix(rot).as_euler("xyz") if rot is not None else np.zeros(3)
    return fmt((*xyz, *rpy))

def box_xml(name, lo, hi, rgba):
    size, mid   = hi - lo, (hi + lo) / 2
    geo         = f"<geometry><box><size>{fmt(size)}</size></box></geometry>"
    return (f"<collision name=\"{name}_c\"><pose>{pose(mid)}</pose>{geo}</collision>\n"
            f"<visual name=\"{name}_v\"><pose>{pose(mid)}</pose>{geo}"
            f"<material><diffuse>{rgba}</diffuse></material></visual>")

def face_xml(name, kind, lo, hi):
    """윗면과 옆면 4개에 무늬 평면 (바깥 방향)"""
    out, mid, half = [], (hi + lo) / 2, (hi - lo) / 2
    for k, (axis, sign) in enumerate([(0, 1), (0, -1), (1, 1), (1, -1), (2, 1)]):
        n       = np.zeros(3); n[axis] = sign                                                          # 면 법선
        inplane = [a for a in range(3) if a != axis]
        dims    = {a: 2 * half[a] for a in inplane}
        ua      = max(inplane, key=lambda a: dims[a])                           # 긴 쪽 = 그림 가로
        u       = np.zeros(3); u[ua] = 1
        va      = [a for a in inplane if a != ua][0]                            # 짧은 쪽 = 그림 세로
        v       = np.cross(n, u)
        center  = mid + n * (half[axis] + GAP)
        mm      = tuple(sorted((round(d * 1000) for d in dims.values()), reverse=True))
        png     = f"brick_{COLORS[kind][0]}_{FACES[mm]}.png"
        rot     = np.column_stack([u, v, n])
        out.append(f"<visual name=\"{name}_f{k}\"><pose>{pose(center, rot)}</pose>"
                   f"<geometry><plane><normal>0 0 1</normal><size>{dims[ua]:.4g} {dims[va]:.4g}</size></plane></geometry>"
                   f"<material><pbr><metal><albedo_map>{PKG_URI}/{png}</albedo_map></metal></pbr></material></visual>") 
    return out

def main():
    ap      = argparse.ArgumentParser(description="Generate paper-brick demo maze world (SDF) and textures")
    ap.add_argument("--out-dir", default="src/pinky_maze_bringup/worlds", help="output directory")
    ap.add_argument("--name", default="demo_maze_a", help="world file name (without .world)")
    args    = ap.parse_args()
    out     = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    for kind, (cname, rgb) in COLORS.items():
        for (w, h), fname in FACES.items():
            texture(rgb, w, h).save(out / f"brick_{cname}_{fname}.png")
    parts   = []
    bs      = bricks()
    for k, (kind, lo, hi) in enumerate(bs):
        rgba    = " ".join(f"{c / 255:.3g}" for c in COLORS[kind][1]) + " 1"
        parts  += [box_xml(f"brick{k}", lo, hi, rgba)] + face_xml(f"brick{k}", kind, lo, hi)
    for k, (lo, hi) in enumerate(foam_walls()):
        parts.append(box_xml(f"foam{k}", lo, hi, FOAM_RGBA))
    body = "\n".join(parts)
    sdf = (f"<?xml version=\"1.0\"?>\n<sdf version=\"1.9\">\n<world name=\"{args.name}\">\n"
           f"<model name=\"{args.name}\"><static>true</static><link name=\"walls\">\n{body}\n"
           f"</link></model>\n</world>\n</sdf>\n")
    (out / f"{args.name}.world").write_text(sdf)
    n_u = sum(1 for b in bs if b[0] == "U")
    print(f"bricks: upright {n_u}, lying {len(bs) - n_u}, total {len(bs)}")
    print(f"wrote {out / (args.name + '.world')} and 6 textures")

if __name__ == "__main__":
    main()