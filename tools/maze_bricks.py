"""
tools/maze_bricks.py - 미로 정의를 종이벽돌 배치(상자 목록)로 바꾸는 계산 모듈

역할
  - 미로 정의(칸 수, 벽 목록, 출구)와 치수를 한곳에 둔다
  - 벽을 종이벽돌(19 x 9.5 x 5 cm)로 쌓았을 때 벽돌 하나하나의 위치를 계산
  - 벽 한 묶음 = 세로 벽돌 1개(높이 190) + 가로 벽돌 2개 2단 = 길이 285 mm
방법
  - VW/HW 벽 목록 -> 같은 선 위 벽을 구간으로 묶음 -> 구간 끝 종류 (외곽/끊김/맞닿음/소유)로 길이 계산
  - 구간 길이를 285 mm 묶음으로 채움 (나누어지지 않으면 오류)
실행법
  모듈로 사용: from maze_bricks import bricks, foam_walls (tools/gen_demo_maze.py)
"""

# ==================== 외부 라이브러리 ==============================
import numpy as np                                                          # 좌표 배열

# ==================== 미로 정의 (데모 미로 A) ======================
COLS, ROWS  = 4, 3                                                          # 칸 수 (열, 행)
START       = (1, 1)                                                        # 출발 칸 = 월드 원점
VW          = {0: [0], 1: [2], 2: [2]}                                      # 행 r: 동쪽 벽이 있는 열 c
HW          = {0: [2], 1: [0]}                                              # 행 r: 북쪽 벽이 있는 열 c
EXITS       = [(3, 2, "east")]                                              # 출구 (열, 행, 방향)

# ==================== 치수 (m) ===================================
P, T, H     = 0.285, 0.05, 0.19                                             # 칸 간격, 벽 두께, 벽 높이
FOAM        = 0.005                                                         # 외곽 폼보드 두께
L_S, L_U    = 0.19, 0.095                                                   # 가로 벽돌 길이, 세로 벽돌 폭       

# ==================== 격자 좌표 ===================================
def gx(i):
    return (i - 0.5 - START[0]) * P                                         # 세로 격자선 i 의 x

def gy(j):
    return (j - 0.5 - START[1]) * P                                         # 가로 격자선 j 의 y

# ==================== 벽 구간 만들기 ===============================
def merge(spans):
    """선 하나 위의 (시작, 끝) 칸 구간을 이어진 것끼리 묶음"""
    runs = []
    for a, b in sorted(spans):
        if runs and runs[-1][1] == a:
            runs[-1][1] = b                                                 # 이어지면 늘림
        else:
            runs.append([a, b])
    return runs

def wall_runs():
    """세로 구간 {i: [[j0, j1]]}, 가로 구간 {j: [[i0, i1]]}"""
    vert, horz = {}, {}
    for r, cols in VW.items():
        for c in cols:
            vert.setdefault(c + 1, []).append((r, r + 1))
    for r, cols in HW.items():
        for c in cols:
            horz.setdefault(r + 1, []).append((c, c + 1))
    return ({k: merge(v) for k, v in vert.items()},
            {k: merge(v) for k, v in horz.items()})

def end_kind(axis, line, node, others, size):
    """구간 끝 종류: outer(외곽), own(모서리 소유), butt(상대 면에 맞닿음), free(끊김)"""
    if node in (0, size):
        return "outer"
    for a, b in others.get(node, []):
        if a < line < b:
            return "butt"                                                   # 상대 구간이 지나감
        if line in (a, b):
            return "own" if axis == "y" else "butt"                         # 꺾임: 세로가 소유
    return "free"

def end_pos(kind, node, grid, size, sign):
    """구간 끝의 실제 좌표 (sign: 구간 바깥 방향)"""
    if kind in ("outer", "butt"):
        return grid(node) - sign * T / 2                                    # 외곽 안쪽 면, 상대 벽 면
    return grid(node) + sign * T / 2                                        # own, free: 마디를 덮음

# ==================== 벽돌 배치 ====================================
def layout(axis, line, run, others, size):
    """구간 하나를 벽돌 묶음으로 채움 -> [(종류, 최소점, 최대점)]"""
    grid    = gy if axis == "y" else gx
    k0      = end_kind(axis, line, run[0], others, size)
    k1      = end_kind(axis, line, run[1], others, size)
    a       = end_pos(k0, run[0], grid, size, -1)
    b       = end_pos(k1, run[1], grid, size, +1)
    n       = (b - a) / (L_S + L_U)
    if abs(n - round(n)) > 1e-6:
        raise ValueError(f"wall on {axis}-line {line} ({b - a:.4f} m) is not a multiple of 0.285 m") 
    seq     = ["S", "U"] if k1 in ("own", "free") or k0 not in ("own", "free") else ["U", "S"]
    c       = (gx if axis == "y" else gy)(line)                             # 벽 중심선 좌표
    out     = []
    for kind in seq * round(n):
        length  = L_S if kind == "S" else L_U
        lo, hi  = (a, a + length)
        a       = hi
        zs      = [(0, H)] if kind == "U" else [(0, H / 2), (H / 2, H)]     # 가로는 2단
        for z0, z1 in zs:
            if axis == "y":
                out.append((kind, np.array([c - T / 2, lo, z0]), np.array([c + T / 2, hi, z1])))
            else:
                out.append((kind, np.array([lo, c - T / 2, z0]), np.array([hi, c + T / 2, z1])))
    return out

def bricks():
    vert, horz  = wall_runs()
    out         = []
    for i, runs in vert.items():
        for run in runs:
            out += layout("y", i, run, horz, ROWS)
    for j, runs in horz.items():
        for run in runs:
            out += layout("x", j, run, vert, COLS)
    return out

def foam_walls():
    """외곽 폼보드 (출구 칸은 비움) -> [(최소점, 최대점)]"""
    x0, x1  = gx(0) + T / 2, gx(COLS) - T / 2                                # 외곽 안쪽 면
    y0, y1  = gy(0) + T / 2, gy(ROWS) - T / 2
    sides   = {"south": (x0 - FOAM, x1 + FOAM, y0 - FOAM, y0, "x"),
               "north": (x0 - FOAM, x1 + FOAM, y1, y1 + FOAM, "x"),
               "west": (x0 - FOAM, x0, y0, y1, "y"),
               "east": (x1, x1 + FOAM, y0, y1, "y")}
    out     = []
    for name, (ax0, ax1, ay0, ay1, along) in sides.items():
        cuts    = sorted(((gx(c + 0.5) if along == "x" else gy(r + 0.5)) for c, r, s in EXITS if s == name))
        lo      = ax0 if along == "x" else ay0
        hi      = ax1 if along == "x" else ay1
        for mid in cuts + [None]:
            end = hi if mid is None else mid - (P - T) / 2
            if end > lo:
                out.append((np.array([ax0 if along == "y" else lo, ay0 if along == "x" else lo, 0]),
                            np.array([ax1 if along == "y" else end, ay1 if along == "x" else end, H])))
            lo = hi if mid is None else mid + (P - T) / 2
    return out
