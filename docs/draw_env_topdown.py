# -*- coding: utf-8 -*-
"""Draw top-down (x-y plane) view of the Unity MuJoCo scene with dimensions.

Sources: libero_pick_up_red_block.xml (table, blocks, arm bases, camera).
All coordinates in MuJoCo world frame, meters.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

# ---- scene geometry (from XML) ----
TABLE_X = 1.2          # table top size (box size 0.6 0.5 0.025 -> full extent)
TABLE_Y = 1.0
TABLE_CX = 0.10        # 桌台中心（2026-08-11: 前移20cm→+0.20，后移10cm→+0.10）
BLOCK_HALF = 0.025     # 5cm cube

blocks = {
    "red":   (-0.25, -0.30),
    "green": (-0.35,  0.05),
    "blue":  (-0.30,  0.30),
}
BASE_L = (-0.70, 0.20)   # L_Base
BASE_R = (-0.70, -0.20)  # R_Base
PEDESTAL = (-0.70, 0.0)
CAMERA = (-0.85, 0.0)    # agentview_site (2026-08-11: -1.15 → -0.95 → -0.85)
REACH = 0.46             # 数据实测最远抓取 0.461m（红块）

fig, ax = plt.subplots(figsize=(11, 9))

# table
ax.add_patch(mpatches.Rectangle((TABLE_CX - TABLE_X/2, -TABLE_Y/2), TABLE_X, TABLE_Y,
                                facecolor="#E8D5B0", edgecolor="#8B6B3F", lw=2))
ax.text(0.85, 0.0, f"桌台 1.2×1.0 m\n中心 x={TABLE_CX:+.2f}（顶面 z=0）",
        ha="left", va="center", fontsize=11, color="#5C4423")

# blocks
for name, (bx, by) in blocks.items():
    c = {"red": "#D93025", "green": "#1E8E3E", "blue": "#1A73E8"}[name]
    ax.add_patch(mpatches.Rectangle((bx-BLOCK_HALF, by-BLOCK_HALF), 0.05, 0.05,
                                    facecolor=c, edgecolor="black", lw=1.2))
    ax.text(bx, by - 0.055, f"{name}\n({bx:.2f}, {by:+.2f})", ha="center", va="top",
            fontsize=9.5, color="black")

# arm bases + pedestal
for tag, (bxx, byy) in [("R 臂", BASE_R), ("L 臂", BASE_L)]:
    ax.plot(bxx, byy, "o", color="0.25", ms=13, zorder=5)
    ax.text(bxx, byy + 0.035, tag, ha="center", va="bottom", fontsize=11, fontweight="bold")
    ax.add_patch(mpatches.Circle((bxx, byy), REACH, fill=False, ls="--",
                                 color="0.45", lw=1.0))
ax.plot(*PEDESTAL, "o", color="0.55", ms=8, zorder=4)
ax.text(PEDESTAL[0] - 0.03, PEDESTAL[1] - 0.05, "共享基座立柱", ha="right", va="top", fontsize=9, color="0.3")
ax.text(-0.92, 0.09, "虚线圆 = 数据实测最远抓取 0.461 m（可达上限 0.46）",
        ha="left", va="top", fontsize=8.5, color="0.35", rotation=90)

# camera（目视目标固定为原点，不随桌移动）
ax.plot(*CAMERA, "^", color="#E37400", ms=15, zorder=6)
ax.text(CAMERA[0] - 0.06, CAMERA[1] + 0.05, "agentview 相机\n(-0.85, 0, 0.40)", ha="center",
        va="bottom", fontsize=9, color="#E37400")
ax.annotate("", xy=(-0.35, 0), xytext=(-0.83, 0),
            arrowprops=dict(arrowstyle="->", color="#E37400", lw=1.5))
ax.text(0.0, 0.60, "注：方块为独立 freejoint，不随桌移动——\n桌沿在 x=-0.50，方块落在距桌沿 0.15~0.25 m 的窄带",
        ha="center", va="top", fontsize=9, color="#5C4423",
        bbox=dict(boxstyle="round,pad=0.3", fc="#F5EFE2", ec="#8B6B3F", lw=0.8))

def dist(p, q):
    return ((p[0]-q[0])**2 + (p[1]-q[1])**2) ** 0.5

def annot(p, q, label, dx=0.0, dy=0.0, color="0.25"):
    mx, my = (p[0]+q[0])/2 + dx, (p[1]+q[1])/2 + dy
    ax.annotate("", xy=q, xytext=p,
                arrowprops=dict(arrowstyle="<->", color=color, lw=1.1,
                                shrinkA=0, shrinkB=0))
    ax.text(mx, my, label, ha="center", va="center", fontsize=8.8,
            color=color, bbox=dict(boxstyle="round,pad=0.15", fc="white", ec=color, lw=0.6))

# base -> block distances
annot(BASE_R, blocks["red"],   f"{dist(BASE_R, blocks['red']):.3f} m",   dy=0.03)
annot(BASE_L, blocks["blue"],  f"{dist(BASE_L, blocks['blue']):.3f} m",  dy=0.03)
annot(BASE_L, blocks["green"], f"{dist(BASE_L, blocks['green']):.3f} m", dx=-0.035, dy=0.05)
annot(BASE_R, blocks["green"], f"{dist(BASE_R, blocks['green']):.3f} m", dx=-0.035, dy=-0.05)
# block <-> block distances
annot(blocks["red"], blocks["green"],  f"{dist(blocks['red'], blocks['green']):.3f} m", dx=-0.10)
annot(blocks["green"], blocks["blue"], f"{dist(blocks['green'], blocks['blue']):.3f} m", dx=0.10)
annot(blocks["red"], blocks["blue"],   f"{dist(blocks['red'], blocks['blue']):.3f} m", dx=-0.07, dy=-0.04, color="#555555")
# base separation
annot(BASE_L, BASE_R, f"{dist(BASE_L, BASE_R):.3f} m", dx=0.05, color="#555555")

ax.set_xlim(-1.35, 1.15)
ax.set_ylim(-0.65, 0.75)
ax.set_aspect("equal")
ax.set_xlabel("x (m) — 朝桌前方为正（相机在 -x 侧）")
ax.set_ylabel("y (m) — 朝相机左侧为正（+y 为左臂侧）")
ax.grid(True, alpha=0.25)
ax.set_title("Unity MuJoCo 场景俯视图（libero_pick_up_red_block.xml，世界坐标）", fontsize=13)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "env_topdown.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print("saved:", out)
