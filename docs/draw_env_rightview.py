# -*- coding: utf-8 -*-
"""Draw right-side view (x-z plane, viewed from -y / R-arm side) of the scene.

Arm home-pose link positions are computed with MuJoCo FK (same XML as Unity).
Camera pitch angle is labeled: camera at agentview_site (-0.85, 0, 0.40),
look target = origin -> pitch = atan2(0.40, 0.85) below horizontal.
"""
import math
import os
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Arial"]
plt.rcParams["axes.unicode_minus"] = False

XML = r"C:\vla\my\libero-unity\Assets\LIBERO\assets\robots\so100_mjcf\libero_pick_up_red_block.xml"
CAM_POS = np.array([-0.85, 0.0, 0.40])   # agentview_site (Unity 导入后生效)
LOOK = np.array([0.0, 0.0, 0.0])          # 目视目标（原点，不随桌移动）
TABLE_CX, TABLE_HALF_X, TABLE_Z, TABLE_TH = 0.10, 0.60, 0.0, 0.05   # 桌面 x∈[-0.50,0.70], z∈[-0.05,0]
LEG_X = np.array([TABLE_CX - 0.50, TABLE_CX + 0.50])               # 桌腿 x（相对桌心 ±0.5）
LEG_BOTTOM = -0.875
BLOCKS = {"red": (-0.25, -0.30), "green": (-0.35, 0.05), "blue": (-0.30, 0.30)}
BLOCK_HALF = 0.025

# ---- MuJoCo FK: R 臂 home 姿态的链路位置（x, z 投影；L 臂在 y=+0.20 处，投影重合）----
m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
home = {"Rotation": 0.0, "Pitch": -3.14, "Elbow": 3.14, "Wrist_Pitch": 0.0,
        "Wrist_Roll": -1.57, "Jaw": 0.04}
for arm in ("R_", "L_"):
    for jt, val in home.items():
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, arm + jt)
        d.qpos[m.jnt_qposadr[jid]] = val
mujoco.mj_forward(m, d)
chain = []
for bname in ("R_Base", "R_Rotation_Pitch", "R_Upper_Arm", "R_Lower_Arm",
              "R_Wrist_Pitch_Roll", "R_Fixed_Jaw"):
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, bname)
    chain.append(d.xpos[bid].copy())
sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "R_eef_site")
chain.append(d.site_xpos[sid].copy())
chain = np.array(chain)   # (7, 3)

fig, ax = plt.subplots(figsize=(11, 8.5))

# 地面与桌台
ax.axhline(0, color="0.55", lw=1.2)
ax.text(-1.30, 0.0, "地面 z=0", ha="right", va="bottom", fontsize=9, color="0.4")
ax.add_patch(mpatches.Rectangle((TABLE_CX - TABLE_HALF_X, -TABLE_TH), 2 * TABLE_HALF_X, TABLE_TH,
                                facecolor="#E8D5B0", edgecolor="#8B6B3F", lw=2))
ax.text(0.75, -0.03, f"桌面 z=0（中心 x={TABLE_CX:+.2f}）", ha="left", va="center", fontsize=10, color="#5C4423")
for lx in LEG_X:
    ax.add_patch(mpatches.Rectangle((lx - 0.025, LEG_BOTTOM), 0.05, -LEG_BOTTOM,
                                    facecolor="0.6", edgecolor="0.45", lw=1))
ax.text(LEG_X[0], LEG_BOTTOM - 0.06, f"桌腿 x={LEG_X[0]:+.2f} 至 z={LEG_BOTTOM}", ha="center", fontsize=8.5, color="0.4")

# 基座立柱 + 机械臂 home 姿态
ax.add_patch(mpatches.Rectangle((-0.72, -0.95), 0.04, 0.875, facecolor="0.55", edgecolor="0.4"))
ax.plot([chain[0, 0], chain[0, 0]], [chain[0, 2], chain[0, 2] + 0.06], color="0.25", lw=3)
for i in range(1, len(chain)):
    ax.plot([chain[i-1, 0], chain[i, 0]], [chain[i-1, 2], chain[i, 2]], color="#1F4E79", lw=5, solid_capstyle="round")
ax.plot(chain[:, 0], chain[:, 2], "o", color="#1F4E79", ms=5)
ax.annotate("R 臂 home（折叠）姿态\n（关节角 Rotation=0 Pitch=-3.14 Elbow=3.14 …）",
            xy=(chain[-1, 0], chain[-1, 2]), xytext=(-0.62, 0.52),
            fontsize=9, color="#1F4E79",
            arrowprops=dict(arrowstyle="->", color="#1F4E79", lw=1))
ax.text(-0.90, -0.36, "R/L 基座 z=-0.075", ha="right", va="center", fontsize=9, color="0.3")

# 可达弧（以基座为心，半径 0.46，指向桌面方向）
theta = np.linspace(-15, 75, 100) * np.pi / 180
bx, bz = -0.70, -0.075
ax.plot(bx + 0.46 * np.cos(theta), bz + 0.46 * np.sin(theta), "--", color="0.45", lw=1.2)
ax.text(bx + 0.46 * np.cos(0.55) + 0.02, bz + 0.46 * np.sin(0.55), "可达半径 0.46 m",
        fontsize=8.5, color="0.4", rotation=38)

# 方块（x-z 投影，y 分量各不相同但投影重叠）
for name, (bx_, by_) in BLOCKS.items():
    c = {"red": "#D93025", "green": "#1E8E3E", "blue": "#1A73E8"}[name]
    ax.add_patch(mpatches.Rectangle((bx_ - BLOCK_HALF, 0.005), 0.05, 0.05,
                                    facecolor=c, edgecolor="black", lw=1.2))
    ax.text(bx_, 0.10, f"{name}\nx={bx_:+.2f} y={by_:+.2f}", ha="center", va="bottom",
            fontsize=8.5, color="black")

# 相机 + 目视线 + 俯仰角
ax.plot(*CAM_POS[[0, 2]], "^", color="#E37400", ms=16, zorder=6)
ax.text(CAM_POS[0] - 0.02, CAM_POS[2] + 0.04, "agentview 相机\n(-0.85, 0, 0.40)",
        ha="center", va="bottom", fontsize=9.5, color="#E37400")
ax.plot([CAM_POS[0], LOOK[0]], [CAM_POS[2], LOOK[2]], "-.", color="#E37400", lw=1.6)
# 水平参考线 + 俯仰角弧
ax.plot([CAM_POS[0], LOOK[0]], [CAM_POS[2], CAM_POS[2]], ":", color="0.5", lw=1)
ax.text(-0.43, 0.415, f"水平距离 0.85 m", ha="center", va="bottom", fontsize=8.5, color="0.45")
pitch = math.degrees(math.atan2(CAM_POS[2] - LOOK[2], LOOK[0] - CAM_POS[0]))
arc = np.linspace(0, math.radians(pitch), 60)
r_arc = 0.13
ax.plot(CAM_POS[0] + r_arc * np.cos(arc), CAM_POS[2] - r_arc * np.sin(arc), color="#E37400", lw=1.5)
ax.text(CAM_POS[0] + 0.20, CAM_POS[2] - 0.05, f"俯仰角 ≈ {pitch:.1f}°（向下）",
        fontsize=11, color="#E37400", fontweight="bold")

ax.set_xlim(-1.35, 1.15)
ax.set_ylim(-1.05, 0.80)
ax.set_aspect("equal")
ax.grid(True, alpha=0.2)
ax.set_xlabel("x (m) — 相机在 -x 侧，朝 +x 看")
ax.set_ylabel("z (m) — 向上为正")
ax.set_title("Unity MuJoCo 场景右视图（x-z 平面，从 R 臂侧 -y 方向观看）", fontsize=13)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "env_rightview.png")
fig.savefig(out, dpi=150, bbox_inches="tight")
print(f"相机俯仰角: {pitch:.1f}°（相对水平，向下）")
print("saved:", out)
