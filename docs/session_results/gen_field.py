#!/usr/bin/env python3
"""第二段階耐久試験: seedごとに 10x10以内のランダムフィールド + 経路非衝突の
円柱配置を決定的に生成し、field_config yamlを書き出す。

フィールド寸法・トラジェクトリはseedから決定的に作られ、sim全ノードが同一の
ものを再現する(トラジェクトリはpattern=seedで、寸法は本スクリプトが計算して
launch引数で渡す)。円柱はランダム経路が通らない場所にのみ置き、機体
(外接半径ROBOT_RADIUS)との衝突を起こさないことを保証する。
"""
import math
import random
import sys

sys.path.insert(0, '/home/yukichi6105/ros2_ws/src/lio_localization_sim')
from lio_localization_sim.trajectory import Trajectory

R_ROBOT = Trajectory.ROBOT_RADIUS  # 0.289m
CYL_R = 0.1
CLEAR = 0.15  # 円柱衝突回避の追加安全余裕


def field_dims(seed):
    rng = random.Random(seed * 7919)
    w = rng.uniform(7.0, 10.0)
    h = rng.uniform(7.0, 10.0)
    return w, h


def gen(seed, out_path):
    w, h = field_dims(seed)
    traj = Trajectory(w, h, pattern=seed)
    # 経路を密サンプリング(起動ホールド3s + 60s走行 + 余裕)
    path = [(traj.state(t).x, traj.state(t).y) for t in
            [i * 0.05 for i in range(int(66.0 / 0.05))]]

    rng = random.Random(seed * 104729 + 1)
    # 円柱は0〜2個。0個(壁のみ=対称性リスクシナリオ)は空リストがROS2 yamlで
    # 型推論不能なため、フィールド外(LiDAR range_max=30m外)に番兵円柱を1本置く。
    # これはLiDARに一度も映らずBearingRangeFactorも一切張られないため、実質
    # 「円柱なし・壁マッチングのみで絶対基準を維持」を再現する。
    n_cyl = rng.choice([0, 1, 1, 2, 2])
    if n_cyl == 0:
        with open(out_path, 'w') as f:
            f.write("/**:\n  ros__parameters:\n")
            f.write(f"    field_width: {w:.4f}\n")
            f.write(f"    field_height: {h:.4f}\n")
            f.write("    cylinder_ids: [0]\n")
            f.write("    cylinder_x: [1000.0]\n    cylinder_y: [1000.0]\n")
            f.write("    cylinder_radius: [0.1]\n")
            f.write("    simulate_ball: false\n")
            f.write("    initial_x: 0.0\n    initial_y: 0.0\n    initial_theta: 0.0\n")
        print(f"seed={seed} field={w:.2f}x{h:.2f} cyl=0(壁のみ) "
              f"peak_v={traj.peak_v:.2f} peak_a={traj.peak_a:.2f} "
              f"peak_w={traj.peak_omega:.2f} peak_al={traj.peak_alpha:.2f} "
              f"exc=({traj.peak_exc_x:.2f},{traj.peak_exc_y:.2f}) "
              f"wall_lim=({w/2-R_ROBOT:.2f},{h/2-R_ROBOT:.2f})")
        return w, h
    usable_x = w / 2.0 - (CYL_R + 0.1)
    usable_y = h / 2.0 - (CYL_R + 0.1)
    min_clear = R_ROBOT + CYL_R + CLEAR  # 経路点からこの距離以上離す
    cyls = []
    tries = 0
    while len(cyls) < n_cyl and tries < 4000:
        tries += 1
        cx = rng.uniform(-usable_x, usable_x)
        cy = rng.uniform(-usable_y, usable_y)
        # 経路非衝突
        if any((cx - px) ** 2 + (cy - py) ** 2 < min_clear ** 2 for (px, py) in path):
            continue
        # 円柱同士も離す
        if any((cx - ox) ** 2 + (cy - oy) ** 2 < (4 * CYL_R) ** 2 for (ox, oy) in cyls):
            continue
        cyls.append((cx, cy))

    ids = list(range(len(cyls)))
    xs = [round(c[0], 4) for c in cyls]
    ys = [round(c[1], 4) for c in cyls]
    rs = [CYL_R] * len(cyls)

    with open(out_path, 'w') as f:
        f.write("/**:\n  ros__parameters:\n")
        f.write(f"    field_width: {w:.4f}\n")
        f.write(f"    field_height: {h:.4f}\n")
        f.write(f"    cylinder_ids: {ids}\n")
        f.write(f"    cylinder_x: {xs}\n")
        f.write(f"    cylinder_y: {ys}\n")
        f.write(f"    cylinder_radius: {rs}\n")
        f.write("    simulate_ball: false\n")  # 第二段階は経路耐久に集中
        f.write("    initial_x: 0.0\n    initial_y: 0.0\n    initial_theta: 0.0\n")

    # 妥当性の証跡を標準出力へ(ランナーがログに残す)
    print(f"seed={seed} field={w:.2f}x{h:.2f} cyl={len(cyls)} "
          f"peak_v={traj.peak_v:.2f} peak_a={traj.peak_a:.2f} "
          f"peak_w={traj.peak_omega:.2f} peak_al={traj.peak_alpha:.2f} "
          f"exc=({traj.peak_exc_x:.2f},{traj.peak_exc_y:.2f}) "
          f"wall_lim=({w/2-R_ROBOT:.2f},{h/2-R_ROBOT:.2f})")
    return w, h


if __name__ == '__main__':
    seed = int(sys.argv[1])
    out = sys.argv[2]
    w, h = gen(seed, out)
    # ランナーがlaunch引数に使うため寸法も別ファイルへ
    with open(out + '.dims', 'w') as f:
        f.write(f"{w:.4f} {h:.4f}\n")
