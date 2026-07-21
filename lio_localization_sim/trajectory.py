"""main.md 6-A-1 軽量2D合成データシミュレータ用の、既知の真値軌跡。

閉形式（解析的な微分）で位置・速度・角速度・加速度を計算する。数値微分を
使わないことで、微小時間刻みでの誤差増幅を避けている。
"""

import math
from dataclasses import dataclass


@dataclass
class State:
    t: float
    x: float
    y: float
    yaw: float
    vx: float  # world frame
    vy: float  # world frame
    omega: float  # body angular velocity (yaw rate)
    ax_body: float  # body frame linear acceleration (重力補正済み)
    ay_body: float
    az_body: float


class Trajectory:
    """Lissajous状の並進経路 + スケジュールされた高角速度スピンバーストの合成軌跡。

    並進方向とは独立にヨーがスピンする場面を作れるよう、オムニホイール機体を
    想定している（robomas_omuni等、実ワークスペースの既存パッケージに合わせた）。
    """

    GRAVITY = 9.81
    MAX_OMEGA = 2.0 * math.pi  # main.md C項: 最大角速度 2π rad/s

    def __init__(self, field_width: float, field_height: float, margin: float = 1.0,
                 pattern: int = 0):
        """patternで決定的な軌道バリエーションを選ぶ(乱数は使わない)。

        0: 既定(従来と同一)。ゆるいリサージュ + スピン5回/180秒
        1: 高速並進。並進周波数1.6倍(最大速度・加速度も1.6倍)
        2: スピン多発。60秒間にスピンバースト4回(高G旋回の連続ストレス)
        3: 壁際周回。マージンを狭めて壁近傍を大きく周回(視野が壁に偏り
           やすく、ポールが死角/遠距離になる時間が長い)
        """
        self.ax_amp = max(field_width / 2.0 - margin, 0.5)
        self.ay_amp = max(field_height / 2.0 - margin, 0.5)
        self.wx = 0.5   # rad/s
        self.wy = 0.5 * 1.3
        # 緩やかな基礎方位変化（並進方向とは独立、オムニホイール機体の想定に対応）。
        self.base_yaw_amp = math.pi / 6.0
        self.base_yaw_freq = 0.15  # rad/s

        # (開始時刻, 継続時間) のスピンバースト。main.mdの「高G旋回」評価に対応。
        self.spin_bursts = [
            (15.0, 2.0),
            (45.0, 3.0),
            (90.0, 2.0),
            (130.0, 4.0),
            (160.0, 2.0),
        ]

        if pattern == 1:
            self.wx = 0.8
            self.wy = 0.8 * 1.3
        elif pattern == 2:
            self.spin_bursts = [
                (10.0, 3.0),
                (22.0, 3.0),
                (35.0, 4.0),
                (50.0, 3.0),
            ]
        elif pattern == 3:
            self.ax_amp = max(field_width / 2.0 - 0.6, 0.5)
            self.ay_amp = max(field_height / 2.0 - 0.6, 0.5)
            self.wx = 0.35
            self.wy = 0.7

        # ボール横切りシナリオ（main.md 6章-4 評価テストに対応）
        self.ball_active_window = (40.0, 46.0)
        self.ball_start = (-self.ax_amp, 0.0)
        self.ball_velocity = (2.0, 0.3)  # m/s
        self.ball_radius = 0.11

    def _spin_offset_and_rate(self, t: float):
        offset = 0.0
        rate = 0.0
        for (t0, dur) in self.spin_bursts:
            if t <= t0:
                continue
            elif t >= t0 + dur:
                offset += self.MAX_OMEGA * dur
            else:
                offset += self.MAX_OMEGA * (t - t0)
                rate = self.MAX_OMEGA
        return offset, rate

    # 起動ホールド: 実際のロボコンではロボットは静止状態でシステムを起動し、
    # 自己位置推定が確立してから試合開始で動き出す。t=0から即座に動く軌道だと
    # ノード起動シーケンス(1〜2秒)の間に機体が移動してICPが最初のロックを
    # 確立できず、現実には起きない「初期永久迷子」を作ってしまう(高速並進
    # パターンの検証で確認)。全パターン共通で開始3秒間は完全静止させる。
    STARTUP_HOLD_SEC = 3.0

    def state(self, t: float) -> State:
        held = t < self.STARTUP_HOLD_SEC
        t = max(t - self.STARTUP_HOLD_SEC, 0.0)
        wx, wy = self.wx, self.wy
        # main.md想定(ロボコンは試合開始時にロボットが静止している)に合わせ、
        # x=-A cos(w t) の形で軌道を組む。この形はt=0でv=0(sin(0)=0)を
        # 厳密に満たしつつ、範囲は元のsin版と同じ[-A, A]に収まる
        # (振り子を端で放したのと同じ形: 端で静止 -> 中心へ加速)。
        # 静止状態からスタートする③の速度事前分布(V0=0固定)と食い違わない。
        # wx!=wyなので位相をずらさなくても2次元的なリサージュ図形になる。
        x = -self.ax_amp * math.cos(wx * t)
        y = -self.ay_amp * math.cos(wy * t)
        vx = self.ax_amp * wx * math.sin(wx * t)
        vy = self.ay_amp * wy * math.sin(wy * t)
        ax_w = self.ax_amp * wx * wx * math.cos(wx * t)
        ay_w = self.ay_amp * wy * wy * math.cos(wy * t)

        # ヨーは並進速度方向(atan2(vy,vx))には追従させない。オムニホイール機体は
        # 進行方向と機体の向きが独立なうえ、速度ベクトルが原点を通過して反転する
        # 瞬間(vx=vy=0)には「進行方向」という定義自体がπだけ不連続に反転して
        # しまう(fableの敵対的レビューで発見、数値検証でt≈62.83sに実際にヨーが
        # 1ms未満でπ跳躍することを確認した)。そのため基礎ヨーは並進運動から
        # 完全に切り離した、独立の滑らかな関数として定義する。
        # (1-cos)形にしてt=0でomega_base=0(静止スタート)にしている。
        yaw_base = self.base_yaw_amp * (1.0 - math.cos(self.base_yaw_freq * t))
        omega_base = self.base_yaw_amp * self.base_yaw_freq * math.sin(self.base_yaw_freq * t)

        spin_offset, spin_rate = self._spin_offset_and_rate(t)
        yaw = yaw_base + spin_offset
        omega = omega_base + spin_rate

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ax_body = cos_yaw * ax_w + sin_yaw * ay_w
        ay_body = -sin_yaw * ax_w + cos_yaw * ay_w

        if held:
            # ホールド中は完全静止(位置・ヨーはt'=0の値、速度・加速度・角速度ゼロ)
            return State(
                t=t, x=x, y=y, yaw=yaw, vx=0.0, vy=0.0, omega=0.0,
                ax_body=0.0, ay_body=0.0, az_body=self.GRAVITY,
            )
        return State(
            t=t, x=x, y=y, yaw=yaw, vx=vx, vy=vy, omega=omega,
            ax_body=ax_body, ay_body=ay_body, az_body=self.GRAVITY,
        )

    def ball_position(self, t: float):
        """アクティブウィンドウ内ならボールの(x, y)を返し、それ以外はNone。"""
        t0, t1 = self.ball_active_window
        if not (t0 <= t <= t1):
            return None
        dt = t - t0
        bx = self.ball_start[0] + self.ball_velocity[0] * dt
        by = self.ball_start[1] + self.ball_velocity[1] * dt
        return (bx, by)
