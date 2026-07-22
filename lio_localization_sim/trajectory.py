"""main.md 6-A-1 軽量2D合成データシミュレータ用の、既知の真値軌跡。

閉形式（解析的な微分）で位置・速度・角速度・加速度を計算する。数値微分を
使わないことで、微小時間刻みでの誤差増幅を避けている。
"""

import math
import random
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
    MAX_VEL = 5.0              # main.md C項: 最大移動速度 5.0 m/s
    MAX_ACC = 4.905           # main.md C項: 最大加速度 0.5G
    # 最大角加速度: main.md未規定のため仮定値(0.5秒で最大角速度2πに到達する
    # 4π rad/s²)。ランダム経路の角運動をこの範囲内に構成的に収める。
    MAX_ALPHA = 4.0 * math.pi
    # 機体の外接半径。500mm等辺三角柱(角を半径50mmでフィレット)の重心から
    # 最遠点までの距離 = 辺長/√3 = 0.5/√3 ≈ 0.289m(フィレットは外接円を縮める
    # 方向なので安全側の上界)。オムニで自由回転するため、中心がこの半径以上
    # 壁・円柱から離れていれば機体は非衝突。
    ROBOT_RADIUS = 0.5 / math.sqrt(3.0)

    def __init__(self, field_width: float, field_height: float, margin: float = 1.0,
                 pattern: int = 0):
        """patternで決定的な軌道バリエーションを選ぶ(乱数は使わない)。

        0: 既定(従来と同一)。ゆるいリサージュ + スピン5回/180秒
        1: 高速並進。並進周波数1.6倍(最大速度・加速度も1.6倍)
        2: スピン多発。60秒間にスピンバースト4回(高G旋回の連続ストレス)
        3: 壁際周回。マージンを狭めて壁近傍を大きく周回(視野が壁に偏り
           やすく、ポールが死角/遠距離になる時間が長い)
        pattern>=1000: ランダム経路(seed=pattern)。第二段階の耐久試験用。
           帯域制限ランダム(ランダム位相サインの重ね合わせ)+平滑立ち上がり
           エンベロープで、運動力学的に妥当な「めちゃくちゃな」経路を生成する。
           生成後に全区間を密サンプリングして実ピークを計測し、振幅を単一
           スケールで縮めることで 速度≤5.0 / 加速度≤0.5G / 角速度≤2π /
           角加速度≤4π / 壁非衝突(中心が壁からmargin以上) を構成的に保証する
           (壁への食い込み・運動力学違反は起こさない。円柱衝突回避は経路が
           通らない場所にのみ円柱を置くランナー側で担保)。
        """
        if pattern >= 2000:
            self._init_stage3(field_width, field_height, pattern)
            return
        if pattern >= 1000:
            self._init_random(field_width, field_height, seed=pattern)
            return
        self.random_mode = False
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

    # ----- ランダム経路モード(第二段階耐久試験) -----
    RANDOM_MARGIN = 0.5   # 壁マージン: ROBOT_RADIUS(0.289) + クリアランス
    RANDOM_TAU = 3.0      # 静止からの平滑立ち上がり時間(s)
    RANDOM_N_HARM = 4     # 軸あたりの調和成分数
    RANDOM_SAFETY = 0.97  # 各限界に対する安全係数(境界での等号を避ける)

    def _init_random(self, field_width: float, field_height: float, seed: int):
        self.random_mode = True
        self.stage3 = None
        rng = random.Random(seed)
        # 従来モードの属性(evaluator/plotが参照)を無効値で用意しておく
        self.spin_bursts = []
        self.ball_active_window = (-1.0, -1.0)  # 常に非アクティブ
        self.ball_start = (0.0, 0.0)
        self.ball_velocity = (0.0, 0.0)
        self.ball_radius = 0.11

        def draw_harmonics(n, w_lo, w_hi, a_lo, a_hi):
            return [(rng.uniform(a_lo, a_hi), rng.uniform(w_lo, w_hi),
                     rng.uniform(0.0, 2.0 * math.pi)) for _ in range(n)]

        # 並進: 特性角周波数を v/L~1rad/s 帯に置くと速度・加速度・振幅の各限界が
        # 近い所で拮抗し、限界いっぱいの激しい経路になりやすい。
        self._hx = draw_harmonics(self.RANDOM_N_HARM, 0.3, 1.6, 0.4, 1.0)
        self._hy = draw_harmonics(self.RANDOM_N_HARM, 0.3, 1.6, 0.4, 1.0)
        # ヨー(オムニ機体、並進と独立): より速い帯域で自由に回す
        self._hyaw = draw_harmonics(self.RANDOM_N_HARM, 0.4, 5.0, 0.3, 1.5)

        # 使用可能な半幅(中心が壁からmargin以上離れる範囲)
        usable_x = max(field_width / 2.0 - self.RANDOM_MARGIN, 0.3)
        usable_y = max(field_height / 2.0 - self.RANDOM_MARGIN, 0.3)

        # 全区間を密サンプリングして実ピークを計測 → 振幅を単一スケールで縮める。
        # サンプル窓は経路が準周期的なので十分長く取れば最大値を捉えられる。
        ts = [i * 0.05 for i in range(int(200.0 / 0.05))]
        max_exc_x = max_exc_y = 1e-9
        max_v = max_a = 1e-9
        for t in ts:
            e, ed, edd = self._envelope(t)
            sx, sxd, sxdd = self._sum_sines(self._hx, t)
            sy, syd, sydd = self._sum_sines(self._hy, t)
            ex, ey = abs(e * sx), abs(e * sy)
            vx = ed * sx + e * sxd
            vy = ed * sy + e * syd
            ax = edd * sx + 2.0 * ed * sxd + e * sxdd
            ay = edd * sy + 2.0 * ed * syd + e * sydd
            max_exc_x = max(max_exc_x, ex)
            max_exc_y = max(max_exc_y, ey)
            max_v = max(max_v, math.hypot(vx, vy))
            max_a = max(max_a, math.hypot(ax, ay))
        s = self.RANDOM_SAFETY * min(
            usable_x / max_exc_x, usable_y / max_exc_y,
            self.MAX_VEL / max_v, self.MAX_ACC / max_a)
        self._hx = [(a * s, w, p) for (a, w, p) in self._hx]
        self._hy = [(a * s, w, p) for (a, w, p) in self._hy]

        # ヨー: 角速度・角加速度の限界に合わせて別途スケール
        max_w = max_al = 1e-9
        for t in ts:
            e, ed, edd = self._envelope(t)
            sw, swd, swdd = self._sum_sines(self._hyaw, t)
            omega = ed * sw + e * swd
            alpha = edd * sw + 2.0 * ed * swd + e * swdd
            max_w = max(max_w, abs(omega))
            max_al = max(max_al, abs(alpha))
        sy_ = self.RANDOM_SAFETY * min(
            self.MAX_OMEGA / max_w, self.MAX_ALPHA / max_al)
        self._hyaw = [(a * sy_, w, p) for (a, w, p) in self._hyaw]

        # 検証用に確定ピークを記録(ランナーが妥当性の証跡としてログする)
        self.peak_v = max_v * s
        self.peak_a = max_a * s
        self.peak_omega = max_w * sy_
        self.peak_alpha = max_al * sy_
        self.peak_exc_x = max_exc_x * s
        self.peak_exc_y = max_exc_y * s

    def _envelope(self, t: float):
        """静止(t=0でv=a=0)から平滑に走り出す C² エンベロープ e, e', e''。"""
        tau = self.RANDOM_TAU
        if t >= tau:
            return 1.0, 0.0, 0.0
        u = t / tau
        # smootherstep: 6u^5-15u^4+10u^3(端点で値0/1、1階・2階微分ゼロ)
        e = u * u * u * (u * (u * 6.0 - 15.0) + 10.0)
        ed = (30.0 * u * u * (u * u - 2.0 * u + 1.0)) / tau
        edd = (60.0 * u * (2.0 * u * u - 3.0 * u + 1.0)) / (tau * tau)
        return e, ed, edd

    @staticmethod
    def _sum_sines(harmonics, t: float):
        """Σ A sin(w t + φ) と その1階・2階時間微分を返す。"""
        s = sd = sdd = 0.0
        for (a, w, p) in harmonics:
            ang = w * t + p
            sn = math.sin(ang)
            cs = math.cos(ang)
            s += a * sn
            sd += a * w * cs
            sdd += -a * w * w * sn
        return s, sd, sdd

    def _base_kinematics(self, t: float):
        """内部時刻tでの基底運動(位置・速度・世界系加速度・ヨー・角速度)を返す。
        戻り値: (x, y, yaw, vx, vy, omega, ax_w, ay_w)。stage3のワープ/オーバー
        シュートはこれを土台に合成する。"""
        e, ed, edd = self._envelope(t)
        sx, sxd, sxdd = self._sum_sines(self._hx, t)
        sy, syd, sydd = self._sum_sines(self._hy, t)
        syaw, syawd, _ = self._sum_sines(self._hyaw, t)
        x = e * sx
        y = e * sy
        yaw = e * syaw
        vx = ed * sx + e * sxd
        vy = ed * sy + e * syd
        omega = ed * syaw + e * syawd
        ax_w = edd * sx + 2.0 * ed * sxd + e * sxdd
        ay_w = edd * sy + 2.0 * ed * syd + e * sydd
        return x, y, yaw, vx, vy, omega, ax_w, ay_w

    @staticmethod
    def _assemble(t, held, x, y, yaw, vx, vy, omega, ax_w, ay_w, gravity):
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        ax_body = cos_yaw * ax_w + sin_yaw * ay_w
        ay_body = -sin_yaw * ax_w + cos_yaw * ay_w
        if held:
            return State(t=t, x=x, y=y, yaw=yaw, vx=0.0, vy=0.0, omega=0.0,
                         ax_body=0.0, ay_body=0.0, az_body=gravity)
        return State(t=t, x=x, y=y, yaw=yaw, vx=vx, vy=vy, omega=omega,
                     ax_body=ax_body, ay_body=ay_body, az_body=gravity)

    def _state_random(self, t: float, held: bool) -> 'State':
        # stage3(pattern>=2000): 妥当な基底経路に破綻イベントを合成する
        if getattr(self, 'stage3', None) == 'overlimit':
            return self._state_overlimit(t, held)
        if getattr(self, 'stage3', None) == 'collision':
            return self._state_collision(t, held)
        x, y, yaw, vx, vy, omega, ax_w, ay_w = self._base_kinematics(t)
        return self._assemble(t, held, x, y, yaw, vx, vy, omega, ax_w, ay_w,
                              self.GRAVITY)

    # ----- 第三段階: 破綻シナリオ(運動力学制約外・検知/自己復帰の検証) -----
    def _init_stage3(self, field_width, field_height, pattern):
        # pattern = 2000 + kind*100 + seed。kind: 0=限界突破, 1=壁衝突
        kind = (pattern - 2000) // 100
        seed = (pattern - 2000) % 100
        # 基底は妥当なランダム経路(seedで決定的)。これを土台に破綻を合成する。
        self._init_random(field_width, field_height, seed=3000 + seed)
        self._field_w = field_width
        self._field_h = field_height
        # 破綻イベントの窓(内部時刻。起動ホールド後の走行時間で指定)
        self.event_t0 = 22.0
        self.event_dur = 10.0
        if kind == 0:
            self.stage3 = 'overlimit'
            # 時間ワープの倍率: dτ/dt を最大 boost 倍にして速度を限界超へ
            # (速度∝boost、加速度∝boost²、角速度∝boost、角加速度∝boost²)
            self.boost = 2.6  # peak_v≈2.6倍→約11m/s(限界5.0超), 加速度は約6.8倍
        elif kind == 1:
            self.stage3 = 'collision'
            # 振幅を(1+overshoot)倍に膨らませ、壁を越えて食い込ませてから戻す
            self.overshoot = 0.7

    def _warp(self, t):
        """時間ワープ τ(t), dτ/dt, d²τ/dt²。窓の間だけ dτ/dt を boost 倍にする
        (raised-cosine バンプで C¹ 連続)。窓外は τ=t+一定オフセット(妥当継続)。"""
        t0, W, boost = self.event_t0, self.event_dur, self.boost
        if t <= t0:
            return t, 1.0, 0.0
        s = t - t0
        if s >= W:
            # 窓通過後は一定オフセット(sin(2π)=0)を足して妥当に継続
            return t + (boost - 1.0) * 0.5 * W, 1.0, 0.0
        two_pi = 2.0 * math.pi
        b = 0.5 * (1.0 - math.cos(two_pi * s / W))         # バンプ 0→1→0
        bd = (math.pi / W) * math.sin(two_pi * s / W)       # b'
        # τ = t + (boost-1)∫b, ∫b = 0.5(s - (W/2π)sin(2π s/W))
        integ = 0.5 * (s - (W / two_pi) * math.sin(two_pi * s / W))
        tau = t + (boost - 1.0) * integ
        dtau = 1.0 + (boost - 1.0) * b
        ddtau = (boost - 1.0) * bd
        return tau, dtau, ddtau

    def _state_overlimit(self, t: float, held: bool) -> 'State':
        tau, dtau, ddtau = self._warp(t)
        x, y, yaw, vx, vy, omega, ax_w, ay_w = self._base_kinematics(tau)
        # 連鎖律: 実時刻の速度=基底速度×dτ、加速度=基底加速度×dτ²+基底速度×d²τ
        vxr = vx * dtau
        vyr = vy * dtau
        omr = omega * dtau
        axr = ax_w * dtau * dtau + vx * ddtau
        ayr = ay_w * dtau * dtau + vy * ddtau
        return self._assemble(t, held, x, y, yaw, vxr, vyr, omr, axr, ayr,
                              self.GRAVITY)

    def _collision_bump(self, t):
        """振幅膨張バンプ f(t), f'(t), f''(t)。窓の間だけ (1+overshoot) 倍。"""
        t0, W, ov = self.event_t0, self.event_dur, self.overshoot
        if t <= t0 or t >= t0 + W:
            return 1.0, 0.0, 0.0
        s = t - t0
        two_pi = 2.0 * math.pi
        b = 0.5 * (1.0 - math.cos(two_pi * s / W))
        bd = (math.pi / W) * math.sin(two_pi * s / W)
        bdd = (two_pi * math.pi / (W * W)) * math.cos(two_pi * s / W)
        return 1.0 + ov * b, ov * bd, ov * bdd

    def _state_collision(self, t: float, held: bool) -> 'State':
        x0, y0, yaw, vx0, vy0, omega, ax0, ay0 = self._base_kinematics(t)
        f, fd, fdd = self._collision_bump(t)
        # 位置=基底×f。速度・加速度は積の微分(位置だけ膨張、ヨーは不変)
        x = x0 * f
        y = y0 * f
        vx = vx0 * f + x0 * fd
        vy = vy0 * f + y0 * fd
        ax = ax0 * f + 2.0 * vx0 * fd + x0 * fdd
        ay = ay0 * f + 2.0 * vy0 * fd + y0 * fdd
        return self._assemble(t, held, x, y, yaw, vx, vy, omega, ax, ay,
                              self.GRAVITY)

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
        if self.random_mode:
            return self._state_random(t, held)
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
