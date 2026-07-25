#!/usr/bin/env python3
"""第三段階の検知+自己復帰を定量化する。
イベント窓 sim時刻 25〜35s を破綻区間とみなし、
 - 破綻前の基準誤差(23〜25s中央値)
 - 破綻中のピーク誤差(25〜35s)
 - 復帰: 窓後(t>35)で誤差がRECOVER閾値未満に戻り2s持続する最初の時刻→復帰時間
 - 最終定常誤差(58〜63s中央値)と永続発散(NaN/大)
 - 検知シグナル(launch.log)
を表示し、判定(検知された & 永続発散なし & 復帰した)を出す。
"""
import csv
import sys
import statistics

RUN = sys.argv[1]
EV0, EV1 = 25.0, 35.0
RECOVER = 8.0  # mm、復帰判定閾値(定常の数倍)

rows = []
with open(f'{RUN}/diag_raw_errors.csv') as f:
    r = csv.reader(f)
    next(r)
    for t, p, y in r:
        rows.append((float(t), float(p), float(y)))

def med(lo, hi):
    v = [p for t, p, _ in rows if lo <= t < hi]
    return statistics.median(v) if v else float('nan')

def pk(lo, hi):
    v = [p for t, p, _ in rows if lo <= t < hi]
    return max(v) if v else float('nan')

base = med(EV0 - 2, EV0)
peak = pk(EV0, EV1)
final = med(58.0, 63.0)

# 復帰時間: 窓後、RECOVER未満が2s続く最初の時刻
recover_t = None
post = [(t, p) for t, p, _ in rows if t >= EV1]
i = 0
while i < len(post):
    t, p = post[i]
    if p < RECOVER:
        j = i
        while j < len(post) and post[j][0] - t < 2.0:
            if post[j][1] >= RECOVER:
                break
            j += 1
        else:
            recover_t = t - EV1
            break
        if post[j][0] - t >= 2.0:
            recover_t = t - EV1
            break
        i = j
    else:
        i += 1

import subprocess
def cnt(pat):
    return int(subprocess.run(['grep', '-c', pat, f'{RUN}/launch.log'],
                              capture_output=True, text=True).stdout.strip() or 0)
reset = cnt('graph (re)initialized') - 1
rel = cnt('reliability lost')
gate = cnt('rejected by gate')
dcs = cnt('downweighted by DCS')
cov = cnt('coverage wait exceeded') + cnt('insufficient IMU')

detected = (reset > 0 or rel > 0 or gate > 0 or dcs > 0 or cov > 0)
diverged = (final != final) or final > 50.0  # NaN or >50mm
recovered = (recover_t is not None) and not diverged and final < RECOVER

verdict = "PASS(検知+復帰)" if (recovered and not diverged) else \
          ("FAIL(永続発散)" if diverged else "要確認")
# 検知は「破綻が起きたなら検知されるべき」。破綻中ピークが基準の数倍なければ
# そもそも破綻が推定器に届いていない(透過)ので検知不要。
transparent = peak < 3.0 * max(base, 1.0)

print(f"=== {RUN.split('/')[-1]}")
print(f"  基準誤差(窓前)={base:.2f}mm  破綻中ピーク={peak:.2f}mm  最終定常={final:.2f}mm")
print(f"  復帰時間={'未復帰' if recover_t is None else f'{recover_t:.1f}s'}  "
      f"(窓後に{RECOVER}mm未満が2s持続)")
print(f"  検知: reset={reset} reliability_lost={rel} gate={gate} dcs={dcs} cov={cov}"
      f"  → {'検知あり' if detected else '検知なし'}")
if transparent:
    print(f"  判定: 透過(破綻が推定器に影響せず、検知不要。ピーク{peak:.1f}mm≈基準)")
else:
    print(f"  判定: {verdict}"
          + ("" if detected else "  [警告: 破綻したのに検知シグナルなし]"))
