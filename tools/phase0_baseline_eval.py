#!/usr/bin/env python3
"""Phase 0 baseline measurement report for the trajectory-tracking-controller upgrade.

Per docs/requirements/経路追従.md §19 "Phase 0：計測と回帰テスト", Phase 0's only job is to
measure and log the CURRENT (unmodified) mission-servo behaviour before any controller
changes are made in later phases. This script does not run anything -- it only parses
artifacts that already exist on disk:

  1. Unity's own per-lap result line, emitted by StartToBingoValidation.Complete() via
     Debug.Log, in the form:
       [StartToBingo RESULT lap=<int>] final=(<x>,<y>)m completion_time=<float>s
       target_error=<float>mm terminal_yaw_error=<float>deg tracking_rmse=<float>mm
       tracking_max=<float>mm max_accel=<float>mps2 wheel_odom_rmse=<float>mm
       wheel_odom_max=<float>mm pickup_error=<float>mm pickup_speed=<float>mm/s
       notes_captured=<bool> collisions=<comma-separated-or-empty>
     This line only exists inside Unity's console output. Today (2026-07-31) nothing in
     the stage4_mcp_run.sh workflow redirects that console to a file automatically -- the
     practical capture path is pulling it via the Unity_GetConsoleLogs MCP tool (or the
     Unity Editor.log at %LOCALAPPDATA%\\Unity\\Editor\\Editor.log on the Windows host) and
     saving it to a text file, whose path is this script's second argument. That gap is a
     real process hole worth fixing outside this script's scope.

  2. The consolidated/*.csv files already produced by consolidate_stage_log.py from a
     --record-stages rosbag2 (ground_truth.csv, odom_fast.csv, state_estimate.csv,
     scan_match_wall.csv, collisions.csv). This script does NOT touch rosbag2 or ROS
     message types at all -- it is pure stdlib post-processing over those CSVs, so it has
     no rosbag2_py/rclpy dependency even though consolidate_stage_log.py does.

Three distinct accuracy metrics appear in this report and must not be conflated:
  - tracking_rmse / tracking_max (from the Unity log line): actual-vs-commanded-reference
    tracking error, i.e. controller accuracy.
  - wheel_odom_rmse / wheel_odom_max (from the Unity log line): naive wheel dead-reckoning
    self-check internal to the Unity script, vs Unity ground truth.
  - state_estimate / odom_fast position error vs ground_truth (computed here from the
    consolidated CSVs): the actual self-localization (ROS estimator) error.

Usage:
    phase0_baseline_eval.py <run_dir> <console_log> [--field-json PATH] [--out PATH]

    <run_dir>      docs/stage4_runs/<run_tag>/ -- must already contain consolidated/
                   (produced beforehand by consolidate_stage_log.py; this script does not
                   invoke it).
    <console_log>  path to a text file containing captured Unity console output. May
                   contain zero or more "[StartToBingo RESULT ...]" lines.

Never fakes a result: every section that cannot be computed from what is actually on disk
is labelled "not computed" / "データなし" with the reason, rather than printing 0 or a
number derived from insufficient data silently.
"""
import argparse
import bisect
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path

RESULT_LINE_RE = re.compile(r'\[StartToBingo RESULT lap=(?P<lap>-?\d+)\]')
FINAL_RE = re.compile(r'final=\(\s*(?P<x>[-+0-9.eE]+)\s*,\s*(?P<y>[-+0-9.eE]+)\s*\)m')

# key -> unit suffix that is concatenated directly onto the numeric value with no space
# (e.g. "completion_time=6.530s"), per the exact Debug.Log format in Complete().
NUMERIC_FIELD_UNITS = {
    'completion_time': 's',
    'target_error': 'mm',
    'terminal_yaw_error': 'deg',
    'tracking_rmse': 'mm',
    'tracking_max': 'mm',
    'max_accel': 'mps2',
    'wheel_odom_rmse': 'mm',
    'wheel_odom_max': 'mm',
    'pickup_error': 'mm',
    'pickup_speed': 'mm/s',
}
REQUIRED_KEYS = ['lap', 'final_x', 'final_y'] + list(NUMERIC_FIELD_UNITS) + ['notes_captured']


def wrap(d):
    return (d + math.pi) % (2.0 * math.pi) - math.pi


def parse_result_line(line):
    """Parse one console line. Returns (record_dict, None) on success, or
    (None, reason_str) if the line contains a RESULT marker but could not be fully
    parsed (so the caller can count/report malformed lines instead of silently
    dropping them)."""
    m = RESULT_LINE_RE.search(line)
    if not m:
        return None, None  # not a RESULT line at all -- not an error
    rest = line[m.end():].rstrip('\r\n')

    fm = FINAL_RE.search(rest)
    if not fm:
        return None, 'final=(x,y)m field missing or malformed'

    rec = {'lap': int(m.group('lap'))}
    try:
        rec['final_x'] = float(fm.group('x'))
        rec['final_y'] = float(fm.group('y'))
    except ValueError:
        return None, 'final=(x,y)m contained a non-numeric value'

    # Remove the final=(...)m chunk so the remainder is plain whitespace-separated
    # key=value tokens (true for every other field in this exact log format).
    remainder = rest[:fm.start()] + rest[fm.end():]
    tokens = remainder.split()
    values = {}
    for tok in tokens:
        if '=' not in tok:
            continue
        key, _, raw = tok.partition('=')
        values[key] = raw

    for key, unit in NUMERIC_FIELD_UNITS.items():
        raw = values.get(key)
        if raw is None:
            return None, f'missing field "{key}"'
        if not raw.endswith(unit):
            return None, f'field "{key}" value "{raw}" missing expected unit suffix "{unit}"'
        numeric = raw[:-len(unit)]
        try:
            rec[key] = float(numeric)
        except ValueError:
            return None, f'field "{key}" value "{raw}" is not numeric'

    notes_raw = values.get('notes_captured')
    if notes_raw is None:
        return None, 'missing field "notes_captured"'
    rec['notes_captured'] = notes_raw.strip().lower() == 'true'

    collisions_raw = values.get('collisions', '')
    rec['collisions'] = [c for c in collisions_raw.split(',') if c]

    for key in REQUIRED_KEYS:
        if key not in rec:
            return None, f'missing field "{key}"'
    return rec, None


def parse_console_log(path):
    """Returns (records, n_malformed, error) where error is a human-readable string if
    the file could not be read at all (records/n_malformed are then empty/0)."""
    if not path.exists():
        return [], 0, f'コンソールログファイルが見つかりません: {path}'
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError as exc:
        return [], 0, f'コンソールログファイルを読み込めません: {path} ({exc})'
    if not text.strip():
        return [], 0, None  # exists but empty -- not a crash, just "no data"

    records, n_malformed = [], 0
    for lineno, line in enumerate(text.splitlines(), start=1):
        if 'StartToBingo RESULT' not in line:
            continue
        rec, reason = parse_result_line(line)
        if rec is not None:
            records.append(rec)
        elif reason is not None:
            n_malformed += 1
            print(f'[phase0_baseline_eval] 警告: {path}:{lineno}: 行を解析できません ({reason})',
                  file=sys.stderr)
    return records, n_malformed, None


# --------------------------------------------------------------------------------------
# consolidated/*.csv handling
# --------------------------------------------------------------------------------------

def load_pose_csv(path):
    """Returns (rows, error) where rows is a list of (t, x, y, yaw) sorted by t, and
    error is a human-readable string if the file is missing/unreadable/empty. NaN or
    unparsable rows are skipped and counted, with a warning if a meaningful fraction of
    the file was unusable."""
    if not path.exists():
        return [], f'{path.name} が見つかりません ({path})'
    try:
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            raw_rows = list(reader)
    except OSError as exc:
        return [], f'{path.name} を読み込めません ({exc})'
    if not raw_rows:
        return [], f'{path.name} にデータ行がありません（ヘッダのみ、または空ファイル）'

    rows, n_bad = [], 0
    for r in raw_rows:
        try:
            t, x, y, yaw = float(r['t']), float(r['x']), float(r['y']), float(r['yaw'])
        except (KeyError, ValueError, TypeError):
            n_bad += 1
            continue
        if any(math.isnan(v) or math.isinf(v) for v in (t, x, y, yaw)):
            n_bad += 1
            continue
        rows.append((t, x, y, yaw))
    rows.sort(key=lambda r: r[0])
    if not rows:
        return [], f'{path.name} の全 {len(raw_rows)} 行が NaN/不正値で使用不能でした'
    if n_bad > 0:
        print(f'[phase0_baseline_eval] 警告: {path.name}: {n_bad}/{len(raw_rows)} 行を'
              f'NaN/不正値としてスキップしました', file=sys.stderr)
    return rows, None


def nearest_match_errors(gt_rows, est_rows):
    """For each (t,x,y,yaw) in est_rows, find the nearest-timestamp gt_rows entry and
    return (position_errors_mm, yaw_errors_deg, n_matched)."""
    if not gt_rows or not est_rows:
        return [], [], 0
    gt_times = [r[0] for r in gt_rows]
    pos_err, yaw_err = [], []
    for t, x, y, yaw in est_rows:
        idx = bisect.bisect_left(gt_times, t)
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(gt_times)]
        best = min(candidates, key=lambda i: abs(gt_times[i] - t))
        _, gx, gy, gyaw = gt_rows[best]
        pos_err.append(math.hypot(x - gx, y - gy) * 1000.0)
        yaw_err.append(abs(math.degrees(wrap(yaw - gyaw))))
    return pos_err, yaw_err, len(pos_err)


def summarize(values):
    """Returns dict(mean, rmse, max, n) or None if values is empty. Never fabricates a
    number from zero samples."""
    if not values:
        return None
    n = len(values)
    mean = statistics.mean(values)
    rmse = math.sqrt(sum(v * v for v in values) / n)
    return {'mean': mean, 'rmse': rmse, 'max': max(values), 'n': n}


# --------------------------------------------------------------------------------------
# wall clearance (best-effort; center-point distance only, see report caveat)
# --------------------------------------------------------------------------------------

def point_to_segment_distance(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
    cx, cy = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy)


def compute_wall_clearance(gt_rows, field_json_path):
    """Returns (result_dict_or_None, note). result_dict has keys
    min_clearance_mm/t/wall_id when computable. This is a CENTER-POINT distance from the
    ground-truth (x,y) to the nearest real wall segment -- it does NOT subtract the
    robot's own half-extent (chassis is a 0.5 m-side triangle), so true chassis-edge
    clearance is smaller than this number. Non-physical reference/alignment segments
    (confidence containing "alignment") are excluded."""
    if not gt_rows:
        return None, 'ground_truth.csv にデータがないため計算できません'
    if not field_json_path.exists():
        return None, f'フィールド定義ファイルが見つかりません ({field_json_path})'
    try:
        field = json.loads(field_json_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        return None, f'フィールド定義ファイルを読み込めません ({exc})'
    segments = [s for s in field.get('walls', {}).get('segments', [])
                if 'alignment' not in s.get('confidence', '')]
    if not segments:
        return None, 'フィールド定義に実壁セグメントが見つかりません'

    best_dist, best_t, best_id = math.inf, None, None
    for t, x, y, _yaw in gt_rows:
        for seg in segments:
            d = point_to_segment_distance(x, y, seg['x1'], seg['y1'], seg['x2'], seg['y2'])
            if d < best_dist:
                best_dist, best_t, best_id = d, t, seg['id']
    return {'min_clearance_mm': best_dist * 1000.0, 't': best_t, 'wall_id': best_id}, None


# --------------------------------------------------------------------------------------
# report rendering
# --------------------------------------------------------------------------------------

LAP_COLUMNS = [
    ('lap', '{:d}', 'lap'),
    ('completion_time', '{:.3f}s', 'completion_time'),
    ('target_error', '{:.2f}mm', 'target_error'),
    ('terminal_yaw_error', '{:.2f}deg', 'terminal_yaw_error'),
    ('tracking_rmse', '{:.2f}mm', 'tracking_rmse'),
    ('tracking_max', '{:.2f}mm', 'tracking_max'),
    ('max_accel', '{:.2f}mps2', 'max_accel'),
    ('wheel_odom_rmse', '{:.2f}mm', 'wheel_odom_rmse'),
    ('wheel_odom_max', '{:.2f}mm', 'wheel_odom_max'),
    ('pickup_error', '{:.2f}mm', 'pickup_error'),
    ('pickup_speed', '{:.2f}mm/s', 'pickup_speed'),
    ('collisions', '{}', 'collisions'),
]

# The 経路追従.md §19 Phase 0 checklist item "現行6.53 s基準の再現" cites a "current 6.53s
# baseline". NOTE (found while writing this script, 2026-07-31): that number does not
# actually appear in docs/start_to_bingo_baseline_and_algorithm_survey.md -- that document's
# "Final baseline test" table explicitly lists completion time under "Limitations of this
# test" as "no measured completion timestamp". The only place "6.53 s" appears in the repo
# is 経路追従.md itself. So this script treats 6.53s as an FYI reference number of unclear
# provenance, not a verified historical measurement -- and per the task's own instruction,
# this comparison is informational only, never a pass/fail gate.
HISTORICAL_COMPLETION_TIME_S = 6.53
HISTORICAL_NOTE = (
    '経路追従.md §19 に「現行6.53 s基準」と記載されているが、この数値は '
    'docs/start_to_bingo_baseline_and_algorithm_survey.md の "Final baseline test" 表には '
    '出現しない（同表は completion time を明示的に "no measured completion timestamp" と '
    'している）。6.53s の出典は経路追従.md本文のみであり、実測値として検証できていない。'
)


def fmt_localization_block(title, stats):
    lines = [f'  [{title} vs ground_truth]']
    if stats is None:
        lines.append('    データなし（サンプル0件、または片方/両方のCSVが利用不能）')
        return lines
    pos, yaw, n = stats
    if pos is None:
        lines.append(f'    データなし（マッチング可能なサンプルが0件, n={n}）')
        return lines
    caveat = '  ※サンプル数が少なく参考値' if pos['n'] < 5 else ''
    lines.append(f'    サンプル数(最近傍マッチ): {pos["n"]}{caveat}')
    lines.append(f'    位置誤差 平均: {pos["mean"]:.2f} mm')
    lines.append(f'    位置誤差 RMSE: {pos["rmse"]:.2f} mm')
    lines.append(f'    位置誤差 最大: {pos["max"]:.2f} mm')
    if yaw is not None:
        lines.append(f'    (参考) yaw誤差 平均/RMSE/最大: '
                      f'{yaw["mean"]:.3f} / {yaw["rmse"]:.3f} / {yaw["max"]:.3f} deg')
    return lines


def render_report(run_dir, console_log_path, lap_records, n_malformed, console_error,
                   se_stats, odom_stats, wall_clearance, wall_note, collisions_rows,
                   collisions_error):
    L = []
    add = L.append
    add('=' * 78)
    add('Phase 0 ベースライン計測レポート')
    add(f'  対象run_dir     : {run_dir}')
    add(f'  コンソールログ  : {console_log_path}')
    add('=' * 78)
    add('')

    # --- Unity側 per-lap 計測値 -----------------------------------------------------
    add('-- ラップ別 Unity計測値 (StartToBingoValidation.Complete() の Debug.Log) --')
    if console_error:
        add(f'  エラー: {console_error}')
        add('  -> per-lapのUnity計測値は利用できません。以下はconsolidated CSVのみの報告です。')
    elif not lap_records:
        add('  [StartToBingo RESULT ...] 行はコンソールログ内に0件でした。')
        add('  -> ラップは実行されなかったか、コンソール出力がこのログに含まれていません。')
        if n_malformed:
            add(f'  (なお、RESULT行らしき行が {n_malformed} 件見つかりましたが解析に失敗しました。'
                'stderrの警告を参照してください。)')
    else:
        header = ('lap  completion_time  target_error  terminal_yaw_error  tracking_rmse  '
                   'tracking_max  max_accel   wheel_odom_rmse  wheel_odom_max  pickup_error  '
                   'pickup_speed  notes  collisions')
        add('  ' + header)
        for r in lap_records:
            collisions_str = ','.join(r['collisions']) if r['collisions'] else '(なし)'
            add('  {lap:>3d}  {completion_time:>14.3f}s  {target_error:>11.2f}mm  '
                '{terminal_yaw_error:>17.2f}deg  {tracking_rmse:>12.2f}mm  '
                '{tracking_max:>11.2f}mm  {max_accel:>8.2f}mps2  '
                '{wheel_odom_rmse:>14.2f}mm  {wheel_odom_max:>13.2f}mm  '
                '{pickup_error:>11.2f}mm  {pickup_speed:>11.2f}mm/s  '
                '{notes_captured!s:>5}  {collisions_str}'.format(
                    collisions_str=collisions_str, **r))
        if n_malformed:
            add(f'  ({n_malformed} 件の RESULT らしき行は解析に失敗し除外されました。'
                'stderrの警告を参照)')
        add('')
        add('  -- 集計 (全ラップ) --')
        n = len(lap_records)
        for key, unit_label in (('completion_time', 's'), ('target_error', 'mm'),
                                 ('terminal_yaw_error', 'deg'), ('tracking_rmse', 'mm'),
                                 ('tracking_max', 'mm'), ('max_accel', 'mps2')):
            vals = [r[key] for r in lap_records]
            add(f'  {key:22s}: n={n}  平均={statistics.mean(vals):.3f}{unit_label}  '
                f'最小={min(vals):.3f}{unit_label}  最大={max(vals):.3f}{unit_label}')
        n_collisions_laps = sum(1 for r in lap_records if r['collisions'])
        add(f'  衝突ありラップ数: {n_collisions_laps} / {n}')
    add('')

    # --- 自己位置推定 誤差 (対 ground_truth) ------------------------------------------
    add('-- 自己位置推定 誤差 (対 ground_truth.csv, 最近傍時刻マッチング) --')
    add('  ※ これは上記の tracking_rmse/tracking_max（目標軌道への追従誤差=制御性能の指標）')
    add('     とも、wheel_odom_rmse/wheel_odom_max（Unity内部の車輪デッドレコニング自己診断）')
    add('     とも別の指標です。ここではROS側 /state_estimate, /odom_fast を')
    add('     /ground_truth_pose と比較した「自己位置推定そのものの実誤差」を計測します。')
    add('')
    L.extend(fmt_localization_block('state_estimate', se_stats))
    add('')
    L.extend(fmt_localization_block('odom_fast', odom_stats))
    add('')

    # --- 壁クリアランス ------------------------------------------------------------
    add('-- 最小壁クリアランス (参考値) --')
    if wall_clearance is None:
        add(f'  未計算: {wall_note}')
    else:
        add(f'  最小クリアランス（ロボット中心点基準）: {wall_clearance["min_clearance_mm"]:.1f} mm'
            f'  (t={wall_clearance["t"]:.3f}s, wall={wall_clearance["wall_id"]})')
        add('  ※ ロボット中心点と壁セグメント間の最短距離であり、シャーシ（一辺0.5mの')
        add('     正三角形）の半径分は差し引いていない。実際の車体端-壁クリアランスは')
        add('     これより小さい可能性がある点に注意。')
    add('')

    # --- 衝突ログ -------------------------------------------------------------------
    add('-- 衝突ログ (consolidated/collisions.csv) --')
    if collisions_error:
        add(f'  未取得: {collisions_error}')
    elif not collisions_rows:
        add('  記録された衝突なし（0件）')
    else:
        add(f'  件数: {len(collisions_rows)}')
        for t, name, speed in collisions_rows[:20]:
            add(f'    t={t:.3f}s  name={name}  relative_speed={speed}mps')
        if len(collisions_rows) > 20:
            add(f'    ... 他 {len(collisions_rows) - 20} 件')
    add('')

    # --- 過去基準との比較 (参考のみ、合否判定ではない) --------------------------------
    add('-- 過去基準との比較 (参考情報のみ。Phase 0 は「今日の実測値」を確立する工程であり、')
    add('   旧基準への一致は合否判定条件ではない) --')
    if lap_records:
        times = [r['completion_time'] for r in lap_records]
        add(f'  今回の completion_time: 平均={statistics.mean(times):.3f}s '
            f'(最小={min(times):.3f}s, 最大={max(times):.3f}s, n={len(times)})')
        add(f'  参照値 (経路追従.md §19記載): {HISTORICAL_COMPLETION_TIME_S:.2f}s')
    else:
        add('  今回のcompletion_timeデータなし（比較不能）')
    add(f'  注記: {HISTORICAL_NOTE}')
    add('')
    add('=' * 78)
    return '\n'.join(L)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir', type=Path,
                         help='docs/stage4_runs/<run_tag>/ (must contain consolidated/)')
    parser.add_argument('console_log', type=Path,
                         help='text file with captured Unity console output')
    parser.add_argument('--field-json', type=Path, default=None,
                         help='path to robocon2026_field.json '
                              '(default: <repo_root>/config/robocon2026_field.json)')
    parser.add_argument('--out', type=Path, default=None,
                         help='report output path (default: <run_dir>/phase0_baseline_report.txt)')
    args = parser.parse_args()

    run_dir = args.run_dir
    consolidated_dir = run_dir / 'consolidated'
    out_path = args.out or (run_dir / 'phase0_baseline_report.txt')
    field_json_path = args.field_json or (Path(__file__).resolve().parents[1]
                                           / 'config' / 'robocon2026_field.json')

    if not run_dir.exists():
        print(f'[phase0_baseline_eval] エラー: run_dirが存在しません: {run_dir}', file=sys.stderr)
        sys.exit(1)
    if not consolidated_dir.exists():
        print(f'[phase0_baseline_eval] エラー: consolidated/ が見つかりません: {consolidated_dir}\n'
              '  -> 先に consolidate_stage_log.py を実行してください。CSVベースの指標は'
              '算出できませんが、コンソールログのみで可能な範囲を報告します。', file=sys.stderr)

    # --- console log -----------------------------------------------------------------
    lap_records, n_malformed, console_error = parse_console_log(args.console_log)
    if console_error:
        print(f'[phase0_baseline_eval] エラー: {console_error}', file=sys.stderr)
    elif not lap_records:
        print('[phase0_baseline_eval] 注意: コンソールログに [StartToBingo RESULT ...] 行が'
              '0件でした。CSVベースの指標のみ報告します。', file=sys.stderr)

    # --- consolidated CSVs -------------------------------------------------------------
    se_stats = odom_stats = None
    wall_clearance, wall_note = None, 'consolidated/ が見つからないため未計算'
    collisions_rows, collisions_error = [], 'consolidated/ が見つからないため未取得'

    if consolidated_dir.exists():
        gt_rows, gt_err = load_pose_csv(consolidated_dir / 'ground_truth.csv')
        if gt_err:
            print(f'[phase0_baseline_eval] 警告: {gt_err}', file=sys.stderr)

        se_rows, se_err = load_pose_csv(consolidated_dir / 'state_estimate.csv')
        if se_err:
            print(f'[phase0_baseline_eval] 警告: {se_err}', file=sys.stderr)
        if gt_rows and se_rows:
            pos, yaw, n = nearest_match_errors(gt_rows, se_rows)
            se_stats = (summarize(pos), summarize(yaw), n)
        else:
            se_stats = (None, None, 0)

        odom_rows, odom_err = load_pose_csv(consolidated_dir / 'odom_fast.csv')
        if odom_err:
            print(f'[phase0_baseline_eval] 警告: {odom_err}', file=sys.stderr)
        if gt_rows and odom_rows:
            pos, yaw, n = nearest_match_errors(gt_rows, odom_rows)
            odom_stats = (summarize(pos), summarize(yaw), n)
        else:
            odom_stats = (None, None, 0)

        wall_clearance, wall_note = compute_wall_clearance(gt_rows, field_json_path)

        collisions_path = consolidated_dir / 'collisions.csv'
        if not collisions_path.exists():
            collisions_error = f'{collisions_path} が見つかりません'
        else:
            collisions_error = None
            try:
                with open(collisions_path, newline='') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        try:
                            t = float(row['t'])
                        except (KeyError, ValueError, TypeError):
                            continue
                        collisions_rows.append((t, row.get('name', ''),
                                                 row.get('relative_speed_mps', '')))
            except OSError as exc:
                collisions_error = f'{collisions_path} を読み込めません ({exc})'

    report = render_report(run_dir, args.console_log, lap_records, n_malformed, console_error,
                            se_stats, odom_stats, wall_clearance, wall_note, collisions_rows,
                            collisions_error)
    print(report)
    try:
        out_path.write_text(report + '\n', encoding='utf-8')
        print(f'\n[phase0_baseline_eval] レポートを書き込みました: {out_path}', file=sys.stderr)
    except OSError as exc:
        print(f'[phase0_baseline_eval] エラー: レポートファイルを書き込めません: '
              f'{out_path} ({exc})', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
