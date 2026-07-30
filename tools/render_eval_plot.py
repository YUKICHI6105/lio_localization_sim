#!/usr/bin/env python3
"""Render an evaluator PNG after a ROS run has completed.

The real-time evaluator deliberately writes only a compact report and CSV from
C++.  Matplotlib stays out of the 1 kHz ROS callback path; this tool consumes
that finished CSV afterwards.
"""

import argparse
import csv
import os
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('errors_csv', type=Path)
    parser.add_argument('output_png', type=Path)
    args = parser.parse_args()

    times: list[float] = []
    position_errors: list[float] = []
    yaw_errors: list[float] = []
    with args.errors_csv.open(encoding='utf-8', newline='') as file:
        for row in csv.DictReader(file):
            times.append(float(row['t']))
            position_errors.append(float(row['pos_err_mm']))
            yaw_errors.append(float(row['yaw_err_deg']))
    if not times:
        raise ValueError(f'No error samples in {args.errors_csv}')

    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    # Do not create a user-home cache while the experiment artifacts are being
    # produced in a sandboxed/CI environment.
    os.environ.setdefault('MPLCONFIGDIR', str(args.output_png.parent / '.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(times, position_errors, linewidth=0.7, color='tab:red')
    axes[0].axhline(10.0, color='green', linestyle='--', label='10 mm requirement')
    axes[0].set_ylabel('position error [mm]')
    axes[0].legend()
    axes[0].grid(True)
    axes[1].plot(times, yaw_errors, linewidth=0.7, color='tab:blue')
    axes[1].set_xlabel('simulation time [s]')
    axes[1].set_ylabel('yaw error [deg]')
    axes[1].grid(True)
    fig.tight_layout()
    fig.savefig(args.output_png, dpi=120)


if __name__ == '__main__':
    main()
