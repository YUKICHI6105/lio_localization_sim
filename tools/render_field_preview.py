#!/usr/bin/env python3
"""Render the engine-neutral Robocon field JSON as a zero-runtime-cost SVG."""

import argparse
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('definition', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    data = json.loads(args.definition.read_text(encoding='utf-8'))
    field = data['field']
    length, width = field['length'], field['width']
    scale, margin = 150.0, 60.0
    canvas_w, canvas_h = length * scale + 2 * margin, width * scale + 2 * margin

    def sx(x):
        return margin + (x + length / 2) * scale

    def sy(y):
        return margin + (width / 2 - y) * scale

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{canvas_w:.0f}" '
        f'height="{canvas_h:.0f}" viewBox="0 0 {canvas_w:.1f} {canvas_h:.1f}">',
        '<rect width="100%" height="100%" fill="#e9eef5"/>',
        '<text x="60" y="34" font-family="sans-serif" font-size="20" '
        'font-weight="bold">千葉大学ロボコン2026 — エンジン共通フィールド定義</text>',
    ]
    for zone in data['zones']:
        x = sx(zone['x_min'])
        w = (zone['x_max'] - zone['x_min']) * scale
        y_max = zone.get('y_max', width / 2)
        y_min = zone.get('y_min', -width / 2)
        parts.append(
            f'<rect x="{x:.2f}" y="{sy(y_max):.2f}" width="{w:.2f}" '
            f'height="{(y_max-y_min) * scale:.2f}" fill="{zone["colour"]}" opacity="0.78"/>')
        parts.append(
            f'<text x="{x + w / 2:.2f}" y="{sy((y_min+y_max)/2):.2f}" text-anchor="middle" '
            f'font-family="sans-serif" font-size="16" fill="white" '
            f'transform="rotate(-90 {x + w / 2:.2f} {sy((y_min+y_max)/2):.2f})">{zone["id"]}</text>')
    thickness = data['walls']['thickness'] * scale
    for wall in data['walls']['segments']:
        parts.append(
            f'<line x1="{sx(wall["x1"]):.2f}" y1="{sy(wall["y1"]):.2f}" '
            f'x2="{sx(wall["x2"]):.2f}" y2="{sy(wall["y2"]):.2f}" '
            f'stroke="#f7f7f7" stroke-width="{thickness:.2f}" stroke-linecap="square"/>')
    for colour, points in data['notes'].items():
        if colour == 'confidence':
            continue
        fill = '#1260ff' if colour == 'blue' else '#ff6412'
        for x, y in points:
            parts.append(
                f'<circle cx="{sx(x):.2f}" cy="{sy(y):.2f}" '
                f'r="{data["rules"]["note_diameter"] * scale / 2:.2f}" '
                f'fill="{fill}" stroke="white" stroke-width="1"/>')
    bingo = data['bingo']
    bx, by = sx(bingo['centre']['x']), sy(bingo['centre']['y'])
    pitch = (bingo['clear_cell'] + bingo['vertical_frame']) * scale
    bingo_width = bingo['width'] * scale
    left = bx - bingo_width / 2
    depth = bingo['total_depth'] * scale
    top = by - depth / 2
    parts.append(f'<rect x="{left:.2f}" y="{top:.2f}" width="{bingo_width:.2f}" height="{depth:.2f}" fill="#aeb4bd" stroke="#333" stroke-width="3"/>')
    for i in range(4):
        post_x = left + bingo['vertical_frame'] * scale / 2 + i * pitch
        parts.append(f'<line x1="{post_x:.2f}" y1="{top:.2f}" x2="{post_x:.2f}" y2="{top + depth:.2f}" stroke="#333" stroke-width="4"/>')
    parts.append(
        f'<text x="{canvas_w/2:.1f}" y="{canvas_h-15:.1f}" text-anchor="middle" '
        f'font-family="sans-serif" font-size="14">{length:.3f} m × {width:.3f} m / +X: start → bingo</text>')
    parts.append('</svg>')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('\n'.join(parts) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
