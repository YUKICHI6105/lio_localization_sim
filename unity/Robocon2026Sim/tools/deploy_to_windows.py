#!/usr/bin/env python3
"""Deploy the Unity source project to a case-insensitive Windows directory."""

import os
import shutil
from pathlib import Path


source = Path(__file__).resolve().parents[1]
destination = Path(os.environ.get(
    'ROBOCON_UNITY_WINDOWS_PROJECT',
    '/mnt/c/Users/kouza.FUKU-PC/UnityProjects/Robocon2026Sim')).resolve()
marker = destination / '.robocon2026-managed'

if destination == source or source in destination.parents:
    raise SystemExit(f'Unsafe deployment destination: {destination}')
if destination.exists() and not marker.exists():
    raise SystemExit(
        f'Refusing to overwrite an unmanaged directory: {destination}\n'
        'Choose an empty path with ROBOCON_UNITY_WINDOWS_PROJECT.')

destination.mkdir(parents=True, exist_ok=True)
marker.write_text(
    'Generated from /home/yukichi6105/ros2_ws/src/lio_localization_sim/unity/Robocon2026Sim\n',
    encoding='utf-8')

for directory in ('Assets', 'Packages', 'ProjectSettings'):
    shutil.copytree(source / directory, destination / directory, dirs_exist_ok=True)
for filename in ('.gitignore', 'README.md'):
    shutil.copy2(source / filename, destination / filename)

print(f'deployed Unity project: {source} -> {destination}')
