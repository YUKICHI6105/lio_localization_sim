#!/usr/bin/env python3
"""Copy the engine-neutral field definition into Unity StreamingAssets."""

import hashlib
import json
import shutil
from pathlib import Path


project = Path(__file__).resolve().parents[1]
workspace = Path(__file__).resolve().parents[3]
source = workspace / 'src/lio_localization_sim/config/robocon2026_field.json'
destination = project / 'Assets/StreamingAssets/robocon2026_field.json'

data = json.loads(source.read_text(encoding='utf-8'))
if data.get('schema_version') != 1 or data.get('units') != 'metres':
    raise SystemExit(f'Unsupported field definition: {source}')

destination.parent.mkdir(parents=True, exist_ok=True)
shutil.copyfile(source, destination)
digest = hashlib.sha256(destination.read_bytes()).hexdigest()
print(f'synced {source} -> {destination}')
print(f'sha256={digest}')
