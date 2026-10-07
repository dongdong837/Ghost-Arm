"""All project resources are resolved relative to this source checkout."""
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / 'assets/worlds/ghost_room.sdf'
MODEL = ROOT / 'assets/models/ghost_display.urdf'
TEMPLATE = ROOT / 'assets/templates/room_template.sdf'
CONFIG = ROOT / 'config'
SCRIPTS = ROOT / 'scripts'
REPORTS = ROOT / 'runtime/reports'
MAPS = ROOT / 'runtime/maps'
for directory in (REPORTS, MAPS):
    directory.mkdir(parents=True, exist_ok=True)
