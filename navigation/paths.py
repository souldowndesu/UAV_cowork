"""Project paths are independent of the caller's current working directory."""
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = PROJECT_ROOT / 'data'
RECORDS = DATA_ROOT / 'records'
MAPS = DATA_ROOT / 'maps'
EXAMPLES = DATA_ROOT / 'examples'
ENVIRONMENT = PROJECT_ROOT / 'environment'


def project_path(value):
    path = Path(value).expanduser()
    return (path if path.is_absolute() else PROJECT_ROOT / path).resolve()
