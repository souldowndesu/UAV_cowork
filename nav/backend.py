"""Never load a native extension built from different kernel sources."""
import hashlib
import os
from pathlib import Path

def source_hash():
    root = Path(__file__).resolve().parents[1] / 'cpp'
    return hashlib.sha256(b''.join((root / p).read_bytes().replace(b'\r\n', b'\n')
                                 for p in ('fast_kernels.cpp', 'fast_planning.cpp'))).hexdigest()

BUILD_HASH = source_hash()
try:
    from . import _fast as native
    if os.environ.get('NAV_FORCE_PYTHON') == '1' or getattr(native, 'source_hash', None) != BUILD_HASH:
        native = None
except ImportError:
    native = None
