"""Serialized merge of new observations into the common map, never a cold overwrite."""
from contextlib import contextmanager
from pathlib import Path
import os
import time
import numpy as np

from .occupancy import UNKNOWN


@contextmanager
def map_lock(path, timeout=20.):
    lock=Path(str(path)+'.lock');lock.parent.mkdir(parents=True,exist_ok=True)
    with lock.open('a+b') as handle:
        if handle.tell()==0:handle.write(b'0');handle.flush()
        started=time.monotonic()
        while True:
            try:
                handle.seek(0)
                if os.name=='nt':
                    import msvcrt
                    msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
                else:
                    import fcntl
                    fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic()-started>=timeout:raise TimeoutError('Shared map is busy')
                time.sleep(.05)
        try:yield
        finally:
            handle.seek(0)
            if os.name=='nt':msvcrt.locking(handle.fileno(),msvcrt.LK_UNLCK,1)
            else:fcntl.flock(handle.fileno(),fcntl.LOCK_UN)


def merge_observations(delta, path, approaches=None):
    """Only observed delta cells overwrite disk; seed history never enters this delta."""
    from .persistent_map import PersistentMap
    with map_lock(path):
        disk=PersistentMap.load(str(path),delta.res,delta.block,delta.origin,strict=True)
        if disk.res!=delta.res or disk.block!=delta.block or not np.allclose(disk.origin,delta.origin):
            raise ValueError('Shared map geometry differs from the current observations')
        for key,block in list(delta.blocks.items()):
            known=block!=UNKNOWN
            if not known.any():continue
            if key not in disk.blocks:disk.blocks[key]=np.full_like(block,UNKNOWN)
            np.copyto(disk.blocks[key],block,where=known)
        disk.approaches.update(approaches or {})
        disk.save(str(path),strict=True)
        return disk.stats()
