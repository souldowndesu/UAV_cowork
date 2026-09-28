"""Verify mark_visible_free floods only through FREE (not UNKNOWN), so it does NOT
flood occluded space behind a wall. Synthetic map: drone at center, a wall at +X with
a small gap, UNKNOWN everywhere behind the wall and elsewhere."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from nav.occupancy import OccupancyMap, UNKNOWN, FREE, OCCUPIED

nx, ny, nz = 60, 60, 20
res = 0.4
origin = np.array([-12.0, -12.0, -4.0])
om = OccupancyMap(nx, ny, nz, res, origin)
om.data[:] = UNKNOWN

# drone at world (0,0,0) -> voxel (30,30,10)
center = np.array([0.0, 0.0, 0.0])
ci, cj, ck = om.world_to_voxel(center)

# Wall at X voxels 38..44 (world x = 3.2 .. 5.6), spanning full Y, full Z.
# Leave a gap at Y voxel 30 (y=0) of width 2 (voxels 29,30) so a thin gap exists.
om.data[38:45, :, :] = OCCUPIED
om.data[38:45, 29:31, :] = UNKNOWN  # gap

# Free space between drone and wall (X voxels 31..37), full Y, at drone's z band.
om.data[31:38, :, ck-2:ck+3] = FREE
# Also mark some free voxels along rays from drone toward gap and open sky.
om.data[ci, cj, ck] = FREE

# Behind wall should stay UNKNOWN. Before wall is FREE.
om.mark_visible_free(center, max_range=10.0)

# Check: space immediately behind wall (X voxel 45) must NOT be FREE (occluded).
behind = om.data[45, :, ck]
print("behind wall FREE count (should be 0):", int((behind == FREE).sum()))
print("behind wall UNKNOWN count:", int((behind == UNKNOWN).sum()))
# Check: free space in front (X voxel 34) is FREE.
front = om.data[34, :, ck]
print("front of wall FREE count (should be >0):", int((front == FREE).sum()))
# Check: the gap itself (X voxel 38..44 at Y 29..30) — gap voxels adjacent to FREE may be marked.
gap = om.data[38:45, 29:31, ck]
print("gap FREE count:", int((gap == FREE).sum()), "gap UNKNOWN count:", int((gap == UNKNOWN).sum()))
# Check: drone voxel is FREE
print("drone voxel:", om.data[ci, cj, ck])

# Assertions
assert om.data[ci, cj, ck] == FREE, "drone voxel not FREE"
assert (om.data[45, :, ck] == FREE).sum() == 0, "occluded space behind wall got flooded FREE!"
print("PASS: occluded space preserved, front space free")
