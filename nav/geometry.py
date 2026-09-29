"""Voxel-centred distance queries with explicit validity and analytic gradients."""
import numpy as np


def distance_query(points, dist, origin, res):
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    shape = np.asarray(dist.shape)
    g = (p - origin) / res - .5
    valid = np.isfinite(g).all(axis=1) & (g >= -.5).all(axis=1) & (g < shape - .5).all(axis=1)
    g = np.clip(np.nan_to_num(g), 0, shape - 1)
    lo = np.floor(g).astype(int)
    hi = np.minimum(lo + 1, shape - 1)
    f = g - lo
    value = np.zeros(len(p))
    grad = np.zeros_like(p)
    for x in (0, 1):
        for y in (0, 1):
            for z in (0, 1):
                b = np.array([x, y, z])
                idx = np.where(b, hi, lo)
                d = dist[tuple(idx.T)]
                w = np.where(b, f, 1 - f)
                value += d * np.prod(w, axis=1)
                for a in range(3):
                    grad[:, a] += d * (2*b[a]-1) * np.prod(np.delete(w, a, axis=1), axis=1) / res
    raw_g = (p - origin) / res - .5
    grad[(raw_g < 0) | (raw_g >= shape - 1)] = 0
    value[~valid] = 0
    grad[~valid] = 0
    return value, grad, valid


def edge_clear(occ, dist, start, offset, minimum):
    """Require every intermediate cell of a diagonal's voxel box to be clear."""
    import itertools
    for delta in itertools.product(*[(0, d) if d else (0,) for d in offset]):
        v = tuple(a+b for a, b in zip(start, delta))
        if occ[v] == 2 or not np.isfinite(dist[v]) or dist[v] < minimum:
            return False
    return True
