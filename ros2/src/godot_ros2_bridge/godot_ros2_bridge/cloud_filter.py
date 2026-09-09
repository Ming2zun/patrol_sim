"""Height-filtered, voxel-deduplicated rolling accumulation in map coordinates."""
import math
import numpy as np


class RollingCloud:
    def __init__(self, min_z=.15, radius=80., voxel=.2, max_voxels=500000):
        if not all(map(math.isfinite, (min_z, radius, voxel))) or radius <= 0 or voxel <= 0 or max_voxels < 1:
            raise ValueError('Invalid height/radius/voxel/capacity')
        self.min_z, self.radius, self.voxel, self.max_voxels = min_z, radius, voxel, max_voxels
        self.cells = {}
        self.evicted = 0

    def clear(self):
        self.cells.clear()
        self.evicted = 0

    def add(self, points, center):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        delta = points[:, :2] - np.asarray(center[:2])
        points = points[(points[:, 2] >= self.min_z) & (np.sum(delta*delta, axis=1) <= self.radius**2)]
        keys = np.floor(points.astype(np.float64)/self.voxel).astype(np.int64)
        for key, point in zip(keys, points):
            self.cells[tuple(key)] = tuple(point)
        while len(self.cells) > self.max_voxels:
            self.cells.pop(next(iter(self.cells)))
            self.evicted += 1

    def prune(self, center):
        x, y = center[:2]
        outside = [k for k, p in self.cells.items() if (p[0]-x)**2+(p[1]-y)**2 > self.radius**2]
        for key in outside:
            del self.cells[key]
        return len(outside)

    def points(self):
        return np.asarray(list(self.cells.values()), dtype=np.float32).reshape(-1, 3)
