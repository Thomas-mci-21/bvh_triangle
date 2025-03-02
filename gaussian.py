import numpy as np
from bvh import AABB
class Gaussian3D:
    def __init__(self, center, color, opacity, scale):
        self.center = center
        self.color = color
        self.opacity = opacity
        self.scale = scale
        # calculate AABB 
        min_point = [center[i] - scale for i in range(3)]
        max_point = [center[i] + scale for i in range(3)]
        self.bbox = AABB(min_point, max_point)

    def gaussian_intersect(self, ray_origin, ray_dir):
        # an easy test
        if not self.bbox.AABB_intersect(ray_origin, ray_dir):
            return None
        
        t = np.dot(np.array(self.center) - np.array(ray_origin), np.array(ray_dir))
        if t < 0:
            return None
        return t