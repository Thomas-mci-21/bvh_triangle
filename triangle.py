from bvh import AABB

class Triangle:
    def __init__(self, vertices, color):
        self.vertices = vertices
        self.color = color
        min_point = [min([v[i] for v in vertices]) for i in range(3)]
        max_point = [max([v[i] for v in vertices]) for i in range(3)]
        self.bbox = AABB(min_point, max_point)

    # Möller–Trumbore intersection algorithm
    def triangle_intersect(self, ray_origin, ray_dir):
        p0, p1, p2 = self.vertices
        # e1 = p1 - p0
        # e2 = p2 - p0
        e1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
        e2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]]
        # s = ray_origin (o) - p0
        s = [ray_origin[0] - p0[0], ray_origin[1] - p0[1], ray_origin[2] - p0[2]]
        # Cross Product: s1 = ray_dir (d) x e2
        s1 = [
            ray_dir[1] * e2[2] - ray_dir[2] * e2[1],
            ray_dir[2] * e2[0] - ray_dir[0] * e2[2],
            ray_dir[0] * e2[1] - ray_dir[1] * e2[0]
        ]
        # Cross Product: s2 = s x e1
        s2 = [
            s[1] * e1[2] - s[2] * e1[1],
            s[2] * e1[0] - s[0] * e1[2],
            s[0] * e1[1] - s[1] * e1[0]
        ]
        # Dot Product: a = e1 · s1
        a = e1[0] * s1[0] + e1[1] * s1[1] + e1[2] * s1[2]
        if abs(a) < 1e-5:
            return None  # Ray is parallel to the triangle
        f = 1.0 / a
        # Dot Product: u = f * (s · s1)
        u = f * (s[0] * s1[0] + s[1] * s1[1] + s[2] * s1[2])
        if u < 0.0 or u > 1.0:
            return None
        # v = f * (ray_dir (d) · s2)
        v = f * (ray_dir[0] * s2[0] + ray_dir[1] * s2[1] + ray_dir[2] * s2[2])
        if v < 0.0 or u + v > 1.0:
            return None
        # t = f * (e2 · s2)
        t = f * (e2[0] * s2[0] + e2[1] * s2[1] + e2[2] * s2[2])
        return t if t >= 0 else None

