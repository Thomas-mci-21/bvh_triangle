import numpy as np
import matplotlib.pyplot as plt

class AABB:
    def __init__(self, min_point, max_point):
        self.min = np.array(min_point)
        self.max = np.array(max_point)
    # whether the ray intersects the bounding box 
    def intersect(self, ray_origin, ray_dir):
        
        # initialize tmin and tmax
        tmin = -np.inf
        tmax = np.inf
        
        # iterate x, y ,z axis
        for i in range(3):
            # Does not intersect if ray_origin is outside the bounding box && the ray direction is parallel 
            if ray_dir[i] == 0:
                if ray_origin[i] < self.min[i] or ray_origin[i] > self.max[i]:
                    return False
                continue
            # get the intersection points in the axis
            t1 = (self.min[i] - ray_origin[i]) / ray_dir[i]
            t2 = (self.max[i] - ray_origin[i]) / ray_dir[i]
            
            # when tmin < t < tmax, the ray has penetrated all three planes ONCE
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
        # if tmin < 0, the intersection does not exist 
        return tmax >= max(tmin, 0)

class Triangle:
    def __init__(self, vertices, color):
        self.vertices = np.array(vertices)
        self.color = np.array(color)
        min_point = np.min(self.vertices, axis=0)
        max_point = np.max(self.vertices, axis=0)
        self.bbox = AABB(min_point, max_point)
    # Möller–Trumbore intersection algorithm
    def intersect(self, ray_origin, ray_dir):
        
        p0, p1, p2 = self.vertices
        #  e1 = p1 - p0 
        #  e2 = p2 - p0
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

class BVHNode:
    def __init__(self, bbox, left=None, right=None, objects=None):
        self.bbox = bbox
        self.left = left
        self.right = right
        self.objects = objects or []

def compute_bbox(objects):
    min_point = np.full(3, np.inf)
    max_point = np.full(3, -np.inf)
    for obj in objects:
        min_point = np.minimum(min_point, obj.bbox.min)
        max_point = np.maximum(max_point, obj.bbox.max)
    return AABB(min_point, max_point)

def build_bvh(objects, max_objects=2):
    if len(objects) <= max_objects:
        return BVHNode(bbox=compute_bbox(objects), objects=objects)
    
    centers = np.array([(obj.bbox.min + obj.bbox.max)/2 for obj in objects])
    span = np.max(centers, axis=0) - np.min(centers, axis=0)
    axis = np.argmax(span)
    
    objects.sort(key=lambda x: (x.bbox.min[axis] + x.bbox.max[axis])/2)
    mid = len(objects)//2
    
    left = build_bvh(objects[:mid], max_objects)
    right = build_bvh(objects[mid:], max_objects)
    combined_bbox = AABB(
        np.minimum(left.bbox.min, right.bbox.min),
        np.maximum(left.bbox.max, right.bbox.max)
    )
    return BVHNode(bbox=combined_bbox, left=left, right=right)

# find the ray's closest intersection point with the objects
def trace_ray(ray_origin, ray_dir, node):
    
    if not node.bbox.intersect(ray_origin, ray_dir):
        return None
    
    # if leaf node, check intersection
    # get info of the closest intersection point
    if node.objects:
        closest = None
        for obj in node.objects:
            t = obj.intersect(ray_origin, ray_dir)
            if t is not None and (closest is None or t < closest[0]):
                closest = (t, obj)
        return closest
    
    hit_left = trace_ray(ray_origin, ray_dir, node.left)
    hit_right = trace_ray(ray_origin, ray_dir, node.right)
    
    # if not leaf node, travel to leaf
    hits = []
    if hit_left: hits.append(hit_left)
    if hit_right: hits.append(hit_right)
    return min(hits, key=lambda x: x[0]) if hits else None



# initialize the scence with objects
objects = [
    Triangle(vertices=[(0, -1, 5), (-1, 1, 5), (1, 1, 5)], color=(1, 0, 0)),
    Triangle(vertices=[(-2, -1, 7), (-3, 1, 7), (-1, 1, 7)], color=(0, 0.23, 0)),
    Triangle(vertices=[(3, -1, 6), (2, 1, 6), (1, 1, 10)], color=(1, 1, 1)),
]

bvh_root = build_bvh(objects)

# rendering parameters
width, height = 200, 200
aspect_ratio = width / height
fov = 60 # field of view
scale = np.tan(np.radians(fov/2))
image = np.zeros((height, width, 3))

# interate ray tracing for every pixel
for y in range(height):
    for x in range(width):
        
        # convert to[-1, -1] * [-1, -1]
        px = (2 * (x + 0.5)/width - 1) * aspect_ratio * scale
        py = (1 - 2 * (y + 0.5)/height) * scale
        
        # ray from [0, 0, 0] point to [px, py, 1] & normalization
        ray_dir = np.array([px, py, 1])
        ray_dir /= np.linalg.norm(ray_dir)
        
        # ray tracing
        # returns (t, obj) or None, t: intersection distance, obj: object 
        hit = trace_ray(np.array([0, 0, 0]), ray_dir, bvh_root)
        if hit: # update the image's color based on the object's color
            image[y, x] = hit[1].color

# show the image
plt.figure(figsize=(18, 18))
plt.imshow(image)
plt.axis('off')
plt.show()