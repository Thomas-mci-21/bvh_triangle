import numpy as np
import math
import matplotlib.pyplot as plt

class AABB:
    def __init__(self, min_point, max_point):
        self.min = min_point
        self.max = max_point

    # whether the ray intersects the bounding box
    def AABB_intersect(self, ray_origin, ray_dir):
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

class BVHNode:
    def __init__(self, bbox, left=None, right=None, objects=None):
        self.bbox = bbox
        self.left = left
        self.right = right
        self.objects = objects or []

def compute_bbox(objects):
    min_point = [float('inf')] * 3
    max_point = [float('-inf')] * 3
    for obj in objects:
        for i in range(3):
            min_point[i] = min(min_point[i], obj.bbox.min[i])
            max_point[i] = max(max_point[i], obj.bbox.max[i])
    return AABB(min_point, max_point)

# build BVH Tree based on stack
def build_bvh(objects, max_objects=2):
    stack = [(objects, None, None, None)]  # (objects, parent, is_left, index)
    root = None

    while stack:
        current_objects, parent, is_left, index = stack.pop()
        bbox = compute_bbox(current_objects)
        node = BVHNode(bbox=bbox, objects=current_objects)

        if parent is None:
            root = node
        elif is_left:
            parent.left = node
        else:
            parent.right = node

        if len(current_objects) > max_objects:
            
            # calculate center of each current object's AABB
            centers = []
            for obj in current_objects:
                center = [(obj.bbox.min[i] + obj.bbox.max[i]) / 2 for i in range(3)]
                centers.append(center)

            # calculate span of centers in each axis
            span = [0] * 3
            for i in range(3):
                min_val = float('inf')
                max_val = float('-inf')
                for center in centers:
                    if center[i] < min_val:
                        min_val = center[i]
                    if center[i] > max_val:
                        max_val = center[i]
                span[i] = max_val - min_val

            # Find the axis with the maximum span
            max_span = span[0]
            axis = 0
            for i in range(1, 3):
                if span[i] > max_span:
                    max_span = span[i]
                    axis = i

            current_objects.sort(key=lambda x: (x.bbox.min[axis] + x.bbox.max[axis]) / 2)
            mid = len(current_objects) // 2
            stack.append((current_objects[:mid], node, True, 0))
            stack.append((current_objects[mid:], node, False, 1))

    return root

# ray tracing based on stack
def trace_ray(ray_origin, ray_dir, node):
    stack = [node]
    closest = None

    while stack:
        current_node = stack.pop()
        if not current_node.bbox.AABB_intersect(ray_origin, ray_dir):
            continue

        if current_node.objects:
            for obj in current_node.objects:
                t = obj.triangle_intersect(ray_origin, ray_dir)
                #if t is not None:
                    #print(f"Intersection found at t = {t} with object")
                if t is not None and (closest is None or t < closest[0]):
                    closest = (t, obj)
        else:
            stack.append(current_node.left)
            stack.append(current_node.right)

    return closest

# read .obj & create object
def read_obj_file(file_path):
    vertices = []
    triangles = []
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if not parts:
                continue
            
            if parts[0] == 'v':
                vertex = [float(parts[1]), float(parts[2]), float(parts[3])]
                # scale and move the object
                scaled_vertex = [vertex[0] * 20, vertex[1] * 20, vertex[2] * 20 + 10]
                vertices.append(scaled_vertex)
            
            elif parts[0] == 'f':
                # create triangle
                indices = [int(parts[1]) - 1, int(parts[2]) - 1, int(parts[3]) - 1]
                triangle_vertices = [vertices[i] for i in indices]
                # set all triangles white
                triangle = Triangle(vertices=triangle_vertices, color=(1, 1, 1))
                triangles.append(triangle)
    print(f"Read {len(vertices)} vertices and {len(triangles)} triangles from the .obj file.")
    return triangles

#  read .obj file
file_path = r'models\bunny\bunny.obj'
objects = read_obj_file(file_path)

bvh_root = build_bvh(objects)

# rendering parameters
width, height = 100, 100
aspect_ratio = width / height
fov = 60  # field of view
scale = np.tan(np.radians(fov/2))
image = np.zeros((height, width, 3))

# interate ray tracing for every pixel
for y in range(height):
    for x in range(width):
        # convert to[-1, -1] * [-1, -1]
        px = (2 * (x + 0.5)/width - 1) * aspect_ratio * scale
        py = (1 - 2 * (y + 0.5)/height) * scale
        # ray from [0, 0, 0] point to [px, py, 1] & normalization
        ray_dir = [px, py, 1]
        norm = math.sqrt(ray_dir[0] ** 2 + ray_dir[1] ** 2 + ray_dir[2] ** 2)
        ray_dir = [i / norm for i in ray_dir]
        # ray tracing
        # returns (t, obj) or None, t: intersection distance, obj: object
        hit = trace_ray(np.array([0, 0, 0]), ray_dir, bvh_root)
        if hit:
            # update the image's color based on the object's color
            image[y, x] = hit[1].color

# show the image
plt.figure(figsize=(18, 18))
plt.imshow(image)
plt.axis('off')
plt.show()