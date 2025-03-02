import numpy as np

# The same for triangle & 3dgs
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

# The same for triangle & 3dgs    
class BVHNode:
    def __init__(self, bbox, left=None, right=None, objects=None):
        self.bbox = bbox
        self.left = left
        self.right = right
        self.objects = objects or []
# The same for triangle & 3dgs
def compute_bbox(objects):
    min_point = [float('inf')] * 3
    max_point = [float('-inf')] * 3
    for obj in objects:
        for i in range(3):
            min_point[i] = min(min_point[i], obj.bbox.min[i])
            max_point[i] = max(max_point[i], obj.bbox.max[i])
    return AABB(min_point, max_point)

# build BVH Tree based on stack
# The same for triangle & 3dgs
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
