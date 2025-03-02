USE_3DGS = False  # Render triangle when False

import numpy as np
import math
import matplotlib.pyplot as plt

from bvh import AABB, BVHNode, compute_bbox, build_bvh
from triangle import Triangle
from read_triangle import read_obj_file
from gaussian import Gaussian3D

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
                if(not USE_3DGS):
                    t = obj.triangle_intersect(ray_origin, ray_dir)
                    #if t is not None:
                        #print(f"Intersection found at t = {t} with object")
                else:
                    t = obj.gaussian_intersect(ray_origin, ray_dir)
                
                if t is not None and (closest is None or t < closest[0]):
                    closest = (t, obj)
        else:
            stack.append(current_node.left)
            stack.append(current_node.right)

    return closest

#  read .obj file
if USE_3DGS:
    num_gaussians = 10
    gaussians = []
    for _ in range(num_gaussians):
        center = [np.random.uniform(-10, 10) for _ in range(3)]
        color = [np.random.uniform(0, 1) for _ in range(3)]
        opacity = np.random.uniform(0.1, 1)
        scale = np.random.uniform(1, 3)
        gaussian = Gaussian3D(center, color, opacity, scale)
        gaussians.append(gaussian)
    objects = gaussians
else:
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