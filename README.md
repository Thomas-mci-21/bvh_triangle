# Ray Tracing Renderer with BVH Acceleration
This project implements a simple ray tracer using a Bounding Volume Hierarchy (BVH) tree for efficient ray-object intersection tests. The ray tracer reads a 3D model from an .obj file, builds a BVH tree, and renders the scene using ray tracing.

## Features
BVH Acceleration: The BVH tree is used to speed up ray-object intersection tests by hierarchically organizing the objects in the scene.

Triangle Intersection: The Möller–Trumbore algorithm is used to determine if a ray intersects with a triangle.

OBJ File Support: The ray tracer can read 3D models from .obj files and render them.

## Usage
Install Dependencies: Ensure you have numpy and matplotlib installed.

```pip install numpy matplotlib```

Run the Ray Tracer:
Place your .obj file in the models/bunny/ directory (or modify the file_path variable to point to your file).

Run the script:
```python3 ray_tracer.py```

View the Output: The rendered image will be displayed using matplotlib.

## Code Overview

AABB Class: Represents an Axis-Aligned Bounding Box (AABB) and provides methods for ray-AABB intersection tests.

Triangle Class: Represents a triangle and provides methods for ray-triangle intersection using the Möller–Trumbore algorithm.

BVHNode Class: Represents a node in the BVH tree, which can be either a leaf node (containing objects) or an internal node (containing child nodes).

build_bvh Function: Constructs the BVH tree by recursively partitioning the objects based on their AABBs **using stack alogrithm**.

trace_ray Function: Traces a ray through the BVH tree to find the closest intersection with an object **using stack alogrithm**.

read_obj_file Function: Reads a 3D model from an .obj file and creates a list of triangles.

## 2025.2.28 update

1. Use bunny.obj (2503 vertices and 4968 triangles) in GAMES101 Assignment 6 as the rendering target

2. Replaced recursion with stack algorithm in BVH Tree buiding & traversing

3. Renamed AABB_intersect & triangle_intersect to help distinguish 

4. Replaced complex np functions with math

