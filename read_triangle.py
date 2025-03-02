
from triangle import Triangle

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