import sys
if not hasattr(sys, 'argv'):
    sys.argv  = ['']
import trimesh
from IPython import embed
import numpy as np
from copy import deepcopy

class TrimeshHelper():
    def __init__(self, _vertices, _faces):
        # self.mesh = trimesh.load(_obj, force='mesh')
        # embed()
        self.mesh_original = trimesh.Trimesh(vertices=_vertices,
                            faces=_faces, use_embree=True)

    def transform(self, _transform):
        # print(_transform)
        self.mesh = deepcopy(self.mesh_original)
        self.mesh = self.mesh.apply_transform(_transform)
        self.all_triangle_center = self.mesh.triangles_center

    def get_vertex(self, _vIndex):
        # embed()
        vertex = self.mesh.vertices[_vIndex] 
        return np.array(vertex).astype(np.float32)

    def check_intersection(self, _ray_origin, _ray_dir):
        locations, index_ray, index_tri = self.mesh.ray.intersects_location(
        ray_origins=_ray_origin[np.newaxis, ...],
        ray_directions=_ray_dir[np.newaxis, ...])

        self.intersect_locations = locations
        self.intersect_tri = index_tri
        if locations.shape[0] > 0:
            return True 
        return False
    
    def get_num_intersection_point(self):
        return self.intersect_locations.shape[0]

    def get_intersection_point(self):
        return np.array(self.intersect_locations).astype(np.float32)
    
    def get_intersection_face(self):
        return np.array(self.intersect_tri).astype(np.float32)
    
    def get_face_center(self, face_num):
        return np.array(self.all_triangle_center[int(face_num)]).astype(np.float32)

    # def build_height_grid(self, _grid_resolution_x, _grid_resolution_z):
    #     self.bbox = self.mesh.bounds
    #     x = self.bbox[:,0]
    #     z = self.bbox[:,2]

    #     grid = np.zeros(shape=(_grid_resolution_x, _grid_resolution_z))
    #     gridsize_x = (x[1]-x[0])/float(_grid_resolution_x)
    #     gridsize_z = (z[1]-z[0])/float(_grid_resolution_z)
        
    #     # bbox in [2,3] shape. bbox[]
    #     # embed()
    #     for v in self.mesh.vertices:
    #         grid_x = int((v[0]-x[0])/gridsize_x)
    #         grid_z = int((v[2]-z[0])/gridsize_z)

    #         grid_x = max(0, min(grid_x, _grid_resolution_x-1))
    #         grid_z = max(0, min(grid_z, _grid_resolution_z-1))

    #         if grid[grid_x, grid_z] > 0:
    #             continue
    #         if v[1] > 0.15 and v[1] < 1.2:
    #             grid[grid_x, grid_z] = 1
        
    #     self.grid = grid.astype(np.float32)
    #     return self.grid
    
    # def get_grid(self):
    #     return self.grid

    # def get_bbox(self):
    #     return self.bbox.astype(np.float32)