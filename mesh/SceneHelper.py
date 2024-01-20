import sys
if not hasattr(sys, 'argv'):
    sys.argv  = ['']
from IPython import embed
import numpy as np
from copy import deepcopy
import pyembree
import trimesh

class SceneHelper(object):
    def __init__(self):
        self.meshes = []

    def add_mesh(self, mesh):
        self.meshes.append(mesh.mesh)

    def clear_all_mesh(self):
        self.meshes = []
    
    def integrate_all_mesh(self):
        self.scene = trimesh.util.concatenate(self.meshes)

    # def load_sdf(self):
    #     self.scene_sdf = SDF(self.scene.vertices, self.scene.faces)

    # def load_scene_from_meshes(self):
    #     self.integrate_all_mesh()
    #     self.load_sdf()        
    
    # def check_point_in_mesh(self, points):
    #     contained = self.scene_sdf.contains(points)
    #     return contained.astype(np.float32)

    def scene_check_intersection(self, _ray_origin, _ray_dir):
        # print("start scene check intersection in python")
        locations, index_ray, index_tri = self.scene.ray.intersects_location(
        ray_origins=_ray_origin,
        ray_directions=_ray_dir)

        # locations, index_ray, index_tri = self.scene.ray.intersects_location(
        # ray_origins=_ray_origin,
        # ray_directions=_ray_dir)

        self.intersect_locations = locations
        self.intersect_tri = index_tri
        if locations.shape[0] > 0:
            return True 
        return False
    
    def get_num_intersection_point(self):
        return self.intersect_locations.shape[0]

    def get_intersection_point(self):
        return np.array(self.intersect_locations).astype(np.float32)

    def build_height_grid(self, _grid_resolution_x, _grid_resolution_z):
        self.bbox = self.scene.bounds
        x = self.bbox[:,0]
        z = self.bbox[:,2]
        # embed()

        grid = np.zeros(shape=(_grid_resolution_x, _grid_resolution_z))
        gridsize_x = (x[1]-x[0])/float(_grid_resolution_x)
        gridsize_z = (z[1]-z[0])/float(_grid_resolution_z)
        
        for v in self.scene.vertices:
            grid_x = int((v[0]-x[0])/gridsize_x)
            grid_z = int((v[2]-z[0])/gridsize_z)

            grid_x = max(0, min(grid_x, _grid_resolution_x-1))
            grid_z = max(0, min(grid_z, _grid_resolution_z-1))

            if grid[grid_x, grid_z] > 0:
                continue
            if v[1] > 0.15 and v[1] < 1.2:
                grid[grid_x, grid_z] = v[1]
        
        self.grid = grid.astype(np.float32)
        return self.grid
    
    def get_grid(self):
        return self.grid

    def get_bbox(self):
        return self.bbox.astype(np.float32)