Download 3D scene scans (*.ply files) from the [PROX dataset](https://prox.is.tue.mpg.de/download.php), and place into ``data/scene/prox`` folder.
As the 3D scene meshes have a lot of vertices/faces, rendering the meshes could be a bit slow. You can download downsampled meshes from the PROX dataset or [decimate through MeshLab](https://help.sketchfab.com/hc/en-us/articles/205852789-MeshLab-Decimating-a-model).
```
/data
  ├── character
  │   └── object // includes xml files for scene meshes (Dummy xml files to format 3D scenes as 6Dof node in DART library)
  │   skel_mxm.xml // skeleton
  ├── scene 
  │   └── prox // includes .ply files of prox 3D scene scans
  └── motion // includes motion database
/result
```