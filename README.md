# LAMA (ICCV 2023)

## [Project Page](https://jiyewise.github.io/projects/LAMA) &nbsp;|&nbsp; [Paper](https://arxiv.org/pdf/2301.02667.pdf) 

<!-- ![teaser.png](./assets/teaser.png) -->

Author's implementation of Locomotion-Action-Manipulation: Synthesizing Human-Scene Interactions in Complex 3D Environments (ICCV 2023)

## Installation
We checked code works in Ubuntu 20.04.

### Setup

All dependencies can be installed at once by the command below.
```bash
install_total.sh
```
Note: The script includes ``sudo`` commands.
This script download sources on ``c_env`` and ``py_env`` and installs on the same directory. It will take some time to install libraries on the ``c_env`` folder.


### Build
You can build via:
```bash
./run_cmake.sh
cd build
make -j8
```

### Data
1. Download bvh files for the motion synthesizer from the [link](https://drive.google.com/drive/folders/1Aa7JkvIy3t9yzt3g998aw-karaz5yFdS), and place into ``data/motion`` folder.
2. From the link above, download ``autoencoder.pkl`` file and place into ``autoencoder/output/cnn_pretrained/model/`` folder.
3. For the scenes, download 3D scene scans (*.ply files) from the [PROX dataset](https://prox.is.tue.mpg.de/download.php), and place into ``data/scene/prox`` folder.
As the 3D scene meshes have a lot of vertices/faces, rendering the meshes could be a bit slow. You can download downsampled meshes from the PROX dataset or [decimate through MeshLab](https://help.sketchfab.com/hc/en-us/articles/205852789-MeshLab-Decimating-a-model).

## Config
The inputs (action cue, 3D scene, initial starting point) are defined in  ``env/env_config/prox_example.xml`` file.

```
/env
  ├── env_config // includes 3D scene, action cue, initial starting point
/data
  ├── character
  │   └── object // includes xml files for scene meshes (Dummy xml files to format 3D scenes as 6Dof node in DART library)
  │   skel_mxm.xml // skeleton
  ├── scene 
  │   └── prox // includes .ply files of prox 3D scene scans
  └── motion // includes motion database
/result
```
You can adjust hyperparameters/configurations defined in  ``env/EnvConfigurations.h`` file.
## Render
You can render the results by executing ``render`` file in ``build/render``.
Sample motions and pretrained policies are included in this repository. 
#### Running pretrained policy
```bash
cd build
./render/render --type=action_control --env=examples/SCENE_NAME/input${i}.xml --ppo=examples/SCENE_NAME_input${i}/network-0 --dir=SAVE_DIR_NAME
```
* Optimizing to fit into chairs (Sec 3.6, Fig. 10): 
add ``--optimize`` flag to run optimization, and the results would be saved in the ``results/SAVE_DIR_NAME`` folder.

* Or, you can run optimization and save the results via the UI. Results would be saved in ``/results/SAVE_DIR_NAME/``.

#### View saved record (for action / for action + manipulation)
```bash
cd build
# for action
./render/render --type=pp_record --dir=SAVE_DIR_NAME
# for action + manip
./render/render --type=manip_record --dir=MANIP_EXAMPLE   
```
Foot contact and penetration evaluation (Sec. 4.1) are included in ``type=pp_record`` renderer.
Note that we cannot provide object meshes used for manipulation, so to visualize human motion only, use ``type=pp_record`` instead of ``type=manip_record``.
#### View BVH files in the motion database
```bash
cd build
./render/render --type=bvh --bvh=walk/BVH_FILENAME.bvh 
```
#### Run action cue generation UI
```bash
cd build
./render/render --type=int_gen --env=examples/SCENE_NAME/input1.xml # only the 3D scene information are read 
```

## Optimize
####  Running RL for action cue optimization
```bash
cd network
python3 ppo.py --test_name="POLICY_NAME" --env="examples/SCENE_NAME/input${i}" --ntimesteps=350 --nslave=8 # adjust the parameters freely
```

## Citation

If you find the repo useful, please cite:


```
@inproceedings{lee2023lama,
    title = {Locomotion-Action-Manipulation: Synthesizing Human-Scene Interactions in Complex 3D Environments},
    author = {Lee, Jiye and Joo, Hanbyul},
    booktitle = {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
    year = {2023}
}  
```
## Thanks to

- https://github.com/dartsim/dart
- https://github.com/facebookresearch/fairmotion
- https://github.com/syleemrl/Chimera
- https://github.com/syleemrl/ParameterizedMotion
