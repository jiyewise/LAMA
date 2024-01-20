import numpy as np
from IPython import embed
from autoencoder_utils import *
import re
import os

class Skeleton(object):
    def __init__(self, _offset, _parent, _joint):
        self.joint = _joint   # len J
        self.parent = _parent # len J
        self.offset = _offset # [J, 3]
    
    def set_foot_index(self, foot_index):
        self.foot_index = foot_index

class Motion(object):
    def __init__(self, _quats, _root_pos, _offset, _parent, _joint, _timestep):
        self.skel = Skeleton(_offset, _parent, _joint)
        self.quats = _quats         # [# Frames, J, 4]
        self.rootPos = _root_pos    # [# Frames, 1, 3]
        self.motionLength = self.rootPos.shape[0] # Frames
        self.timestep = _timestep

channelmap = {
    'Xrotation': 'x',
    'Yrotation': 'y',
    'Zrotation': 'z'
}

channelmap_inv = {
    'x': 'Xrotation',
    'y': 'Yrotation',
    'z': 'Zrotation',
}

ordermap = {
    'x': 0,
    'y': 1,
    'z': 2,
}

def get_skeleton_info(bvh_file):
    skeleton = read_bvh(bvh_file, read_motion=False)
    # shape of skeleton offset: [num_joints, 3]
    return skeleton.offset, skeleton.parent, len(skeleton.parent)

def motion_from_aa(data, skel_path):
    # return Motion(rotations, root_positions, offsets, parents, names, frametime)
    skel = read_bvh(filename=skel_path, read_motion=False)
    rotations, root_positions = logmap_to_quat_and_root(data)
    return Motion(rotations, root_positions[:,np.newaxis,:], skel.offset, skel.parent, skel.joint, 1/30.0) # 30fps

def read_bvh(filename, start=None, end=None, order=None, read_motion=True):
    f = open(filename, "r")
    i = 0
    active = -1
    end_site = False

    names = []
    orients = np.array([]).reshape((0, 4)) # [J, 4]
    offsets = np.array([]).reshape((0, 3)) # [J, 3]
    parents = np.array([], dtype=int)

    # Parse the  file, line by line
    for line in f:

        if "HIERARCHY" in line: continue
        if "MOTION" in line: continue

        rmatch = re.match(r"ROOT (\w+)", line)
        if rmatch:
            names.append(rmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "{" in line: continue

        if "}" in line:
            if end_site:
                end_site = False
            else:
                active = parents[active]
            continue

        offmatch = re.match(r"\s*OFFSET\s+([\-\d\.e]+)\s+([\-\d\.e]+)\s+([\-\d\.e]+)", line)
        if offmatch:
            if not end_site:
                offsets[active] = np.array([list(map(float, offmatch.groups()))])
            continue

        chanmatch = re.match(r"\s*CHANNELS\s+(\d+)", line)
        if chanmatch:
            channels = int(chanmatch.group(1))
            if order is None:
                if channels == 3:
                    channelStart = 0 
                    channelEnd = 3
                else: # channels == 6
                    channelStart = 3
                    channelEnd = 6
                parts = line.split()[2 + channelStart:2 + channelEnd]
                if any([p not in channelmap for p in parts]):
                    continue
                order = "".join([channelmap[p] for p in parts])
            continue

        jmatch = re.match("\s*JOINT\s+(\w+)", line)
        if jmatch:
            names.append(jmatch.group(1))
            offsets = np.append(offsets, np.array([[0, 0, 0]]), axis=0)
            orients = np.append(orients, np.array([[1, 0, 0, 0]]), axis=0)
            parents = np.append(parents, active)
            active = (len(parents) - 1)
            continue

        if "End Site" in line:
            end_site = True
            continue
        
        fmatch = re.match("\s*Frames:\s+(\d+)", line)
        if fmatch:
            if not read_motion:
                skel = Skeleton(offsets, parents, names)
                f.close()
                return skel
            if start and end:
                fnum = (end - start) - 1
            else:
                fnum = int(fmatch.group(1))
            root_positions = offsets[np.newaxis, 0:1, :].repeat(fnum, axis=0)
            rotations = np.zeros((fnum, len(orients), 3))
            continue

        fmatch = re.match("\s*Frame Time:\s+([\d\.]+)", line)
        if fmatch:
            frametime = float(fmatch.group(1))
            continue

        if (start and end) and (i < start or i >= end - 1):
            i += 1
            continue

        dmatch = line.strip().split(' ')
        if dmatch:
            data_block = np.array(list(map(float, dmatch)))
            N = len(parents)
            fi = i - start if start else i
            if channels == 3:
                root_positions[fi, 0:1] = data_block[0:3]
                rotations[fi, :] = data_block[3:].reshape(N, 3)
            elif channels == 6:
                data_block = data_block.reshape(N, 6)
                root_positions[fi, :] = data_block[0, 0:3]
                rotations[fi, :] = data_block[:, 3:6]
            else:
                raise Exception("This parser supports only 3 or 6 dof for not-root joints!")
            # elif channels == 6:
            #     data_block = data_block.reshape(N, 6)
            #     root_positions[fi, :] = data_block[:, 0:3]
            #     rotations[fi, :] = data_block[:, 3:6]
            # elif channels == 9:
            #     root_positions[fi, 0] = data_block[0:3]
            #     data_block = data_block[3:].reshape(N - 1, 9)
            #     rotations[fi, 1:] = data_block[:, 3:6]
            #     root_positions[fi, 1:] += data_block[:, 0:3] * data_block[:, 6:9]
            # else:
            #     raise Exception("Too many channels! %i" % channels)

            i += 1

    f.close()
    rotations = euler_to_quat(np.radians(rotations), order=order)
    rotations = remove_quat_discontinuities(rotations)

    return Motion(rotations, root_positions, offsets, parents, names, frametime)

def get_num_frame(filename):
    return Motion.motionLength

def convert(filename):
    m = read_bvh(filename)
    logMap = quat_to_logmap(m.quats, m.rootPos.reshape(-1,3))
    return logMap

if __name__ == '__main__':
    directory = "../data/motion/lafan_retarget/aiming1_subject1.bvh"
    m = read_bvh(directory)
    convert(directory)