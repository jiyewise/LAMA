from copy import deepcopy
from matplotlib.pyplot import axes
import torch

# sys.path.insert(0, os.path.dirname(__file__))
# sys.path.append("..")

import argparse
import logging
import numpy as np
import os
import pickle

from IPython import embed
from bvh import *
from autoencoder_utils import *
from fairmotion.core import motion as motion_classes
from fairmotion.ops import conversions
import sys

logging.basicConfig(
	format="[%(asctime)s] %(message)s",
	datefmt="%Y-%m-%d %H:%M:%S",
	level=logging.INFO,
)


def load_data_from_bvh_normalized(base_dir, bvh_list, window=120, offset=10):
	# assume that always list is given for bvh_list. if custom, [custom_bvh] should be given
	assert isinstance(bvh_list, list), "Always a list of bvh filenames should be given. If custom, should be given as [custom_bvh_file] format."
	
	# for seq repr
	norm_Q = []
	X_height = []
	facing_dir_vel = []
	X_vel = []
	contact = []

	# for loss
	X = []
	Q = []

	# yrot_X = []
	# seq_X = [] # [B, S, 1, 3] and [:,0,...] should be [0, height at start of sequence, 0]

	total_num = len(bvh_list)
	for idx, file in enumerate(bvh_list):
		if isinstance(file, np.ndarray):
			skel_path = os.path.join(base_dir, "data/skel.bvh") # TODO later connect to config or so .. 
			m = bvh.load(skel_path, scale=1)
			skel = deepcopy(m.skel)
			rot, rootPos = logmap_to_quat_and_root(file)
			p = np.zeros(rot.shape[:-2] + (3,))
			p[:,0,:] = rootPos
			seq_T = conversions.Rp2T(rot, p)
			motion_output = motion_classes.Motion.from_matrix(seq_T, skel)
			motion_output.set_fps(30)
			bvh.save(
				motion_output,
				filename=os.path.join(base_dir, "data/convert.bvh")
			)
			file = os.path.join(base_dir, "data/convert.bvh")
			# sys.exit(0)

		if isinstance(file, str) and file.endswith('.bvh'):
			print(f"Processing file {idx}/{total_num}",end='\r')
			seq_path = os.path.join(base_dir, file)
			motion = read_bvh(seq_path)

		else:
			continue

		_ , global_p = quat_fk(motion.quats, motion.rootPos, motion.skel.offset, motion.skel.parent)
		c_l, c_r = extract_feet_contacts(global_p, [3,4], [7,8], velfactor=0.05) # [F, 2] for each
		c_lr = np.concatenate((c_l, c_r), axis=-1)  #  [124, 4]

		root_pos = deepcopy(motion.rootPos) # [F, 1, 3]
		joint_Q = deepcopy(motion.quats) # [F, 22, 4]

		# get root position differences in xz coordinates
		root_xz = np.array([1,0,1])*root_pos  # [F, 1, 3]
		root_xz = root_xz.reshape(-1, 3)   # [F, 3]
		velocity = root_xz[1:] - root_xz[:-1]  # [F-1, 3]

		# root height (y)
		height = root_pos[:,0,1][..., np.newaxis]   # [F, 1]

		# for all frames make the root Q face the z axis
		root_Q = deepcopy(joint_Q[:,0,:])   # [F, 4]
		forward = np.array([1, 0, 1])[np.newaxis, np.newaxis, :] \
					* quat_mul_vec(root_Q, np.array([0, 0, 1])[np.newaxis, np.newaxis, :])
		forward = normalize(forward).reshape(-1, 3) # dir is 3 dimensional vector (F, 3)
		facing_rotations = quat_normalize(quat_between(forward, np.array([0,0,1]))) # send forward to z (F, 4)
		joint_Q[:,0,:] = quat_mul(facing_rotations, root_Q) # send to z
		
		velocity = quat_mul_vec(facing_rotations[1:], velocity)
		facing_rot_diff = quat_to_angle_axis(quat_mul(facing_rotations[1:], quat_inv(facing_rotations[:-1])))[..., 1] 
		
		# copy the first frame vel of the velocity array (assume that vel is same at t=0 and t=1)
		velocity = np.concatenate((velocity[0:1], velocity), axis=0) # assume that first axis is frame / shape: [F, 3]
		facing_rot_diff = np.concatenate((facing_rot_diff[0:1], facing_rot_diff), axis=0) # assume that first axis is frame
		facing_rot_diff = facing_rot_diff[..., np.newaxis]   # [F, 1]

		# split into sliding windows
		i = 0
		while True:
			# if the file length is < 120
			if i == 0 and i+window > motion.rootPos.shape[0]:
				length_offset = window - motion.rootPos.shape[0]
				# NOTE all the concatenated velocity should be zero (as root position does not change)
				norm_Q_window = np.concatenate((joint_Q, np.tile(joint_Q[-1:], (length_offset, 1, 1))), axis=0) # [window, J, 4]
				X_height_window = np.concatenate((height, np.tile(height[-1:], (length_offset, 1))), axis=0) # [window, 1]
				facing_dir_vel_window = np.concatenate((facing_rot_diff, np.zeros(shape=(length_offset, 1))), axis=0) # [window, 1]
				X_vel_window = np.concatenate((velocity, np.zeros(shape=(length_offset, 3))), axis=0) # [window, 3]
				contact_window = np.concatenate((c_lr, np.tile(c_lr[-1:], (length_offset, 1))), axis=0) # [window, 4]
				
				# norm_global_p_window = np.concatenate((normalized_global_p, np.tile(normalized_global_p[-1:], (length_offset, 1, 1))), axis=0) # [window, J, 3]
				motion_quats = np.concatenate((motion.quats, np.tile(motion.quats[-1:], (length_offset, 1, 1))), axis=0) # [window, J, 4]
				motion_rootPos = np.concatenate((motion.rootPos, np.tile(motion.rootPos[-1:], (length_offset, 1, 1))), axis=0) # [window, 1, 3]


			elif i+window > motion.rootPos.shape[0]:
				break

			else:
				norm_Q_window = joint_Q[i:i+window]
				X_height_window = height[i:i+window]
				facing_dir_vel_window = facing_rot_diff[i:i+window]
				X_vel_window = velocity[i:i+window]
				contact_window = c_lr[i:i+window]

				motion_quats = motion.quats[i: i+window]
				motion_rootPos = motion.rootPos[i: i+window]

			# record
			norm_Q.append(norm_Q_window)
			X_height.append(X_height_window)
			facing_dir_vel.append(facing_dir_vel_window)
			X_vel.append(X_vel_window)
			contact.append(contact_window)

			# norm_global_p.append(norm_global_p_window)
			X.append(motion_rootPos)
			Q.append(motion_quats)

			i += offset
	
	# into numpy array
	norm_Q = np.asarray(norm_Q)
	X_height = np.asarray(X_height)
	facing_dir_vel = np.asarray(facing_dir_vel)
	X_vel = np.asarray(X_vel)
	contact = np.asarray(contact)

	# norm_global_p = np.asarray(norm_global_p)
	X = np.asarray(X) # [batch, window, 1, 3]
	Q = np.asarray(Q) # [batch, window, J, 4]

	# sequences start at (0,0) facing the z axis
	X_start = X[:,0:1,0:1,::2]
	X[:,:,0:1,0] = X[:,:,0:1,0] - X_start[...,0] # X
	X[:,:,0:1,2] = X[:,:,0:1,2] - X_start[...,1] # Z

	X, Q, global_Q, global_P = rotate_at_frame(X, Q, motion.skel)
	# return normalized Q, height, velocity, facing rot diff, contact -> this forms sequence
	# return for loss: normalized global pos, sequence-based normalized X and Q for root reconstruction loss
	
	# return norm_Q, X_height, facing_dir_vel, X_vel, contact, norm_global_p, seq_X
	return norm_Q, X_height, facing_dir_vel, X_vel, contact, X, Q, global_P


def load_data_with_args(bvh_list, args):
	return load_data(bvh_list, args.base_dir, args.seq_len, args.offset, args.rep)

def load_data(bvh_list, base_dir="", seq_len=120, offset=10, rep="6d"):

	# Get test-set for windows of 120 frames, offset by 10 frames
	norm_Q, X_height, facing_dir_vel, X_vel, contact, X, Q, x_global = load_data_from_bvh_normalized(base_dir, bvh_list, \
																	window=seq_len, offset=offset)		

	x_mean = np.mean(x_global.reshape([x_global.shape[0], x_global.shape[1], -1]).transpose([0, 2, 1]), axis=(0, 2), keepdims=True)
	x_std = np.std(x_global.reshape([x_global.shape[0], x_global.shape[1], -1]).transpose([0, 2, 1]), axis=(0, 2), keepdims=True)

	# define input
	# sequence is a concatenated vector of [contact, root height, root dir vel (angle), root xz velocity, joint rotations (in quaternions)]
	# make into a vector with size (total # of frames, seq_len, -1)

	total, seq, _ , _ = norm_Q.shape

	if rep == "quat":
		Q_view = norm_Q.reshape(total, seq, -1) # (total # of frames, seq, 4J)
		seq = np.concatenate((contact, X_height, facing_dir_vel, X_vel, Q_view), axis=-1)

	elif rep == "6d":
		# test r6d with pytorch3d conversions
		Q_tensor = torch.tensor(norm_Q)
		Rsix = transforms.matrix_to_rotation_6d(transforms.quaternion_to_matrix(Q_tensor)).numpy()
		Rsix_view = Rsix.reshape(total, seq, -1)
		# embed()
		seq = np.concatenate((contact, X_height, facing_dir_vel, X_vel, Rsix_view), axis=-1)

	else:
		assert False, "Representation should be either quat or 6d"

	input_ = {}
	input_['seq'] = seq # shape [total, seq, 4+3+6J] or [total, seq, 4+3+4J]
	input_['global_p'] = x_global # shape [total, seq, J, 3]
	input_['root'] = X # shape [total, seq, 1, 3]
	input_['local_q'] = Q
	input_['contact'] = contact
	input_['x_mean'] = x_mean
	input_['x_std'] = x_std
	input_['rep'] = rep

	return input_ , total

def parse_filenames_and_load(args):

	fnames_list = ['train', 'test', 'validation']
	# fnames_list = ['custom']

	if not os.path.exists(args.preprocess_path):
		os.mkdir(args.preprocess_path)

	for fnames in fnames_list:
		file = open(os.path.join(args.config_path, f'{fnames}_fnames.txt'), 'r')
		filename_list = file.read().split("\n")
		logging.info(f"Start processing {fnames} data...")
		data , total_len = load_data_with_args(filename_list, args)
		logging.info(f"Processed {fnames} data with {total_len} sequences")
		pickle.dump(data, open(os.path.join(args.preprocess_path, f"{fnames}.pkl"), "wb"))


if __name__ == "__main__":

	# data = load_data_from_bvh_normalized("./data/", ["mixamo_interact_mxm/Sit2Type_mxm.bvh"])
	# add argparse
	parser = argparse.ArgumentParser()
	parser.add_argument(
		"--base-dir",
		type=str,
		required=True,
	)
	parser.add_argument(
		"--config-path",
		type=str,
		required=True
	)
	parser.add_argument(
		"--preprocess-path",
		type=str,
		default=None
	)
	parser.add_argument(
		"--seq-len",
		type=int,
		default=120
	)
	parser.add_argument(
		"--offset",
		type=int,
		default=10
	)
	parser.add_argument(
		"--rep",
		type=str,
		default="6d"
	)
	args = parser.parse_args()
	if args.preprocess_path is None:
		args.preprocess_path = os.path.join(args.base_dir, "preprocess_norm_w_jiye")
	
	parse_filenames_and_load(args)