# Copyright (c) Facebook, Inc. and its affiliates.

import numpy as np
import pickle
import torch
from fairmotion.utils import constants
from IPython import embed
from preprocess_norm import *
from pytorch3d import transforms

from torch.utils.data import Dataset, DataLoader

class MotionData(Dataset):
	def __init__(self, dataset_path="", device="cuda", data=None, base_dir="", seq_len=120, mean=None, std=None, debug=False):
		"""
		Args:
			bvh_path (string): Path to the bvh files.
			seq_len (int): The max len of the sequence for interpolation.
		"""

		if 'pkl' in dataset_path:
			self.data = pickle.load(open(dataset_path, "rb"))

		elif dataset_path != "":
			self.data, _ = load_data([dataset_path], seq_len=seq_len)

		elif data is not None:
			self.data, _ = load_data([data], base_dir=base_dir, seq_len=seq_len)
		
		self.num_frames, self.seq_len, self.seq_dim = self.data['seq'].shape
		self.debug = debug 

		# normalize
		if mean is None or std is None:
			self.mean = np.mean(self.data['seq'], axis=(0,1))
			self.std = np.std(self.data['seq'], axis=(0,1))
		else:
			self.mean = mean
			self.std = std
		
		self.device = device

	def __len__(self):
		return self.num_frames

	def __getitem__(self, idx):
		idx_ = None
		if self.debug:
			idx_ = 0
		else:
			idx_ = idx

		norm_seq = (self.data['seq'][idx_] - self.mean) / (
			self.std + constants.EPSILON
		)

		sample = {}
		sample['src_seq'] = norm_seq 
		sample['tgt_seq'] = norm_seq

		sample['global_p'] = self.data['global_p'][idx_]
		sample['root'] = self.data['root'][idx_]
		
		sample['local_q'] = self.data['local_q'][idx_]
		sample['contact'] = self.data['contact'][idx_]
		
		return sample


	def get_x_mean_and_std(self):
		return self.data['x_mean'], self.data['x_std']

	def get_seq_dim(self):
		return self.seq_dim
	
	def get_seq_length(self):
		return self.seq_len

def get_loader_from_data(
	data,
	base_dir="",
	seq_len=80,
	batch_size=100,
	device="cuda",
	mean=None,
	std=None,
	shuffle=False,
	drop_last=True
):
	"""Returns data loader for custom dataset.
	Args:
		dataset_path: path to pickled numpy dataset
		device: Device in which data is loaded -- 'cpu' or 'cuda'
		batch_size: mini-batch size.
	Returns:
		data_loader: data loader for custom dataset.
	"""
	# build a custom dataset
	dataset = MotionData(data=data, base_dir=base_dir, device=device, mean=mean, std=std, seq_len=seq_len)

	# data loader for custom dataset
	# this will return (src_seqs, tgt_seqs) for each iteration
	data_loader = DataLoader(
		dataset=dataset, batch_size=batch_size, shuffle=shuffle, num_workers=8,  drop_last=drop_last
	)
	return data_loader

def get_loader(
	dataset_path,
	seq_len=80,
	batch_size=100,
	device="cuda",
	mean=None,
	std=None,
	shuffle=False,
	drop_last=True
):
	"""Returns data loader for custom dataset.
	Args:
		dataset_path: path to pickled numpy dataset
		device: Device in which data is loaded -- 'cpu' or 'cuda'
		batch_size: mini-batch size.
	Returns:
		data_loader: data loader for custom dataset.
	"""
	# build a custom dataset
	dataset = MotionData(dataset_path=dataset_path, device=device, mean=mean, std=std, seq_len=seq_len)

	# data loader for custom dataset
	# this will return (src_seqs, tgt_seqs) for each iteration
	data_loader = DataLoader(
		dataset=dataset, batch_size=batch_size, shuffle=shuffle, num_workers=8,  drop_last=drop_last
	)
	return data_loader

if __name__=="__main__":
	fnames = ['custom']
	datasets = {}
	for fname in fnames:
		datasets[fnames] = MotionData(f"./data/preprocess_norm/{fname}.pkl", "cpu")
	# lafan_data = MotionData('./data/lafan_mxm/', device="cpu")