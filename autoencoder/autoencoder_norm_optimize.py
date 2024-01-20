import argparse
from copy import deepcopy
from email.mime import base
import logging
from select import select
# from black import out
import numpy as np
import os
import random
import torch
import torch.nn as nn
import torch.optim as optim
from IPython import embed 
from model import Conv3AutoEncoder, ConvAutoencoder, Seq2Seq, LSTMDecoder, LSTMEncoder
from bvh import *
import yaml
from dataset_norm import *
from autoencoder_utils import *
from pytorch3d import transforms
from autoencoder_norm import RenderData

from tensorboardX import SummaryWriter

logging.basicConfig(
	format="[%(asctime)s] %(message)s",
	datefmt="%Y-%m-%d %H:%M:%S",
	level=logging.INFO,
)


class LatentOptimizeConv(nn.Module):
	def __init__(self):
		super(LatentOptimizeConv, self).__init__()

	def set_params(self, latent):
		self.latent = nn.Parameter(latent)

	def forward(self, model):
		# the model is freezed
		output = model.decode(self.latent)
		return output

class LatentOptimizeLSTM(nn.Module):
	def __init__(self):
		super(LatentOptimizeLSTM, self).__init__()

	def set_params_conv(self, latent):
		self.latent = nn.Parameter(latent)

	def set_params(self, hidden, cell):
		self.hidden = nn.Parameter(hidden)
		self.cell = nn.Parameter(cell) 		

	def forward(self, model, tgt, max_len=None):
		# the model is freezed
		output = model.decode(tgt, self.hidden, self.cell, max_len=max_len)
		return output

def set_seeds():
	torch.manual_seed(1)
	np.random.seed(1)
	random.seed(1)
	torch.backends.cudnn.deterministic = True
	torch.backends.cudnn.benchmark = False


class AutoEncoder(object):
	def __init__(self, base_dir="./"):
		self.optimize = False
		self.base_dir = base_dir
		return

	def set_info(self, directory):
		self.directory = directory + "/"
		self.device = "cuda" if torch.cuda.is_available() else "cpu"
		logging.info(f"Using device: {self.device}")

		# open config file
		config_dir = self.directory + "config.yaml"
		self.config = yaml.safe_load(open(config_dir, 'r').read())
		
		# set information from config and args
		self.rep = self.config['data']['rep']
		self.save_frequency = self.config['train']['save_frequency']
		self.set_skel_info() # load skeleton info

		# create or set log and model directory
		self.log_dir = os.path.join(directory, "log/")
		self.model_dir = os.path.join(directory, "model/")

		self.axis_y = torch.from_numpy(np.tile(np.array([0.0,1.0,0.0]), (1,1))).to(self.device).unsqueeze(1)
		self.rotmat_id_tensor = torch.from_numpy(np.eye(3)[np.newaxis,...]).to(self.device).repeat(1,1,1)

	def set_skel_info(self):
		self.skel_offset, self.skel_parent, self.num_joints = get_skeleton_info(os.path.join(self.base_dir, self.config['skel']['bvh']))
		self.skel_offset = torch.from_numpy(self.skel_offset[np.newaxis, np.newaxis, ...]).to(self.device) 		# expand skel offset into tensor

	def load_custom_data(self, custom):
		self.dataloader = {}
		data = np.load(self.directory + "mean_and_std.npz")
		self.mean = data['mean']
		self.std = data['std']

		self.dataloader = {}
		self.bvh_name = ""
		if isinstance(custom, str):
			self.dataloader['custom'] = get_loader(dataset_path=custom, \
											batch_size=1, \
											seq_len=self.config['data']['seq_length'], \
											device=self.device, \
											mean = self.mean, \
											std = self.std,
											shuffle=False, drop_last=False)
			self.bvh_name = custom.strip().split("/")[-1][:-4]

		elif isinstance(custom, np.ndarray):
			self.dataloader['custom'] = get_loader_from_data(data=custom, \
											base_dir=self.base_dir,\
											batch_size=1, \
											seq_len=self.config['data']['seq_length'], \
											device=self.device, \
											mean = self.mean, \
											std = self.std,
											shuffle=False, drop_last=False)

		self.mean = torch.from_numpy(self.mean).to(self.device)
		self.std = torch.from_numpy(self.std).to(self.device)

	def load_target(self, custom_target):
		self.targets = {}
		for i in range(self.config['data']['seq_length']):
			self.targets[i] = []
		num_target, _ = custom_target.shape
		for i in range(num_target):
			frame = int(custom_target[i][0])
			if frame >= self.config['data']['seq_length']:
				frame = self.config['data']['seq_length']-1
			joint = int(custom_target[i][1])
			target_pos = custom_target[i][2:5]*100
			target_ori = conversions.A2R(custom_target[i][5:])
			self.targets[frame].append((joint, torch.from_numpy(target_pos).to(self.device), torch.from_numpy(target_ori).to(self.device)))

	def init_optimize(self, directory, custom, custom_target=None, bm_info=None):
		self.set_info(directory)
		self.load_custom_data(custom=custom)
		
		self.targets = None
		if custom_target is not None:
			self.load_target(custom_target)
		
		self.bm = None

		self.latent_module = LatentOptimizeLSTM().to(self.device) if self.config['model'] == "lstm" else LatentOptimizeConv().to(self.device)
		self.build_network()

		self.optimize = True
		self.init_latent_optimize_setting()
		self.build_latent_optimizer()

		

	def build_network(self):
		seq_dim = self.dataloader['custom'].dataset.get_seq_dim()
		logging.info("Preparing model...")

		if self.config['model'] == "lstm":
			enc = LSTMEncoder(
				input_dim=seq_dim, hidden_dim=self.config['train']['lstm_hidden_dim']
			).to(self.device)
			dec = LSTMDecoder(
				input_dim=seq_dim,
				hidden_dim=self.config['train']['lstm_hidden_dim'],
				output_dim=seq_dim,
				device=self.device,
			).to(self.device)

			self.model = Seq2Seq(enc, dec)
		
		elif self.config['model'] == "conv":
			self.model = ConvAutoencoder(seq_dim)

		elif self.config['model'] == "conv3":
			self.model = Conv3AutoEncoder(seq_dim, self.config['train']['conv_channels'])

		self.model = self.model.to(self.device)
		self.model.zero_grad()
		self.model.double()

		self.criterion = nn.MSELoss()
		self.model.load_state_dict(torch.load(os.path.join(self.model_dir, 'autoencoder.pkl'), map_location=self.device))
		logging.info("autoencoder model loaded")


	def build_latent_optimizer(self):
		logging.info("Preparing latent optimizer...")
		self.optimizer = optim.Adam(self.latent_module.parameters(), lr=0.005, \
									# betas = (self.config['train']['beta1'], self.config['train']['beta2']), \
									weight_decay = self.config['train']['weight_decay']) # default is 0.0001

	def init_latent_optimize_setting(self):
		logging.info("Initialize latent optimization...")
		# get initial hidden, cell values
		sampled_batch = next(iter(self.dataloader['custom']))
		self.src_seq = sampled_batch['src_seq'].to(self.device)
		self.tgt_seq = sampled_batch['tgt_seq'].to(self.device) # [batch, seq_len, dim]

		self.tgt_quat = sampled_batch['local_q'].to(self.device)
		self.tgt_root = sampled_batch['root'][...,0,:].to(self.device)
		self.tgt_contact = sampled_batch['contact'].to(self.device)
		self.tgt_foot = sampled_batch['global_p'][..., [3,4,7,8],:].to(self.device)

		# freeze model and run encoder
		for param in self.model.parameters():
			param.requires_grad = False
		
		if self.config['model'] == "lstm":
			hidden, cell = self.model.encode(self.src_seq)
			# set parameters
			self.latent_module.set_params(hidden, cell)

		elif "conv" in self.config['model']:
			self.model.eval()
			latent = self.model.encode(self.src_seq)
			self.latent_module.set_params(latent)

		
	def latent_optimize(self, use_root_vel=True, epoch_num=500):
		logging.info("Latent optimization...")
		torch.autograd.set_detect_anomaly(True)
		self.render_data = []

		# feet idx: [3,4] [7,8]
		for epoch in range(500):
			# decode with set hidden, cell value 
			if self.config['model'] == "lstm":
				max_len = self.tgt_seq.shape[1]
				tgt_frame = self.src_seq[:,0].unsqueeze(1)
				outputs = self.latent_module(self.model, tgt_frame, max_len)
			elif "conv" in self.config['model']:
				outputs = self.latent_module(self.model)
			outputs.double()
			batch, seq, _ = outputs.shape

			# denormalize
			skel = self.skel_offset.repeat(batch,seq,1,1)
			denorm_root_pos_pred, denorm_rotations_pred_ = self.denorm_and_recon(outputs)
			if denorm_rotations_pred_.shape[-1] == 3:
				output_pos = rot_matrix_fk_tensor(denorm_rotations_pred_, denorm_root_pos_pred, skel, self.skel_parent)
			else:
				output_pos = quat_fk_tensor(denorm_rotations_pred_ , denorm_root_pos_pred, skel, self.skel_parent) # FK	

			# compare with target and get losses
			target_loss = torch.tensor(0.0).to(self.device)
			if self.targets is not None:
				target_count = 0
				for target_frames in self.targets:
					if len(self.targets[target_frames]) == 0:
						continue
					for target_tuple in self.targets[target_frames]:					
					# target_tuple = self.targets[target_frames]
						target_pos = target_tuple[1]
						cur = output_pos[:,target_frames,target_tuple[0]]
						target_loss += torch.sqrt(torch.sum(torch.abs(cur-target_pos)**2)) 
						target_count += 1
						
						# for root, match orientation also
						if target_tuple[0] == 0:
							cur_root_ori = denorm_rotations_pred_[:,target_frames,target_tuple[0]]
							target_loss += torch.sqrt(torch.sum(torch.abs(cur_root_ori - target_tuple[2])**2))
				
				target_loss /= float(target_count)

			# contact loss
			self.criterion = nn.MSELoss()
			foot_diff = torch.abs(self.tgt_foot - output_pos[...,[3,4,7,8],:])
			contact_loss = torch.mean(foot_diff)

			# root velocity loss
			tgt_root_vel = self.tgt_root[0,1:] - self.tgt_root[0,:-1]
			output_root_vel = denorm_root_pos_pred[0,1:] - denorm_root_pos_pred[0,:-1]

			# optimizer step
			self.optimizer.zero_grad()
			if use_root_vel:
				root_vel_loss = self.criterion(tgt_root_vel, output_root_vel)
				self.loss_total = target_loss*3 + contact_loss*5 + root_vel_loss*10
			else:
				root_vel_loss = self.criterion(tgt_root_vel, output_root_vel)
				root_loss_x = self.criterion(self.tgt_root[...,0], denorm_root_pos_pred[...,0])
				root_loss_z = self.criterion(self.tgt_root[...,2], denorm_root_pos_pred[...,2])
				self.loss_total = target_loss*7 + contact_loss*6 + root_loss_x + root_loss_z + root_vel_loss*10
				root_vel_loss += root_loss_x + root_loss_z
			
			self.loss_total.backward()
			torch.nn.utils.clip_grad_norm_(self.latent_module.parameters(), 0.5)
			self.optimizer.step()

			logging.info(f"current epoch: {epoch} current target loss : {target_loss} contact loss : {contact_loss} root vel loss: {root_vel_loss}")


		# done, save to bvh
		rd = RenderData(gt_root=self.tgt_root[0].detach().cpu().numpy(), \
						gt_rot=self.tgt_quat[0].detach().cpu().numpy(), \
						output_root=denorm_root_pos_pred[0].detach().cpu().numpy(), \
						output_rot=denorm_rotations_pred_[0].detach().cpu().numpy())
		if self.bvh_name != "":
			rd.convert_to_matrix()
		else:
			rd.convert_to_logmap()
		self.render_data.append(rd)

		if self.bvh_name != "":
			write_result(self.render_data, os.path.join(self.base_dir, self.config['skel']['bvh']), self.directory, f"optimize/{self.bvh_name}/", 0)			
		

	def get_logmap(self, idx):
		assert len(self.render_data) > 0, "render data list is empty"
		assert idx < len(self.render_data), "idx should be smaller than render data list size"

		return self.render_data[len(self.render_data)-1].logmap

	def denorm_and_recon(self, outputs):
		# denorm output and perform FK
		batch, seq_len, _ = outputs.shape
		denorm_seq = denormalize(outputs, self.mean, self.std, self.device)

		root_height = denorm_seq[...,4:5]	
		facing_dir_vel = denorm_seq[...,5:6]
		root_xz_vel = denorm_seq[..., 6:9]	
		Q = denorm_seq[..., 9:].reshape(batch, seq_len, -1, 6)

		# norm_Q = transforms.matrix_to_quaternion(transforms.rotation_6d_to_matrix(Q))
		norm_rotmat = transforms.rotation_6d_to_matrix(Q)

		facing_dir_aa = self.axis_y * facing_dir_vel

		norm_rotmat_list = []
		root_pos_rotmat = []

		for i in range(seq_len):
			if i == 0:
				facing_rotation_mat, starting_pos_mat = self.rotmat_id_tensor, (self.axis_y.squeeze(-2) * root_height[:,0,:]).clone()
			else:
				rot_diff_mat = transforms.axis_angle_to_matrix(facing_dir_aa[:,i])
				facing_rotation_mat = rot_diff_mat @ facing_rotation_mat
				starting_pos_mat += (facing_rotation_mat.swapaxes(-2,-1) @ root_xz_vel[:,i,:].unsqueeze(-1)).squeeze(-1)
				starting_pos_mat[:,1] = (root_height[:,i,0]).clone()

			# record
			root_rotmat = facing_rotation_mat.swapaxes(-2,-1) @ norm_rotmat[:, i, 0, ...]
			frame_rotmat = torch.concat((root_rotmat.unsqueeze(1),norm_rotmat[:,i,1:,:]), dim=1) # [batch, joint, 3, 3]
			norm_rotmat_list.append(frame_rotmat)
			root_pos_rotmat.append(starting_pos_mat.clone())

		rotmat_stack = torch.stack(norm_rotmat_list, dim=1)
		root_pos_rotmat_stack = torch.stack(root_pos_rotmat, dim=1)

		return root_pos_rotmat_stack, rotmat_stack
