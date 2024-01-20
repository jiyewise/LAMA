import argparse
from copy import deepcopy
import logging
from select import select
from fairmotion.ops import conversions
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

from tensorboardX import SummaryWriter

logging.basicConfig(
	format="[%(asctime)s] %(message)s",
	datefmt="%Y-%m-%d %H:%M:%S",
	level=logging.INFO,
)


def set_seeds():
	torch.manual_seed(1)
	np.random.seed(1)
	random.seed(1)
	torch.backends.cudnn.deterministic = True
	torch.backends.cudnn.benchmark = False


class RenderData(object):
	def __init__(self, gt_root, gt_rot, output_root, output_rot, use_gt=True):
		self.output_root = output_root # [seq_len, 3]
		self.output_rot = output_rot # [seq_len, J, 4]
		if use_gt:
			self.gt_root = gt_root # [seq_len, 3]
			self.gt_rot = gt_rot # [seq_len, J, 4]

	def convert_to_T(self, rot, root):
		if rot.shape[-1] == 3:
			return rotmat_to_T(rot, root)
		else:
			assert rot.shape[-1] == 4, "rotations should be either rotation matrices or quaternions"
			return quat_to_T(rot, root)

	def convert_to_matrix(self, scale=1, use_gt=True):
		self.output_root *= scale
		self.output_T = self.convert_to_T(self.output_rot, self.output_root)
		if use_gt:
			self.gt_root *= scale
			self.gt_T = self.convert_to_T(self.gt_rot, self.gt_root)
	
	def convert_to_logmap(self, scale=0.01, use_gt=False):
		self.output_root *= scale 
		aa = conversions.R2A(self.output_rot)
		frame, joint, _ = aa.shape
		aa = aa.reshape(frame, -1)
		self.logmap = np.concatenate((self.output_root, aa), axis=-1, dtype=np.float32)
		
		

class AutoEncoder(object):
	def __init__(self):
		# set_seeds()
		self.custom = False
		return

	def set_info(self, directory, config="", pretrain=False, is_train=True):
		self.directory = directory + "/"
		self.pretrain = pretrain if is_train else True
		self.device = "cuda" if torch.cuda.is_available() else "cpu"
		logging.info(f"Using device: {self.device}")

		# open config file
		config_dir = "./config/"+config + ".yaml" if is_train else self.directory + "config.yaml"
		self.config = yaml.safe_load(open(config_dir, 'r').read())
		
		# set information from config and args
		self.rep = self.config['data']['rep']
		self.save_frequency = self.config['train']['save_frequency']
		self.set_skel_info() # load skeleton info

		# create or set log and model directory
		if not os.path.exists(directory + "log/"):
				os.mkdir(directory + "log/")
		self.log_dir = directory +"log/"
		if not os.path.exists(directory + "model/"):
			os.mkdir(directory + "model/")
		
		self.log_dir = directory +"log/"
		self.model_dir = directory +"model/"

		if is_train:
			self.skel_offset = self.skel_offset.repeat(self.config['train']['batch_size'], self.config['data']['seq_length'],1,1)
			os.system('cp {} {}'.format('./config/'+config+'.yaml', directory+'config.yaml'))


	def set_skel_info(self):
		self.skel_offset, self.skel_parent, self.num_joints = get_skeleton_info(self.config['skel']['bvh'])
		self.skel_offset = torch.from_numpy(self.skel_offset[np.newaxis, np.newaxis, ...]).to(self.device) 		# expand skel offset into tensor

	
	def load_data(self, preprocess_directory, batch_size, is_train=True):
		fnames = ['train', 'validation'] if is_train else ['train', 'test']
		self.dataloader = {}
		
		if is_train is False:
			data = np.load(self.directory + "mean_and_std.npz")
			self.mean = data['mean']
			self.std = data['std']

		for fname in fnames:
			if fname == "train":
				self.dataloader[fname] = get_loader(dataset_path=os.path.join(preprocess_directory, f"{fname}.pkl"), \
												batch_size=batch_size, \
												device=self.device, \
												shuffle=is_train,
												drop_last=True)
				self.mean = self.dataloader['train'].dataset.mean
				self.std = self.dataloader['train'].dataset.std 
				np.savez(self.directory+"mean_and_std", mean=self.mean, std=self.std)

			else:
				self.dataloader[fname] = get_loader(dataset_path=os.path.join(preprocess_directory, f"{fname}.pkl"), \
												batch_size=batch_size, \
												device=self.device, \
												mean = self.mean, \
												std = self.std,
												shuffle=is_train, drop_last=False)

		self.x_mean, self.x_std = self.dataloader['train'].dataset.get_x_mean_and_std() 

		# convert to tensor for future calculations
		self.mean = torch.from_numpy(self.mean).to(self.device)
		self.std = torch.from_numpy(self.std).to(self.device)
		self.x_mean = torch.from_numpy(self.x_mean).to(self.device)
		self.x_std = torch.from_numpy(self.x_std).to(self.device).view(1, 1, self.num_joints, 3)

	def load_custom_data(self, preprocess_directory, custom_bvh_path):
		self.dataloader = {}
		data = np.load(self.directory + "mean_and_std.npz")
		self.mean = data['mean']
		self.std = data['std']

		fnames = ['train', 'custom']
		self.dataloader = {}
		
		for fname in fnames:
			if fname == "train":
				self.dataloader[fname] = get_loader(dataset_path=os.path.join(preprocess_directory, f"{fname}.pkl"), \
												batch_size=self.config['train']['batch_size'], \
												device=self.device, \
												shuffle=True,
												drop_last=True)
				self.mean = self.dataloader['train'].dataset.mean
				self.std = self.dataloader['train'].dataset.std 
				np.savez(self.directory+"mean_and_std", mean=self.mean, std=self.std)

			else:
				self.dataloader['custom'] = get_loader(dataset_path=custom_bvh_path, \
												batch_size=1, \
												seq_len=self.config['data']['seq_length'], \
												device=self.device, \
												mean = self.mean, \
												std = self.std,
												shuffle=False, drop_last=False)


		self.x_mean, self.x_std = self.dataloader['train'].dataset.get_x_mean_and_std() 

		self.mean = torch.from_numpy(self.mean).to(self.device)
		self.std = torch.from_numpy(self.std).to(self.device)
		self.x_mean = torch.from_numpy(self.x_mean).to(self.device)
		self.x_std = torch.from_numpy(self.x_std).to(self.device).view(1, 1, self.num_joints, 3)

		self.bvh_name = custom_bvh_path.strip().split("/")[-1][:-4]

	def init_train(self, directory, config, pretrain=False):
			
		self.set_info(directory, config=config, pretrain=pretrain, is_train=True)
		self.load_data(preprocess_directory=self.config['data']['preprocess'], batch_size=self.config['train']['batch_size'], is_train=True)
	
		self.build_network() 
		self.build_optimizer()
	
	def init_test(self, directory):
		
		self.set_info(directory, is_train=False)
		self.load_data(preprocess_directory=self.config['data']['preprocess'], batch_size=self.config['test']['batch_size'], is_train=False)

		self.build_network()
		self.build_optimizer()

	def init_custom(self, directory, custom_bvh_path):
		self.custom = True
		self.set_info(directory, is_train=False)
		self.load_custom_data(preprocess_directory=self.config['data']['preprocess'], custom_bvh_path=custom_bvh_path)

		self.build_network()
		self.build_optimizer()

	def build_network(self):
		seq_dim = self.dataloader['train'].dataset.get_seq_dim()
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
			if "conv_dropout" in self.config['train']:
				self.model = Conv3AutoEncoder(seq_dim, self.config['train']['conv_channels'], dropout=self.config['train']['conv_dropout'])
			else:
				self.model = Conv3AutoEncoder(seq_dim, self.config['train']['conv_channels'])

		self.model = self.model.to(self.device)
		self.model.zero_grad()
		self.model.double()

		self.criterion = nn.MSELoss()
		if self.config['model'] == "lstm":
			self.model.init_weights()

		if self.pretrain:
			self.model.load_state_dict(torch.load(os.path.join(self.model_dir, 'autoencoder.pkl')))
			logging.info("autoencoder model loaded")


	def build_optimizer(self):
		logging.info("Preparing optimizer...")

		if self.config['model'] == "lstm":
			self.optimizer = optim.Adam(self.model.parameters(), lr=self.config['train']['lr'], \
										betas = (self.config['train']['beta1'], self.config['train']['beta2']), \
										weight_decay = self.config['train']['weight_decay']) # default is 0.0001

		elif "conv" in self.config['model']:
			self.optimizer = optim.Adam(self.model.parameters(), lr=self.config['train']['lr'], weight_decay=self.config['train']['weight_decay'])

		self.scheduler = optim.lr_scheduler.StepLR(self.optimizer, step_size=150, gamma=self.config['train']['lr_decay'])

		if self.pretrain:
			self.optimizer.load_state_dict(torch.load(os.path.join(self.model_dir + 'optimizer.pkl')))
			logging.info("optimizer loaded")


	def run(self, mode="test"):
		logging.info("Testing model...")

		self.model.eval()
		render_data = []
		epoch_loss = 0
		steps_per_epoch = len(self.dataloader[mode])

		is_test = False if mode == "validation" else True
		if mode == "custom":
			batch = 1
		else:
			batch = self.config['test']['batch_size']
		if is_test:
			self.skel_offset = self.skel_offset.repeat(batch, self.config['data']['seq_length'],1,1)

		self.teacher_forcing_ratio = 0
		for iterations, sampled_batch in enumerate(self.dataloader[mode]):
			with torch.no_grad():
				src_seq = sampled_batch['src_seq'].to(self.device)
				tgt_seq = sampled_batch['tgt_seq'].to(self.device)
				global_pos = sampled_batch['global_p'].to(self.device)
				root = sampled_batch['root'].to(self.device)
				tgt_frame = src_seq[:,0].unsqueeze(1) # as the autoencoder does reconstruction, first frame of the sequence should be fed
				max_len = tgt_seq.shape[1] # for eval, the target sequence length is 1, so we have to specify the max len
				if self.config['model'] == "lstm":
					outputs = self.model(
						src_seq, tgt_frame, max_len=max_len, teacher_forcing_ratio=0
					)
				elif "conv" in self.config['model']:
					outputs = self.model(src_seq)
				outputs = outputs.double()
				results = self.denorm_and_get_loss_norm(outputs, tgt_seq, global_pos, root, is_test) # (denorm_root_pos_pred, denorm_rotations_pred_)

				batch,seq, _ = outputs.shape
				if results is not None:
					select_idx = 10
					if select_idx >= batch:
						select_idx = batch-1
					tgt_root = sampled_batch['global_p'][...,0,:]
					tgt_rotations = sampled_batch['local_q']

				
					rd = RenderData(gt_root=tgt_root[select_idx], \
					gt_rot=tgt_rotations[select_idx].detach().cpu().numpy(), \
					output_root=results[0][select_idx].detach().cpu().numpy(),\
					output_rot=results[1][select_idx].detach().cpu().numpy())

					rd.convert_to_matrix()
					render_data.append(rd)
			if is_test:
				print(f"{iterations} iteration done")
			epoch_loss += self.loss_total.item()
		
		epoch_loss /= steps_per_epoch
		logging.info(
			f"Test mode: {mode} | "
			f"{mode} loss: {epoch_loss}"
		)
		if mode == "test":
			write_result(render_data, self.config['skel']['bvh'], self.directory, "testset/", select_idx)
		
		if mode == "custom":
			write_result(render_data, self.config['skel']['bvh'], self.directory, f"custom/{self.bvh_name}/", select_idx)			

	
	def train(self):
		self.writer = SummaryWriter(self.log_dir)
		logging.info("Training model...")
		torch.autograd.set_detect_anomaly(True)

		self.loss_total_min = 100000

		for epoch in range(self.config['train']['num_epoch']):
			epoch_loss = 0
			self.model.train()
			# teacher forcing ratio
			# self.teacher_forcing_ratio = 0.0 if self.pretrain else np.clip(
			# 	get_exp_teacher_forcing_ratio(epoch, scale=0.2), a_min=0, a_max=1,
			# )

			self.teacher_forcing_ratio = 0.0 if (self.pretrain or epoch >= 50) else float((50-epoch)/50)

			logging.info(
			f"Running epoch {epoch} | "
			f"teacher_forcing_ratio={self.teacher_forcing_ratio}"
			)

			steps_per_epoch = len(self.dataloader['train'])
			for iterations, sampled_batch in enumerate(self.dataloader['train']):
				src_seq = sampled_batch['src_seq'].to(self.device)
				if self.config['data']['add_noise']:
					src_seq = src_seq + 0.01 * torch.randn(src_seq.shape).to(self.device)
				tgt_seq = sampled_batch['tgt_seq'].to(self.device) # [batch, seq_len, dim]
				global_pos = sampled_batch['global_p'].to(self.device)
				root = sampled_batch['root'].to(self.device)
				# q = sampled_batch['local_q'].to(self.device)

				if self.config['model'] == "lstm":
					outputs = self.model(
						src_seq, tgt_seq, teacher_forcing_ratio=self.teacher_forcing_ratio
					)
				elif "conv" in self.config['model']:
					outputs = self.model(src_seq)
				outputs = outputs.double()

				# results = self.denorm_and_get_loss_norm(outputs, tgt_seq, global_pos, root, q)
				results = self.denorm_and_get_loss_norm(outputs, tgt_seq, global_pos, root)
				
				self.optimize()
				self.update(epoch, steps_per_epoch, iterations)
				epoch_loss += self.loss_total.item()
				# print(f"step {iterations} done")
			
			epoch_loss /= steps_per_epoch
			self.run(mode="validation") 
			self.save(epoch_loss, epoch)


	def denorm_and_get_loss_norm(self, outputs, tgt_seq, global_pos, root, get_denorm_results=False):

		# denorm output and perform FK
		batch, seq_len, _ = outputs.shape
		denorm_seq = denormalize(outputs, self.mean, self.std, self.device)
		denorm_tgt_seq = denormalize(tgt_seq, self.mean, self.std, self.device)
		tgt_facing_dir_vel = denorm_tgt_seq[...,5:6]

		root_height = denorm_seq[...,4:5]	
		facing_dir_vel = denorm_seq[...,5:6]
		root_xz_vel = denorm_seq[..., 6:9]	
		Q = denorm_seq[..., 9:].reshape(batch, seq_len, -1, 6)

		norm_rotmat = transforms.rotation_6d_to_matrix(Q)

		axis_y = torch.from_numpy(np.tile(np.array([0.0,1.0,0.0]), (batch,1))).to(self.device).unsqueeze(1)
		rotmat_id_tensor = torch.from_numpy(np.eye(3)[np.newaxis,...]).to(self.device).repeat(batch,1,1)

		facing_dir_aa = axis_y * facing_dir_vel

		rotmat_list = []
		root_pos_rotmat = []

		for i in range(seq_len):
			if i == 0:
				facing_rotation_mat, starting_pos_mat = rotmat_id_tensor, (axis_y.squeeze(-2) * root_height[:,0,:]).clone()
			else:
				rot_diff_mat = transforms.axis_angle_to_matrix(facing_dir_aa[:,i])
				facing_rotation_mat = rot_diff_mat @ facing_rotation_mat
				starting_pos_mat += (facing_rotation_mat.swapaxes(-2,-1) @ root_xz_vel[:,i,:].unsqueeze(-1)).squeeze(-1)
				starting_pos_mat[:,1] = (root_height[:,i,0]).clone()

			# record
			root_rotmat = facing_rotation_mat.swapaxes(-2,-1) @ norm_rotmat[:, i, 0, ...]
			frame_rotmat = torch.concat((root_rotmat.unsqueeze(1),norm_rotmat[:,i,1:,:]), dim=1) # [batch, joint, 3, 3]
			rotmat_list.append(frame_rotmat)
			root_pos_rotmat.append(starting_pos_mat.clone())

		rotmat_stack = torch.stack(rotmat_list, dim=1)
		root_pos_rotmat_stack = torch.stack(root_pos_rotmat, dim=1)

		# TODO: normalize root (provide answer root pos by teacher forcing ration)
		idx_teacher = []
		idx_else = []
		for i in range(seq_len):
			if random.random() < self.teacher_forcing_ratio:
				idx_teacher.append(i)
			else:
				idx_else.append(i)
		root = root.reshape(batch, seq_len, 3)
		root_mix = root_pos_rotmat_stack.clone()
		root_mix[:,idx_teacher,:] = root[:,idx_teacher,:]
		# root_mix[:,idx_else,:] = root[:,idx_else,:]
		# embed()

		if self.skel_offset.shape[0] != batch:
			# output_pos_mat = rot_matrix_fk_tensor(rotmat_stack, root_pos_rotmat_stack, self.skel_offset[0:batch], self.skel_parent)
			output_pos_mat = rot_matrix_fk_tensor(rotmat_stack, root_mix, self.skel_offset[0:batch], self.skel_parent)
		else:
			# output_pos_mat = rot_matrix_fk_tensor(rotmat_stack, root_pos_rotmat_stack, self.skel_offset, self.skel_parent)
			output_pos_mat = rot_matrix_fk_tensor(rotmat_stack, root_mix, self.skel_offset, self.skel_parent)
		
		# get losses (if not custom)
		self.contact_mse_loss = self.criterion(outputs[...,:4], tgt_seq[...,:4])
		self.root_mse_loss = self.criterion(outputs[...,4:9], tgt_seq[...,4:9])
		self.rotation_mse_loss = self.criterion(outputs[...,4+5:], tgt_seq[...,4+5:])			

		# root reconstruction loss
		# self.root_recon_loss = 0.01*self.criterion(root_pos_rotmat_stack, root.squeeze(-2))

		# compare pos
		pos_diff = torch.abs(global_pos - output_pos_mat) / self.x_std

		self.pos_mean_loss = torch.mean(pos_diff)
		# self.pos_mse_loss = self.criterion(global_pos, output_pos) / self.x_std

		self.loss_total = self.config['train']['loss_pos_weight'] * self.pos_mean_loss + \
						self.config['train']['loss_quat_weight'] * self.rotation_mse_loss + \
						self.config['train']['loss_root_weight'] * self.root_mse_loss + \
						self.config['train']['loss_contact_weight'] * self.contact_mse_loss


		return (root_pos_rotmat_stack, rotmat_stack) if get_denorm_results else None


	def denorm_and_get_loss(self, outputs, tgt_seq, global_pos, get_denorm_results=False):
		# denorm output and perform FK
		denorm_seq = denormalize(outputs, self.mean, self.std, self.device)

		output_pos, denorm_root_info, denorm_rotations_pred_ = divide_and_fk(denorm_seq, root_start_idx=4, rot_start_idx=4+5, rep=self.rep, \
																			  skel=self.skel_offset, parent=self.skel_parent)
		
		# get losses
		self.contact_mse_loss = self.criterion(outputs[...,:4], tgt_seq[...,:4])
		self.root_mse_loss = self.criterion(outputs[...,4:4+5], tgt_seq[...,4:4+5])
		self.rotation_mse_loss = self.criterion(outputs[...,4+5:], tgt_seq[...,4+5:])					

		# compare pos
		pos_diff = torch.abs(global_pos - output_pos) / self.x_std
		self.pos_mean_loss = torch.mean(pos_diff)
		# self.pos_mse_loss = self.criterion(global_pos, output_pos) / self.x_std

		self.loss_total = self.config['train']['loss_pos_weight'] * self.pos_mean_loss + \
						self.config['train']['loss_quat_weight'] * self.rotation_mse_loss + \
						self.config['train']['loss_root_weight'] * self.root_mse_loss + \
						self.config['train']['loss_contact_weight'] * self.contact_mse_loss

		if get_denorm_results and denorm_rotations_pred_.shape[-1] == 3:
			denorm_rotations_pred_ = transforms.matrix_to_quaternion(denorm_rotations_pred_) # TODO later erase this and edit quat_to_T function in render data

		return (denorm_root_info, denorm_rotations_pred_) if get_denorm_results else None
		
	def optimize(self):
		self.optimizer.zero_grad()
		self.loss_total.backward()
		torch.nn.utils.clip_grad_norm_(self.model.parameters(), 0.5)
		self.optimizer.step()
	
	def update(self, epoch, steps_per_epoch, idx):
		self.writer.add_scalar('loss_pos', self.pos_mean_loss.item(), global_step = epoch * steps_per_epoch + idx)
		self.writer.add_scalar('loss_quat', self.rotation_mse_loss.item(), global_step = epoch * steps_per_epoch + idx)
		self.writer.add_scalar('loss_root', self.root_mse_loss.item(), global_step = epoch * steps_per_epoch + idx)
		self.writer.add_scalar('loss_contact', self.contact_mse_loss.item(), global_step = epoch * steps_per_epoch + idx)
		self.writer.add_scalar('loss_total', self.loss_total.item(), global_step = epoch * steps_per_epoch + idx)

	def save(self, epoch_loss, epoch):
		if (epoch % self.save_frequency == self.save_frequency-1) or epoch_loss < self.loss_total_min:
			# print("save")
			logging.info("Saving model")
			torch.save(self.model.state_dict(), self.model_dir + 'autoencoder.pkl')
			torch.save(self.optimizer.state_dict(), self.model_dir + 'optimizer.pkl')
			if epoch_loss < self.loss_total_min:
				self.loss_total_min = epoch_loss
		
		logging.info(f"Current Epoch: {epoch} | "
					 f"Current Loss: {epoch_loss} | "
					 f"Best Loss: {self.loss_total_min}")


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--test_name", type=str, default="")
	parser.add_argument("--config", type=str, default="")

	args = parser.parse_args()
	if not os.path.exists("./output/"):
		os.mkdir("./output/")
	directory = "./output/" + args.test_name + "/"
	if not os.path.exists(directory):
		os.mkdir(directory)
	
	autoencoder = AutoEncoder()
	# autoencoder.init_train(directory=directory, config=args.config, pretrain=False)
	# autoencoder.run("validation")
	# autoencoder.train()

	autoencoder.init_test(directory=directory)
	autoencoder.run()

	# autoencoder.init_custom(directory=directory, custom_bvh_path="./data/sit_z.bvh")
	# autoencoder.run(mode="custom")
	# autoencoder.write_result()

	# autoencoder.init_run(directory=directory, is_custom=True, custom_bvh_path="clockwise_jog.bvh")
	# autoencoder.run()
	# autoencoder.write_result()

	# autoencoder.init_run(directory=directory, is_optimize=True, is_custom=True, custom_bvh_path="stand.bvh")
	# autoencoder.latent_optimize()
	# autoencoder.write_result()