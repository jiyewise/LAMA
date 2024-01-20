import sys
if not hasattr(sys, 'argv'):
    sys.argv  = ['']

from network import Actor
from network import Critic
from env_wrapper import EnvWrapper
import argparse
import random
import numpy as np
import tensorflow as tf
import datetime
import os
import time
from copy import deepcopy
from utils import RunningMeanStd
import scipy.integrate as integrate
from IPython import embed

np.set_printoptions(threshold=sys.maxsize)
os.environ['CUDA_VISIBLE_DEVICES'] = ''

class PPO(object):
	def __init__(self, learning_rate_actor=2e-4, learning_rate_critic=0.001, learning_rate_decay=0.9993,
		gamma=0.95, lambd=0.95, epsilon=0.2):
		random.seed(int(time.time()))
		np.random.seed(int(time.time()))
		
		self.learning_rate_critic = learning_rate_critic
		self.learning_rate_actor = learning_rate_actor
		self.learning_rate_decay = learning_rate_decay

		self.epsilon = epsilon
		self.gamma = gamma
		self.lambd = lambd
		self.reward_max = 0

	def initRun(self, directory, num_state, num_action, num_slave=1):
		self.pretrain = True
		self.directory = directory

		self.num_slave = num_slave
		self.num_action = num_action
		self.num_state = num_state

		self.buildOptimize()

		self.ckpt = tf.train.Checkpoint(
			actor_mean=self.actor.mean,
			actor_logstd=self.actor.logstd,
			critic=self.critic.value
		)

		self.load(self.directory)
		li = self.directory.split("network")
		suffix = li[-1]
		self.RMS = RunningMeanStd(shape=(self.num_state))
		self.RMS.load(li[0]+"network"+li[1]+'rms'+suffix)
		self.RMS.set_num_states(self.num_state)

	def initTrain(self, name, env, pretrain, directory, batch_size=512, steps_per_iteration=6000, optim_frequency=1, save_frequency=5):

		self.name = name
		self.directory = directory
		self.steps_per_iteration = steps_per_iteration
		self.optim_frequency = optim_frequency
		self.save_frequency = save_frequency
		self.batch_size = batch_size

		self.env = env
		self.num_slave = self.env.num_slave
		self.num_action = self.env.num_action
		self.num_state = self.env.num_state

		self.buildOptimize()

		self.ckpt = tf.train.Checkpoint(
			actor_mean=self.actor.mean,
			actor_logstd=self.actor.logstd,
			critic=self.critic.value
		)

		if pretrain != "":
			self.load(self.directory+pretrain)
			li = pretrain.split("network")
			suffix = li[-1]

			self.env.RMS.load(self.directory+'rms'+suffix)
			self.env.RMS.set_num_states(self.num_state)
			self.pretrained = True
		else:
			self.pretrained = False
		
		self.printSetting()
		

	def printSetting(self):
		
		print_list = []
		print_list.append(datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
		print_list.append("test_name : {}".format(self.name))
		print_list.append("num slaves : {}".format(self.num_slave))
		print_list.append("num states : {}".format(self.num_state))
		print_list.append("num actions : {}".format(self.num_action))
		print_list.append("learning rate : {}".format(self.learning_rate_actor))
		print_list.append("gamma : {}".format(self.gamma))
		print_list.append("lambd : {}".format(self.lambd))
		print_list.append("batch size : {}".format(self.batch_size))
		print_list.append("steps per iteration : {}".format(self.steps_per_iteration))
		print_list.append("clip ratio : {}".format(self.epsilon))
		print_list.append("pretrained : {}".format(self.pretrained))

		for s in print_list:
			print(s)

		if self.directory is not None:
			out = open(self.directory+"settings", "w")
			for s in print_list:
				out.write(s + "\n")
			out.close()

			out = open(self.directory+"results", "w")
			out.close()

	def buildOptimize(self):
		self.actor = Actor(self.num_state, self.num_action, 'actor')
		self.critic = Critic(self.num_state, 'critic')

		self.critic_trainer = tf.keras.optimizers.Adam(learning_rate=self.learning_rate_critic)
		self.actor_trainer = tf.keras.optimizers.Adam(learning_rate=self.learning_rate_actor)

	@tf.function
	def train_critic_network(self, states, TD):
		with tf.GradientTape() as tape:
			loss = tf.reduce_mean(tf.square(self.critic.get_value(states) - TD))
		params = self.critic.get_variable(True)
		grads = tape.gradient(loss, params)
		grads, _ = tf.clip_by_global_norm(grads, 0.5)
		
		self.critic_trainer.apply_gradients(zip(grads, params))
		return loss

	@tf.function
	def train_actor_network(self, states, actions, neglogp, GAE):
		with tf.GradientTape() as tape:
			means = self.actor.get_mean_action(states)
			cur_neglogp = self.actor.neglogp(actions, means)
			ratio = tf.exp(neglogp-cur_neglogp)
			clipped_ratio = tf.clip_by_value(ratio, 1.0 - self.epsilon, 1.0 + self.epsilon)
			loss = -tf.reduce_mean(tf.minimum(ratio*GAE, clipped_ratio*GAE))
		
		params = self.actor.get_variable(True)
		grads = tape.gradient(loss, params)
		grads, _ = tf.clip_by_global_norm(grads, 0.5)
		
		self.actor_trainer.apply_gradients(zip(grads, params))
		return loss

	def update(self, tuples):
		state_batch, action_batch, TD_batch, neglogp_batch, GAE_batch = self.compute_TD_GAE(tuples)

		if len(state_batch) < self.batch_size:
			return
		GAE_batch = (GAE_batch - GAE_batch.mean())/(GAE_batch.std() + 1e-5)

		ind = np.arange(len(state_batch))
		np.random.shuffle(ind)

		lossval_ac = 0
		lossval_c = 0
		for _ in range(5):
			for s in range(int(len(ind)//self.batch_size)):
				selectedIndex = ind[s*self.batch_size:(s+1)*self.batch_size]
				lossval_ac += self.train_actor_network(tf.constant(state_batch[selectedIndex], dtype=np.float32), 
													   tf.constant(action_batch[selectedIndex], dtype=np.float32), 
													   tf.constant(neglogp_batch[selectedIndex], dtype=np.float32), 
													   tf.constant(GAE_batch[selectedIndex], dtype=np.float32))
				lossval_c += self.train_critic_network(tf.constant(state_batch[selectedIndex], dtype=np.float32), 
													   tf.constant(TD_batch[selectedIndex], dtype=np.float32))
		self.lossvals = []
		self.lossvals.append(['loss actor', lossval_ac / 5.0])
		self.lossvals.append(['loss critic', lossval_c / 5.0])

	def compute_TD_GAE(self, tuples):
		state_batch = []
		action_batch = []
		TD_batch = []
		neglogp_batch = []
		GAE_batch = []
		for data in tuples:
			size = len(data)		
			# get values
			states, actions, rewards, values, neglogprobs = zip(*data)
			values = np.concatenate((values, [0]), axis=0)
			advantages = np.zeros(size)
			ad_t = 0
			for i in reversed(range(len(data))):
				delta = rewards[i] + values[i+1] * self.gamma - values[i]
				ad_t = delta + self.lambd * self.gamma  * ad_t
				advantages[i] = ad_t

			TD = values[:size] + advantages

			for i in range(size):
				state_batch.append(states[i])
				action_batch.append(actions[i])
				TD_batch.append(TD[i])
				neglogp_batch.append(neglogprobs[i])
				GAE_batch.append(advantages[i])
		return np.array(state_batch), np.array(action_batch), np.array(TD_batch), np.array(neglogp_batch), np.array(GAE_batch)

					
	def save(self):
		self.ckpt.write(self.directory+'network-0')
		self.env.RMS.save(self.directory+'rms-0')

	def load(self, path):
		print("Loading parameters from {}".format(path))
		saved_variables = tf.train.list_variables(path)
		saved_values = []
		for v in saved_variables:
			saved_values.append(tf.train.load_variable(path,v[0]))
		saved_dict = {n[0] : v for n, v in zip(saved_variables, saved_values)}
		trainable_variables = self.actor.get_variable(True) + self.critic.get_variable(True)
	
		for v in trainable_variables:
			key = v.name[:-2]+'/.ATTRIBUTES/VARIABLE_VALUE'
			if key in saved_dict:
				saved_v = saved_dict[key]
				if v.shape == saved_v.shape:
					print("Restore {}".format(key))
					v.assign(saved_v)

	def print_network_summary(self):
		print_list = []
		for v in self.lossvals:
			print_list.append('{}: {:.3f}'.format(v[0], v[1]))

		print_list.append('===============================================================')
		for s in print_list:
			print(s)

	def train(self, num_iteration):
		epi_info_iter = []

		for it in range(num_iteration):
			for i in range(self.num_slave):
				self.env.reset(i)
			states = self.env.get_states()
			local_step = 0
			last_print = 0
	
			epi_info = [[] for _ in range(self.num_slave)]	

			while True:
				# set action
				actions, neglogprobs = self.actor.get_action(states)
				actions = actions.numpy()
				neglogprobs = neglogprobs.numpy()
				values = self.critic.get_value(states)
				rewards, dones = self.env.step(actions)

				for j in range(self.num_slave):
					if not self.env.get_terminated(j):
						if rewards[j] is not None:
							epi_info[j].append([states[j], actions[j], rewards[j], values[j], neglogprobs[j]])
							local_step += 1
						if dones[j]:
							if len(epi_info[j]) != 0:
								epi_info_iter.append(deepcopy(epi_info[j]))

							if local_step < self.steps_per_iteration:
								epi_info[j] = []
								self.env.reset(j)
							else:
								self.env.set_terminated(j)

				if local_step >= self.steps_per_iteration:
					if self.env.get_all_terminated():
						print('iter {} : {}/{}'.format(it+1, local_step, self.steps_per_iteration),end='\r')
						break
				if last_print + 100 < local_step: 
					print('iter {} : {}/{}'.format(it+1, local_step, self.steps_per_iteration),end='\r')
					last_print = local_step
				
				states = self.env.get_states()
			print('')

			if it % self.optim_frequency == self.optim_frequency - 1:
				self.update(epi_info_iter) 
				epi_info_iter = []

			if it % self.save_frequency == self.save_frequency - 1:
				summary = self.env.print_summary()
				self.print_network_summary()
				if self.directory is not None:
					self.save()

				if self.directory is not None and self.reward_max < summary['r_per_e']:
					self.reward_max = summary['r_per_e']
					self.env.RMS.save(self.directory+'rms-rmax')

					os.system("cp {}network-{}.data-00000-of-00001 {}network-rmax.data-00000-of-00001".format(self.directory, 0, self.directory))
					os.system("cp {}network-{}.index {}network-rmax.index".format(self.directory, 0, self.directory))


	def run(self, state):
		state = np.reshape(state, (1, self.num_state))
		state = self.RMS.apply(state)
		
		values = self.critic.get_value(state)
		action = self.actor.get_mean_action(state)
		# action, _ = self.actor.get_action(state)

		return action

if __name__=="__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--ntimesteps", type=int, default=1000000)
	parser.add_argument("--env", type=str, default="")
	parser.add_argument("--test_name", type=str, default="")
	parser.add_argument("--pretrain", type=str, default="")
	parser.add_argument("--nslave", type=int, default=8)
	parser.add_argument("--no-plot", dest='plot', action='store_false')

	parser.set_defaults(plot=True)

	args = parser.parse_args()
	if not os.path.exists("./output/"):
		os.mkdir("./output/")

	directory = "./output/" + args.test_name + "/"
	if not os.path.exists(directory):
		os.mkdir(directory)

	env = EnvWrapper(env=args.env, num_slave=args.nslave, directory=directory, plot=args.plot)
	ppo = PPO()
	ppo.initTrain(env=env, name=args.test_name, directory=directory, pretrain=args.pretrain)
	ppo.train(args.ntimesteps)