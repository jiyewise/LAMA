import tensorflow as tf
import numpy as np
from tensorflow.keras import Sequential, Input
from tensorflow.keras.layers import Dense

actor_layer_size = 256
critic_layer_size = 256
activ = "relu"

# l2_regularizer_scale = 0.0
# regularizer = tf.keras.regularizers.l2(0.5 * (l2_regularizer_scale))
# kernel_initialize_func = tf.compat.v1.keras.initializers.VarianceScaling(scale=1.0, mode="fan_avg", distribution="uniform")
class Actor(object):
	def __init__(self, num_states, num_actions, name):
		self.mean, self.logstd = self.create_network(num_states, num_actions, name)

	def create_network(self, num_states, num_actions, name):
		postfix = name+'_mean'
		mean = Sequential()
		mean.add(Dense(actor_layer_size, activation=activ, input_shape=(num_states, ), dtype=tf.float32, name=postfix+'/layer_with_weights-0'))
		mean.add(Dense(actor_layer_size, activation=activ, dtype=tf.float32, name=postfix+'/layer_with_weights-1'))
		mean.add(Dense(actor_layer_size, activation=activ, dtype=tf.float32, name=postfix+'/layer_with_weights-2'))
		mean.add(Dense(num_actions, dtype=tf.float32, name=postfix+'/layer_with_weights-3'))

		logstd = tf.Variable(initial_value = np.zeros(num_actions), name=name+'_logstd', dtype=tf.float32, trainable=True)	
		return mean, logstd
	
	@tf.function
	def std(self):
		return tf.exp(self.logstd)

	@tf.function
	def neglogp(self, action, mean):
		return 0.5 * tf.reduce_sum(input_tensor=tf.square((action - mean) / self.std()), axis=-1) \
			   + 0.5 * np.log(2.0 * np.pi) * tf.cast(tf.shape(input=action)[-1], dtype=tf.float32) \
			   + tf.reduce_sum(self.logstd, axis=-1)

	@tf.function
	def get_action(self, states):
		mean = self.mean(states)
		action = mean + self.std() * tf.random.normal(tf.shape(mean))
		neglogprob = self.neglogp(action, mean)
		return action, neglogprob

	@tf.function
	def get_mean_action(self, states):
		return self.mean(states)

	def get_variable(self, trainable_only=False):
		if trainable_only:
			return self.mean.trainable_variables + [self.logstd]
		else:
			return self.mean.variables + [self.logstd] 


class Critic(object):
	def __init__(self, num_states, name):
		self.value = self.create_network(num_states, name)

	def create_network(self, num_states, name):	
		value = Sequential()
		value.add(Dense(critic_layer_size, activation=activ, input_shape=(num_states, ), dtype=tf.float32, name=name+'/layer_with_weights-0'))
		value.add(Dense(critic_layer_size, activation=activ, dtype=tf.float32, name=name+'/layer_with_weights-1'))
		value.add(Dense(1, dtype=tf.float32, name=name+'/layer_with_weights-2'))
	
		return value

	@tf.function
	def get_value(self, states):
		return self.value(states)[:, 0]

	def get_variable(self, trainable_only=False):
		if trainable_only:
			return self.value.trainable_variables
		else:
			return self.value.variables