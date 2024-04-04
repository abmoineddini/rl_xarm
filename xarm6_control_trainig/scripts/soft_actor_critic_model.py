# import gym
# from gym import spaces
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray, Empty, Float32MultiArray
# import tensorflow as tf
# from keras import layers



# class XArm6Env(gym.Env, Node):
#     def __init__(self):
#         super().__init__('xarm_controller_env')
#         self.action_space = spaces.Box(low=np.array([-np.pi]*6), high=np.array([np.pi]*6), dtype=np.float32)
#         self.observation_space = spaces.Box(low=np.array([-640, -350, -450]), high=np.array([640, 350, 1000.0]), dtype=np.float32)
#         # ROS2 setup
#         self.subscription = self.create_subscription(Float64MultiArray, '/ball_position', self.ball_position_callback, 10)
#         self.publisher = self.create_publisher(Float32MultiArray, '/xarm_joint_angles', 10)
#         self.publisher_reset = self.create_publisher(Empty, '/reset_sim', 10)
        
#         self.state = np.zeros((10, 3), dtype=np.float32)

#     def create_actor(self, input_shape, action_space):
#         inputs = layers.Input(shape=input_shape)
#         x = layers.Dense(256, activation="relu")(inputs)
#         x = layers.Dense(256, activation="relu")(x)
#         outputs = layers.Dense(action_space.shape[0], activation="tanh")(x)
#         model = tf.keras.Model(inputs, outputs)
#         return model

#     def create_critic(self, input_shape, action_shape):
#         state_input = layers.Input(shape=input_shape)
#         action_input = layers.Input(shape=action_shape)
#         concat = layers.Concatenate()([state_input, action_input])

#         x = layers.Dense(256, activation="relu")(concat)
#         x = layers.Dense(256, activation="relu")(x)
#         outputs = layers.Dense(1)(x)

#         model = tf.keras.Model([state_input, action_input], outputs)
#         return model
    

#     def ball_position_callback(self, msg):
#         self.state = np.insert(self.state, [10], msg.data, axis=0)
#         self.state = np.delete(self.state, 0, axis=0)
        

#     def step(self, action):
#         # Publish action
#         msg = Float64MultiArray()
#         msg.data = action
#         self.publisher.publish(msg)

#         # Reward is higher the closer the ball is to (0, 0, 0)
#         reward = -np.linalg.norm(self.state[-10:])
#         done = False

#         if np.isnan(self.state).any():
#             done = True
#             reward = -100  # Large penalty for failure state

#         return self.state, reward, done, {}

#     def reset(self):
#         self.publisher_reset.publish()


