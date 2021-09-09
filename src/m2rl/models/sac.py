#!/bin/python3

import os
import numpy as np
from datetime import datetime
import gym
from gym.envs.registration import register
import torch as th
import stable_baselines3
from m2rl.common import ros_params
from m2rl.models import basic_model
from m2rl.models.utils import get_policy_kwargs, get_action_noise

# ROS packages required
import rospy
import rospkg

# SB3 Callbacks
from stable_baselines3.common.callbacks import CheckpointCallback

# Noise
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

# Logger
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from stable_baselines3.common.logger import TensorBoardOutputFormat
from stable_baselines3.common.callbacks import BaseCallback



class SAC(basic_model.BasicModel):

    def __init__(self, env, save_model_path, log_path, config_file_pkg="m2rl", config_filename="sac_config.yaml", ns="/") -> None:
        """
        SAC constructor.

        @param env: The environment to be used.
        @param save_model_path: The path to save the model.
        @param log_path: The path to save the log.

        @param config_file_pkg: The package where the config file is located. Default: m2rl.
        @param config_filename: The name of the config file. Default: sac_config.yaml.
        @param ns: The namespace of the ROS parameters. Default: "/".
        
        """
        
        rospy.loginfo("Init SAC Policy")
        print("Init SAC Policy")

        self.env = env
        self.ns = ns
        self.save_model_path = save_model_path
        self.save_trained_model_path = None

        # Load YAML Config File
        ros_params.ROS_Load_YAML_from_pkg(config_file_pkg, config_filename, ns=ns)

        #--- Init super class
        super(SAC, self).__init__(env, save_model_path, log_path)

        #--- SDE for SAC
        if rospy.get_param(ns + "/model_params/use_sde"):
            model_sde = True
            model_sde_sample_freq   = rospy.get_param(ns + "/model_params/sde_params/sde_sample_freq")
            model_use_sde_at_warmup = rospy.get_param(ns + "/model_params/sde_params/use_sde_at_warmup")
            self.action_noise = None
        else:
            model_sde = False
            model_sde_sample_freq   = -1
            model_use_sde_at_warmup = False

        #--- SAC model parameters
        model_learning_rate          = rospy.get_param(ns + "/model_params/sac_params/learning_rate")
        model_buffer_size            = rospy.get_param(ns + "/model_params/sac_params/buffer_size")
        model_learning_starts        = rospy.get_param(ns + "/model_params/sac_params/learning_starts")
        model_batch_size             = rospy.get_param(ns + "/model_params/sac_params/batch_size")
        model_tau                    = rospy.get_param(ns + "/model_params/sac_params/tau")
        model_gamma                  = rospy.get_param(ns + "/model_params/sac_params/gamma")
        model_gradient_steps         = rospy.get_param(ns + "/model_params/sac_params/gradient_steps")
        model_ent_coef               = rospy.get_param(ns + "/model_params/sac_params/ent_coef")
        model_target_update_interval = rospy.get_param(ns + "/model_params/sac_params/target_update_interval")
        model_target_entropy         = rospy.get_param(ns + "/model_params/sac_params/target_entropy")
        model_train_freq_freq        = rospy.get_param(ns + "/model_params/sac_params/train_freq/freq")
        model_train_freq_unit        = rospy.get_param(ns + "/model_params/sac_params/train_freq/unit")

        #--- Create or load model 
        if rospy.get_param(ns + "/model_params/load_model"): # Load model
            model_name = rospy.get_param(ns + "/model_params/model_name")
            assert os.path.exists(save_model_path + model_name + ".zip"), "Model {} doesn't exist".format(model_name)
            rospy.logwarn("Loading model: " + model_name)
            self.model = stable_baselines3.SAC.load(save_model_path + model_name, env=env, verbose=1, action_noise=self.action_noise,
                    use_sde=model_sde, sde_sample_freq=model_sde_sample_freq, use_sde_at_warmup=model_use_sde_at_warmup,
                    learning_rate=model_learning_rate, buffer_size=model_buffer_size, learning_starts=model_learning_starts,
                    batch_size=model_batch_size, tau=model_tau, gamma=model_gamma, gradient_steps=model_gradient_steps, 
                    ent_coef=model_ent_coef, target_update_interval=model_target_update_interval,
                    target_entropy=model_target_entropy, train_freq=(model_train_freq_freq, model_train_freq_unit))
            if os.path.exists(save_model_path + model_name + "_replay_buffer.pkl"):
                rospy.logwarn("Loading replay buffer")
                self.model.load_replay_buffer(save_model_path + model_name + "_replay_buffer")
            else:
                rospy.logwarn("No replay buffer found")

        else: # Create new model
            rospy.logwarn("Creating new model")
            self.model = stable_baselines3.SAC("MlpPolicy", env, verbose=1 ,action_noise=self.action_noise,
                    use_sde=model_sde, sde_sample_freq=model_sde_sample_freq, use_sde_at_warmup=model_use_sde_at_warmup,
                    learning_rate=model_learning_rate, buffer_size=model_buffer_size, learning_starts=model_learning_starts,
                    batch_size=model_batch_size, tau=model_tau, gamma=model_gamma, gradient_steps=model_gradient_steps, 
                    policy_kwargs=self.policy_kwargs, ent_coef=model_ent_coef, target_update_interval=model_target_update_interval,
                    target_entropy=model_target_entropy, train_freq=(model_train_freq_freq, model_train_freq_unit)) 

        #--- Logger
        self.set_model_logger()
