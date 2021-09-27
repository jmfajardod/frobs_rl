#!/bin/python3

import os
import stable_baselines3
from frobs_rl.common import ros_params
from frobs_rl.models import basic_model

# ROS packages required
import rospy



class PPO(basic_model.BasicModel):

    def __init__(self, env, save_model_path, log_path, config_file_pkg="frobs_rl", config_filename="ppo_config.yaml", ns="/") -> None:
        """
        PPO constructor.

        @param env: The environment to be used.
        @param save_model_path: The path to save the model.
        @param log_path: The path to save the log.

        @param config_file_pkg: The package where the config file is located. Default: frobs_rl.
        @param config_filename: The name of the config file. Default: ppo_config.yaml.
        @param ns: The namespace of the ROS parameters. Default: "/".
        
        """
        
        rospy.loginfo("Init PPO Policy")
        print("Init PPO Policy")

        self.env = env
        self.ns = ns
        self.save_model_path = save_model_path
        self.save_trained_model_path = None

        # Load YAML Config File
        ros_params.ROS_Load_YAML_from_pkg(config_file_pkg, config_filename, ns=ns)

        #--- Init super class
        super(PPO, self).__init__(env, save_model_path, log_path)

        #--- SDE for PPO
        if rospy.get_param(ns + "/model_params/use_sde"):
            model_sde = True
            model_sde_sample_freq   = rospy.get_param(ns + "/model_params/sde_params/sde_sample_freq")
            self.action_noise = None
        else:
            model_sde = False
            model_sde_sample_freq   = -1

        #--- PPO model parameters
        model_learning_rate = rospy.get_param(ns + "/model_params/ppo_params/learning_rate")
        model_n_steps       = rospy.get_param(ns + "/model_params/ppo_params/n_steps")
        model_batch_size    = rospy.get_param(ns + "/model_params/ppo_params/batch_size")
        model_n_epochs      = rospy.get_param(ns + "/model_params/ppo_params/n_epochs")
        model_gamma         = rospy.get_param(ns + "/model_params/ppo_params/gamma")
        model_gae_lambda    = rospy.get_param(ns + "/model_params/ppo_params/gae_lambda")
        model_clip_range    = rospy.get_param(ns + "/model_params/ppo_params/clip_range")
        model_ent_coef      = rospy.get_param(ns + "/model_params/ppo_params/ent_coef")
        model_vf_coef       = rospy.get_param(ns + "/model_params/ppo_params/vf_coef")
        model_max_grad_norm = rospy.get_param(ns + "/model_params/ppo_params/max_grad_norm")

        #--- Create or load model 
        if rospy.get_param(ns + "/model_params/load_model"): # Load model
            model_name = rospy.get_param(ns + "/model_params/model_name")
            assert os.path.exists(save_model_path + model_name + ".zip"), "Model {} doesn't exist".format(model_name)
            rospy.logwarn("Loading model: " + model_name)
            self.model = stable_baselines3.PPO.load(save_model_path + model_name, env=env, verbose=1, learning_rate=model_learning_rate,
                            use_sde=model_sde, sde_sample_freq= model_sde_sample_freq,
                            n_steps=model_n_steps, batch_size=model_batch_size, n_epochs=model_n_epochs, gamma=model_gamma, 
                            gae_lambda=model_gae_lambda, clip_range=model_clip_range, ent_coef=model_ent_coef, 
                            vf_coef=model_vf_coef, max_grad_norm=model_max_grad_norm)
            if os.path.exists(save_model_path + model_name + "_replay_buffer.pkl"):
                rospy.logwarn("Loading replay buffer")
                self.model.load_replay_buffer(save_model_path + model_name + "_replay_buffer")
            else:
                rospy.logwarn("No replay buffer found")

        else: # Create new model
            rospy.logwarn("Creating new model")
            self.model = stable_baselines3.PPO("MlpPolicy", env, verbose=1 , learning_rate=model_learning_rate,
                    use_sde=model_sde, sde_sample_freq= model_sde_sample_freq,
                    n_steps=model_n_steps, batch_size=model_batch_size, n_epochs=model_n_epochs, gamma=model_gamma, 
                    gae_lambda=model_gae_lambda, clip_range=model_clip_range, ent_coef=model_ent_coef, 
                    policy_kwargs=self.policy_kwargs, vf_coef=model_vf_coef, max_grad_norm=model_max_grad_norm)

        #--- Logger
        self.set_model_logger()

        
