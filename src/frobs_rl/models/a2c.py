#!/bin/python3

import os
import stable_baselines3
from frobs_rl.common import ros_params
from frobs_rl.models import basic_model

# ROS packages required
import rospy



class A2C(basic_model.BasicModel):

    """
    Advantage Actor-Critic (A2C) algorithm.

    Paper: https://arxiv.org/abs/1602.01783
    
    :param env: The environment to be used.
    :param save_model_path: The path to save the model.
    :param log_path: The path to save the log.
    :param load_trained: If True, load a trained model.

    :param config_file_pkg: The package where the config file is located. Default: frobs_rl.
    :param config_filename: The name of the config file. Default: a2c_config.yaml.
    :param ns: The namespace of the ROS parameters. Default: "/".
    """

    def __init__(self, env, save_model_path, log_path, load_trained=False,
                config_file_pkg="frobs_rl", config_filename="a2c_config.yaml", ns="/") -> None:
        """
        A2C constructor.
        """
        
        rospy.loginfo("Init A2C Policy")
        print("Init A2C Policy")

        self.env = env
        self.ns = ns
        self.save_model_path = save_model_path
        self.save_trained_model_path = None

        # Load YAML Config File
        ros_params.ros_load_yaml_from_pkg(config_file_pkg, config_filename, ns=ns)

        #--- Init super class
        super(A2C, self).__init__(env, save_model_path, log_path, load_trained=load_trained)

        if load_trained:
            rospy.logwarn("Loading trained model")
            self.model = stable_baselines3.A2C.load(save_model_path, env=env)
        else:
            #--- SDE for A2C
            if rospy.get_param(ns + "/model_params/use_sde"):
                model_sde = True
                model_sde_sample_freq   = rospy.get_param(ns + "/model_params/sde_params/sde_sample_freq")
                self.action_noise = None
            else:
                model_sde = False
                model_sde_sample_freq   = -1

            #--- A2C model parameters
            model_learning_rate = rospy.get_param(ns + "/model_params/a2c_params/learning_rate")
            model_n_steps       = rospy.get_param(ns + "/model_params/a2c_params/n_steps")
            model_gamma         = rospy.get_param(ns + "/model_params/a2c_params/gamma")
            model_gae_lambda    = rospy.get_param(ns + "/model_params/a2c_params/gae_lambda")
            model_ent_coef      = rospy.get_param(ns + "/model_params/a2c_params/ent_coef")
            model_vf_coef       = rospy.get_param(ns + "/model_params/a2c_params/vf_coef")
            model_max_grad_norm = rospy.get_param(ns + "/model_params/a2c_params/max_grad_norm")
            model_use_rms_prop  = rospy.get_param(ns + "/model_params/a2c_params/use_rms_prop")
            model_rms_prop_eps  = rospy.get_param(ns + "/model_params/a2c_params/rms_prop_eps")
            model_norm_advant   = rospy.get_param(ns + "/model_params/a2c_params/normalize_advantage")

            #--- Create or load model 
            if rospy.get_param(ns + "/model_params/load_model"): # Load model
                model_name = rospy.get_param(ns + "/model_params/model_name")
                assert os.path.exists(save_model_path + model_name + ".zip"), "Model {} doesn't exist".format(model_name)
                rospy.logwarn("Loading model: " + model_name)
                self.model = stable_baselines3.A2C.load(save_model_path + model_name, env=env, verbose=1, learning_rate=model_learning_rate,
                                n_steps=model_n_steps, gamma=model_gamma, gae_lambda=model_gae_lambda, ent_coef=model_ent_coef,
                                vf_coef=model_vf_coef, max_grad_norm=model_max_grad_norm,
                                use_sde=model_sde, sde_sample_freq= model_sde_sample_freq,
                                use_rms_prop=model_use_rms_prop, rms_prop_eps=model_rms_prop_eps, normalize_advantage=model_norm_advant)
                if os.path.exists(save_model_path + model_name + "_replay_buffer.pkl"):
                    rospy.logwarn("Loading replay buffer")
                    self.model.load_replay_buffer(save_model_path + model_name + "_replay_buffer")
                else:
                    rospy.logwarn("No replay buffer found")

            else: # Create new model
                rospy.logwarn("Creating new model")
                self.model = stable_baselines3.A2C("MlpPolicy", env, verbose=1 , policy_kwargs=self.policy_kwargs,
                        learning_rate=model_learning_rate, n_steps=model_n_steps, gamma=model_gamma,
                        gae_lambda=model_gae_lambda, ent_coef=model_ent_coef, vf_coef=model_vf_coef, max_grad_norm=model_max_grad_norm,
                        use_sde=model_sde, sde_sample_freq= model_sde_sample_freq,
                        use_rms_prop=model_use_rms_prop, rms_prop_eps=model_rms_prop_eps, normalize_advantage=model_norm_advant)

            #--- Logger
            self.set_model_logger()

    
    def load_trained(model_path, env=None):
        """
        Load a trained model. Use only with predict function, as the logs will not be saved.

        :param model_path: The path to the trained model.
        :type model_path: str
        :param env: The environment to be used.
        :type env: gym.Env

        :return: The loaded model.
        :rtype: frobs_rl.A2C
        """

        model = A2C(env=env, save_model_path=model_path, log_path=model_path, load_trained=True)

        return model
