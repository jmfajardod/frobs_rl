#!/bin/python3

import os
import stable_baselines3
from frobs_rl.common import ros_params
from frobs_rl.models import basic_model

# ROS packages required
import rospy


class TD3(basic_model.BasicModel):

    """
    Twin Delayed DDPG (TD3) algorithm.

    Paper: https://arxiv.org/abs/1802.09477

    :param env: The environment to be used.
    :param save_model_path: The path to save the model.
    :param log_path: The path to save the log.
    :param load_trained: Whether to load a trained model(Used in load function).

    :param config_file_pkg: The package where the config file is located. Default: frobs_rl.
    :param config_filename: The name of the config file. Default: td3_config.yaml.
    :param ns: The namespace of the ROS parameters. Default: "/".
    """

    def __init__(self, env, save_model_path, log_path, load_trained=False,
                config_file_pkg="frobs_rl", config_filename="td3_config.yaml", ns="/") -> None:
        """
        TD3 constructor.
        """
        
        rospy.loginfo("Init TD3 Policy")
        print("Init TD3 Policy")

        self.env = env
        self.ns = ns
        self.save_model_path = save_model_path
        self.save_trained_model_path = None

        # Load YAML Config File
        ros_params.ros_load_yaml_from_pkg(config_file_pkg, config_filename, ns=ns)

        #--- Init super class
        super(TD3, self).__init__(env, save_model_path, log_path, load_trained=load_trained)

        if load_trained:
            rospy.logwarn("Loading trained model")
            self.model = stable_baselines3.TD3.load(save_model_path, env=env)
        else:
            #--- TD3 model parameters  
            model_learning_rate       = rospy.get_param(ns + "/model_params/td3_params/learning_rate")
            model_buffer_size         = rospy.get_param(ns + "/model_params/td3_params/buffer_size")
            model_learning_starts     = rospy.get_param(ns + "/model_params/td3_params/learning_starts")
            model_batch_size          = rospy.get_param(ns + "/model_params/td3_params/batch_size")
            model_tau                 = rospy.get_param(ns + "/model_params/td3_params/tau")
            model_gamma               = rospy.get_param(ns + "/model_params/td3_params/gamma")
            model_gradient_steps      = rospy.get_param(ns + "/model_params/td3_params/gradient_steps")
            model_train_freq_freq     = rospy.get_param(ns + "/model_params/td3_params/train_freq/freq")
            model_train_freq_unit     = rospy.get_param(ns + "/model_params/td3_params/train_freq/unit")
            model_policy_delay        = rospy.get_param(ns + "/model_params/td3_params/policy_delay")
            model_target_policy_noise = rospy.get_param(ns + "/model_params/td3_params/target_policy_noise")
            model_target_noise_clip   = rospy.get_param(ns + "/model_params/td3_params/target_noise_clip")


            #--- Create or load model 
            if rospy.get_param(ns + "/model_params/load_model"): # Load model
                model_name = rospy.get_param(ns + "/model_params/model_name")
                assert os.path.exists(save_model_path + model_name + ".zip"), "Model {} doesn't exist".format(model_name)
                rospy.logwarn("Loading model: " + model_name)
                self.model = stable_baselines3.TD3.load(save_model_path + model_name, env=env, verbose=1, action_noise=self.action_noise,
                        learning_rate=model_learning_rate, buffer_size=model_buffer_size, learning_starts=model_learning_starts,
                        batch_size=model_batch_size, tau=model_tau, gamma=model_gamma,gradient_steps=model_gradient_steps,
                        policy_delay=model_policy_delay, target_policy_noise=model_target_policy_noise, 
                        target_noise_clip=model_target_noise_clip, train_freq=(model_train_freq_freq, model_train_freq_unit))
                if os.path.exists(save_model_path + model_name + "_replay_buffer.pkl"):
                    rospy.logwarn("Loading replay buffer")
                    self.model.load_replay_buffer(save_model_path + model_name + "_replay_buffer")
                else:
                    rospy.logwarn("No replay buffer found")

            else: # Create new model
                rospy.logwarn("Creating new model")
                self.model = stable_baselines3.TD3("MlpPolicy", env, verbose=1 ,action_noise=self.action_noise,
                        learning_rate=model_learning_rate, buffer_size=model_buffer_size, learning_starts=model_learning_starts,
                        batch_size=model_batch_size, tau=model_tau, gamma=model_gamma,gradient_steps=model_gradient_steps,
                        policy_kwargs=self.policy_kwargs, policy_delay=model_policy_delay, target_policy_noise=model_target_policy_noise, 
                        target_noise_clip=model_target_noise_clip, train_freq=(model_train_freq_freq, model_train_freq_unit))

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
        :rtype: frobs_rl.TD3
        """

        model = TD3(env=env, save_model_path=model_path, log_path=model_path, load_trained=True)

        return model
