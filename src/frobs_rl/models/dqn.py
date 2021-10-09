#!/bin/python3

import os
import stable_baselines3
from frobs_rl.common import ros_params
from frobs_rl.models import basic_model

# ROS packages required
import rospy


class DQN(basic_model.BasicModel):
    """
    Deep Q Network (DQN) algorithm.

    Paper: https://arxiv.org/abs/1312.5602

    :param env: The environment to be used.
    :param save_model_path: The path to save the model.
    :param log_path: The path to save the log.
    :param load_trained: Whether to load a trained model.

    :param config_file_pkg: The package where the config file is located. Default: frobs_rl.
    :param config_filename: The name of the config file. Default: dqn_config.yaml.
    :param ns: The namespace of the ROS parameters. Default: "/".
    """

    def __init__(self, env, save_model_path, log_path, load_trained=False,
                config_file_pkg="frobs_rl", config_filename="dqn_config.yaml", ns="/") -> None:
        """
        DQN constructor.
        """
        
        rospy.loginfo("Init DQN Policy")
        print("Init DQN Policy")

        self.env = env
        self.ns = ns
        self.save_model_path = save_model_path
        self.save_trained_model_path = None

        # Load YAML Config File
        ros_params.ros_load_yaml_from_pkg(config_file_pkg, config_filename, ns=ns)

        #--- Init super class
        super(DQN, self).__init__(env, save_model_path, log_path, load_trained=load_trained)

        if load_trained:
            rospy.logwarn("Loading trained model")
            self.model = stable_baselines3.DQN.load(save_model_path, env=env)
        else:
            #--- DQN model parameters  
            model_learning_rate           = rospy.get_param(ns + "/model_params/dqn_params/learning_rate")
            model_buffer_size             = rospy.get_param(ns + "/model_params/dqn_params/buffer_size")
            model_learning_starts         = rospy.get_param(ns + "/model_params/dqn_params/learning_starts")
            model_batch_size              = rospy.get_param(ns + "/model_params/dqn_params/batch_size")
            model_tau                     = rospy.get_param(ns + "/model_params/dqn_params/tau")
            model_gamma                   = rospy.get_param(ns + "/model_params/dqn_params/gamma")
            model_gradient_steps          = rospy.get_param(ns + "/model_params/dqn_params/gradient_steps")
            model_train_freq_freq         = rospy.get_param(ns + "/model_params/dqn_params/train_freq/freq")
            model_train_freq_unit         = rospy.get_param(ns + "/model_params/dqn_params/train_freq/unit")
            model_target_update_interval  = rospy.get_param(ns + "/model_params/dqn_params/target_update_interval")
            model_exploration_fraction    = rospy.get_param(ns + "/model_params/dqn_params/exploration_fraction")
            model_exploration_initial_eps = rospy.get_param(ns + "/model_params/dqn_params/exploration_initial_eps")
            model_exploration_final_eps   = rospy.get_param(ns + "/model_params/dqn_params/exploration_final_eps")
            model_max_grad_norm           = rospy.get_param(ns + "/model_params/dqn_params/max_grad_norm")


            #--- Create or load model 
            if rospy.get_param(ns + "/model_params/load_model"): # Load model
                model_name = rospy.get_param(ns + "/model_params/model_name")
                assert os.path.exists(save_model_path + model_name + ".zip"), "Model {} doesn't exist".format(model_name)
                rospy.logwarn("Loading model: " + model_name)
                self.model = stable_baselines3.DQN.load(save_model_path + model_name, env=env, verbose=1, learning_rate=model_learning_rate, 
                                buffer_size=model_buffer_size, learning_starts=model_learning_starts, batch_size=model_batch_size, 
                                tau=model_tau, gamma=model_gamma, gradient_steps=model_gradient_steps, 
                                target_update_interval=model_target_update_interval, exploration_fraction=model_exploration_fraction, 
                                exploration_initial_eps=model_exploration_initial_eps,exploration_final_eps=model_exploration_final_eps, 
                                max_grad_norm=model_max_grad_norm, train_freq=(model_train_freq_freq, model_train_freq_unit))

                if os.path.exists(save_model_path + model_name + "_replay_buffer.pkl"):
                    rospy.logwarn("Loading replay buffer")
                    self.model.load_replay_buffer(save_model_path + model_name + "_replay_buffer")
                else:
                    rospy.logwarn("No replay buffer found")

            else: # Create new model
                rospy.logwarn("Creating new model")

                self.model = stable_baselines3.DQN("MlpPolicy", env,  verbose=1, learning_rate=model_learning_rate, 
                                buffer_size=model_buffer_size, learning_starts=model_learning_starts, batch_size=model_batch_size, 
                                tau=model_tau, gamma=model_gamma, gradient_steps=model_gradient_steps, 
                                target_update_interval=model_target_update_interval, exploration_fraction=model_exploration_fraction, 
                                exploration_initial_eps=model_exploration_initial_eps,exploration_final_eps=model_exploration_final_eps, 
                                max_grad_norm=model_max_grad_norm, policy_kwargs=self.policy_kwargs, 
                                train_freq=(model_train_freq_freq, model_train_freq_unit))

            #--- Logger
            self.set_model_logger()


    def load_trained(model_path, env=None):
        """
        Load a trained model. Use only with predict function, as the logs will not be saved.

        :param model_path: The path to the trained model.
        :type model_path: str
        :param env: The environment to be used.
        :type env: gym.Env

        :return: The trained model.
        :rtype: frobs_rl.DQN
        """

        model = DQN(env=env, save_model_path=model_path, log_path=model_path, load_trained=True)

        return model