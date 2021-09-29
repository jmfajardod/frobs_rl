#!/bin/python3

import numpy as np
import torch as th
import stable_baselines3

# ROS packages required
import rospy

# Noise
from stable_baselines3.common.noise import NormalActionNoise #, OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.env_checker import check_env

def get_policy_kwargs(ns="/"):
    """
    Function to get the policy kwargs from the ROS params server.

    @param ns: namespace of the ROS params server
    @type ns: str

    @return: policy kwargs
    """

    if rospy.get_param(ns + "/model_params/use_custom_policy") == True:
        # Activation function for the policy
        activation_function = rospy.get_param(ns + "/model_params/policy_params/activation_fn").lower()
        if activation_function == "relu":
            activation_fn = th.nn.ReLU
        elif activation_function == "tanh":
            activation_fn = th.nn.Tanh
        elif activation_function == "elu":
            activation_fn = th.nn.ELU
        elif activation_function == "selu":
            activation_fn = th.nn.SELU

        # Feature extractor for the policy
        feature_extractor = rospy.get_param(ns + "/model_params/policy_params/features_extractor_class")
        if feature_extractor == "FlattenExtractor":
            features_extractor_class = stable_baselines3.common.torch_layers.FlattenExtractor
        elif feature_extractor == "BaseFeaturesExtractor":
            features_extractor_class = stable_baselines3.common.torch_layers.BaseFeaturesExtractor
        elif feature_extractor == "CombinedExtractor":
            features_extractor_class = stable_baselines3.common.torch_layers.CombinedExtractor

        # Optimizer for the policy
        optimizer_class = rospy.get_param(ns + "/model_params/policy_params/optimizer_class")
        if optimizer_class == "Adam":
            optimizer_class = th.optim.Adam
        elif optimizer_class == "SGD":
            optimizer_class = th.optim.SGD
        elif optimizer_class == "RMSprop":
            optimizer_class = th.optim.RMSprop
        elif optimizer_class == "Adagrad":
            optimizer_class = th.optim.Adagrad
        elif optimizer_class == "Adadelta":
            optimizer_class = th.optim.Adadelta

        # Net Archiecture for the policy
        net_arch = rospy.get_param(ns + "/model_params/policy_params/net_arch")

        policy_kwargs = dict(activation_fn=activation_fn, features_extractor_class=features_extractor_class,
                            optimizer_class=optimizer_class, net_arch=net_arch)
        rospy.logwarn(policy_kwargs)
        print(policy_kwargs)
    else:
        policy_kwargs = None

    return policy_kwargs

def get_action_noise(action_space_shape, ns="/"):
    """
    Function to get the action noise from the ROS params server.

    @param action_space_shape: shape of the action space.
    @param ns: namespace of the ROS params server
    """

    action_noise = None

    if rospy.has_param(ns + "/model_params/use_action_noise") is False:
        rospy.loginfo("Parameter use_action_noise was not found on the parameter server.")
        return action_noise

    if rospy.get_param(ns + "/model_params/use_action_noise"):
        action_mean  = rospy.get_param(ns + "/model_params/action_noise/mean")
        action_sigma = rospy.get_param(ns + "/model_params/action_noise/sigma")
        action_noise = NormalActionNoise(mean= action_mean*np.ones(action_space_shape), sigma=action_sigma*np.ones(action_space_shape))
        

    return action_noise

def test_env(env):
    """
    Use SB3 env checker.
    """
    check_env(env)
    return True
