#!/bin/python3

import os

# ROS packages required
import rospy
import stable_baselines3
import stable_baselines3.common
import stable_baselines3.common.buffers
from stable_baselines3.common.policies import ActorCriticPolicy

from frobs_rl.common import ros_params
from frobs_rl.models import basic_model


class PPO(basic_model.BasicModel):
    """
    Proximal Policy Optimization (PPO) algorithm.

    Paper: https://arxiv.org/abs/1707.06347

    :param env: The environment to be used.
    :param save_model_path: The path to save the model.
    :param log_path: The path to save the log.
    :param load_trained: Whether to load a trained model or not.

    :param config_file_pkg: The package where the config file is located. Default: frobs_rl.
    :param config_filename: The name of the config file. Default: ppo_config.yaml.
    :param ns: The namespace of the ROS parameters. Default: "/".
    """

    def __init__(
        self,
        env,
        save_model_path,
        log_path,
        load_trained=False,
        load_il_policy=False,
        il_policy_path=None,
        config_file_pkg="frobs_rl",
        config_filename="ppo_config.yaml",
        ns="/",
    ) -> None:
        """
        PPO constructor.
        """

        rospy.loginfo("Init PPO Policy")
        print("Init PPO Policy")

        self.env = env
        self.ns = ns
        self.save_model_path = save_model_path
        self.save_trained_model_path = None

        # Load YAML Config File
        ros_params.ros_load_yaml_from_pkg(config_file_pkg, config_filename, ns=ns)

        # --- Init super class
        super(PPO, self).__init__(
            env, save_model_path, log_path, load_trained=load_trained
        )

        if load_trained:
            rospy.logwarn("Loading trained model")
            self.model = stable_baselines3.PPO.load(save_model_path, env=env)
        elif load_il_policy:
            rospy.logwarn("Loading imitation learning policy")
            self.model = stable_baselines3.PPO.load(il_policy_path, env=env)

            # Modify hyperparameters
            self.model.batch_size = rospy.get_param(
                ns + "/model_params/ppo_params/batch_size"
            )
            self.model.learning_rate = rospy.get_param(
                ns + "/model_params/ppo_params/learning_rate"
            )
            self.model.n_steps = rospy.get_param(
                ns + "/model_params/ppo_params/n_steps"
            )
            self.model.n_epochs = rospy.get_param(
                ns + "/model_params/ppo_params/n_epochs"
            )
            self.model.gamma = rospy.get_param(ns + "/model_params/ppo_params/gamma")
            self.model.gae_lambda = rospy.get_param(
                ns + "/model_params/ppo_params/gae_lambda"
            )
            self.model.ent_coef = rospy.get_param(
                ns + "/model_params/ppo_params/ent_coef"
            )
            self.model.vf_coef = rospy.get_param(
                ns + "/model_params/ppo_params/vf_coef"
            )
            self.model.max_grad_norm = rospy.get_param(
                ns + "/model_params/ppo_params/max_grad_norm"
            )

            self.model.policy.use_sde = rospy.get_param(ns + "/model_params/use_sde")
            self.model.rollout_buffer = stable_baselines3.common.buffers.RolloutBuffer(
                buffer_size=self.model.n_steps,
                observation_space=self.model.observation_space,
                action_space=self.model.action_space,
                device=self.model.device,
                gae_lambda=self.model.gae_lambda,
                gamma=self.model.gamma,
                n_envs=self.model.n_envs,
            )
            self.model.rollout_buffer.full = False

            # Set SDE
            if rospy.get_param(ns + "/model_params/use_sde"):
                self.model.use_sde = True
                self.model.sde_sample_freq = rospy.get_param(
                    ns + "/model_params/sde_params/sde_sample_freq"
                )
                self.model.action_noise = None
                rospy.logerr(f"IL policy SDE sample freq: {self.model.sde_sample_freq}")
            else:
                self.model.use_sde = False
                self.model.sde_sample_freq = -1

            rospy.logwarn(f"IL policy Net arch: {self.model.policy.net_arch}")
            rospy.logwarn(f"IL policy Activation fn: {self.model.policy.activation_fn}")
            rospy.logwarn(f"IL policy Batch size: {self.model.batch_size}")
            rospy.logwarn(f"IL policy Learning rate: {self.model.learning_rate}")
            rospy.logwarn(f"IL policy N steps: {self.model.n_steps}")

            self.set_model_logger()
        else:
            # --- SDE for PPO
            if rospy.get_param(ns + "/model_params/use_sde"):
                model_sde = True
                model_sde_sample_freq = rospy.get_param(
                    ns + "/model_params/sde_params/sde_sample_freq"
                )
                self.action_noise = None
            else:
                model_sde = False
                model_sde_sample_freq = -1

            # --- PPO model parameters
            model_learning_rate = rospy.get_param(
                ns + "/model_params/ppo_params/learning_rate"
            )
            model_n_steps = rospy.get_param(ns + "/model_params/ppo_params/n_steps")
            model_batch_size = rospy.get_param(
                ns + "/model_params/ppo_params/batch_size"
            )
            model_n_epochs = rospy.get_param(ns + "/model_params/ppo_params/n_epochs")
            model_gamma = rospy.get_param(ns + "/model_params/ppo_params/gamma")
            model_gae_lambda = rospy.get_param(
                ns + "/model_params/ppo_params/gae_lambda"
            )
            model_clip_range = rospy.get_param(
                ns + "/model_params/ppo_params/clip_range"
            )
            model_ent_coef = rospy.get_param(ns + "/model_params/ppo_params/ent_coef")
            model_vf_coef = rospy.get_param(ns + "/model_params/ppo_params/vf_coef")
            model_max_grad_norm = rospy.get_param(
                ns + "/model_params/ppo_params/max_grad_norm"
            )

            # --- Create or load model
            if rospy.get_param(ns + "/model_params/load_model"):  # Load model
                model_name = rospy.get_param(ns + "/model_params/model_name")
                assert os.path.exists(
                    save_model_path + model_name + ".zip"
                ), "Model {} doesn't exist".format(model_name)
                rospy.logwarn("Loading model: " + model_name)
                self.model = stable_baselines3.PPO.load(
                    save_model_path + model_name,
                    env=env,
                    verbose=1,
                    learning_rate=model_learning_rate,
                    use_sde=model_sde,
                    sde_sample_freq=model_sde_sample_freq,
                    n_steps=model_n_steps,
                    batch_size=model_batch_size,
                    n_epochs=model_n_epochs,
                    gamma=model_gamma,
                    gae_lambda=model_gae_lambda,
                    clip_range=model_clip_range,
                    ent_coef=model_ent_coef,
                    vf_coef=model_vf_coef,
                    max_grad_norm=model_max_grad_norm,
                )
                if os.path.exists(save_model_path + model_name + "_replay_buffer.pkl"):
                    rospy.logwarn("Loading replay buffer")
                    self.model.load_replay_buffer(
                        save_model_path + model_name + "_replay_buffer"
                    )
                else:
                    rospy.logwarn("No replay buffer found")

            else:  # Create new model
                rospy.logwarn("Creating new model")
                self.model = stable_baselines3.PPO(
                    "MlpPolicy",
                    env,
                    verbose=1,
                    learning_rate=model_learning_rate,
                    use_sde=model_sde,
                    sde_sample_freq=model_sde_sample_freq,
                    n_steps=model_n_steps,
                    batch_size=model_batch_size,
                    n_epochs=model_n_epochs,
                    gamma=model_gamma,
                    gae_lambda=model_gae_lambda,
                    clip_range=model_clip_range,
                    ent_coef=model_ent_coef,
                    policy_kwargs=self.policy_kwargs,
                    vf_coef=model_vf_coef,
                    max_grad_norm=model_max_grad_norm,
                )

            # --- Logger
            self.set_model_logger()

    @staticmethod
    def load_trained(model_path, env=None):
        """
        Load a trained model. Use only with predict function, as the logs will not be saved.

        :param model_path: The path to the trained model.
        :type model_path: str
        :param env: The environment to be used.
        :type env: gym.Env

        :return: The loaded model.
        :rtype: frobs_rl.PPO
        """

        model = PPO(
            env=env, save_model_path=model_path, log_path=model_path, load_trained=True
        )

        return model
