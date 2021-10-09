.. _rl_models:

RL Models
============

**FRobs_RL** include an API of all the RL algorithms implemented in `stable-baselines3 <https://github.com/DLR-RM/stable-baselines3>`_, namely `PPO <https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html>`_, `A2C <https://stable-baselines3.readthedocs.io/en/master/modules/a2c.html>`_, `DDPG <https://stable-baselines3.readthedocs.io/en/master/modules/ddpg.html>`_, `DQN <https://stable-baselines3.readthedocs.io/en/master/modules/dqn.html>`_, `SAC <https://stable-baselines3.readthedocs.io/en/master/modules/sac.html>`_ and `TD3 <https://stable-baselines3.readthedocs.io/en/master/modules/td3.html>`_. All of these have an API in which all the algorithm parameters are read from the ROS parameter server, which means that the parameters can be loaded from a YAML file or set/changed through the ROS CLI commands.

In the API of the algorithms is configured the logging system, by default **FRobs_RL** saves the training logs in both CSV and Tensorboard files. In the API is also selected that the models are trained periodically in intervals set by the user in the YAML file, in case the user does not want to save the models they just need to change the associated parameter.

**FRobs_RL** include YAML templates for the parameters of all algorithms in the `config folder <https://github.com/jmfajardod/frobs_rl/tree/main/config>`_. To use any of the algorithms the user just needs to create a config folder inside the ROS package where the enviroment to train is located, change the parameters as needed and call the algorithm from the Python script.

Basic YAML parameters
---------------------

All of the algorithms YAML file has the next parameters:

.. code-block:: YAML

    model_params:

        # Training 
        training_steps: 5000      # The number of training steps to perform

        # Save params
        save_freq: 1000 # The step interval to save a new model
        save_prefix: ppo_model # Prefix of models saved
        trained_model_name: trained_model # Name of final model to save
        save_replay_buffer: False

        # Load model params
        load_model: False
        model_name: ppo_model_5000_steps

        # Logging parameters
        log_folder: PPO_1
        log_interval: 4 # The number of episodes between logs
        reset_num_timesteps: False # If true, will reset the number of timesteps to 0 every training 

        # Use custom policy - Only MlpPolicy is supported (Only used when new model is created)
        use_custom_policy: False
        policy_params:
            net_arch: [400, 300] # List of hidden layer sizes
            activation_fn: relu  # relu, tanh, elu or selu
            features_extractor_class: FlattenExtractor # FlattenExtractor, BaseFeaturesExtractor or CombinedExtractor
            optimizer_class: Adam # Adam, Adadelta, Adagrad, RMSprop or SGD

In these parameters the user can select how many steps to train the model (*training_steps*), how often the models will be saved (*save_freq*), the name of the final model saved after all the training steps (*trained_model_name*), wheter to load a previously saved model and its name (*load_model*, *model_name*), etc.

For the logging parameters the **user needs to remember to** change the log folder parameter every time a train script is executed, as **FRobs_RL** checks if the folder is created and will raise an error if a folder with the same name exists.

Finally, the user can choose is they want a custom neural network (*use_custom_policy*) and its arquitecture/parameters like the optimizer or the activation function.


Algorithm parameters
---------------------

The algorithm specific parameters are located below the general parameters shown above. The algorithm related parameters include wheter to use action noise, or `gSDE folder <https://arxiv.org/abs/2005.05719>`_  along the algorithm parameters like the learning rate, batch size, etc. The default algorithm specific parameters for PPO are shown below:

.. code-block:: YAML

    # Use SDE
    use_sde: False
    sde_params:
        sde_sample_freq: -1

    # PPO parameters
    ppo_params:
        learning_rate: 0.0003
        n_steps: 100    # The number of steps to run for each environment per update (i.e. rollout buffer size is n_steps * n_envs where n_envs is number of environment copies running in parallel)
        batch_size: 100 # Minibatch size
        n_epochs: 5     # Number of epoch when optimizing the surrogate loss
        gamma: 0.99
        gae_lambda: 0.95
        clip_range: 0.2
        ent_coef: 0.0
        vf_coef: 0.5
        max_grad_norm: 0.5


Use of algorithms
-----------------

After copying the YAML template to the config folder of the user ROS package the user just need to import the algorithm from the library and set the env, the save path where the models will be saved, the log path where all the logs will be saved and the location and name of the YAML paremeter file. After creating the algorithm the user needs to call the *train* method to initiate the learning process which will be the number of steps specific in the YAML file (*training_steps*).

An example using the TD3 algorithm is shown below:

.. code-block:: python
    
    from kobuki_maze_rl.task_env import kobuki_maze
    from frobs_rl.common import ros_gazebo, ros_node
    import gym
    import rospy

    # Import TD3 algorithm
    from frobs_rl.models.td3 import TD3

    if __name__ == '__main__':
        # Kill all processes related to previous runs
        ros_node.ros_kill_all_processes()

        # Launch Gazebo 
        ros_gazebo.launch_Gazebo(paused=True, gui=False)

        # Start node
        rospy.logwarn("Start")
        rospy.init_node('kobuki_maze_train')

        # Launch the task environment
        env = gym.make('KobukiMazeEnv-v0')

         #--- Normalize action space
        env = NormalizeActionWrapper(env)

        #--- Normalize observation space
        env = NormalizeObservWrapper(env)

        #--- Set max steps
        env = TimeLimitWrapper(env, max_steps=15000)
        env.reset()

         #--- Set the save and log path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("kobuki_maze_rl")


        #-- TD3
        save_path = pkg_path + "/models/td3/"
        log_path = pkg_path + "/logs/td3/"
        model = TD3(env, save_path, log_path, config_file_pkg="kobuki_maze_rl", config_filename="td3.yaml")


        model.train()
        model.save_model()
        model.close_env()

        sys.exit()


Custom algorithms
-----------------

While all the algorithms in *stable-baselines3* are included, the user can use the **FRobs_RL** library with their custom algorithms, althought the user must handle the model saving and logging processes.

It is recommended that if the user wants to implement a new algorithm it is done inheriting the base RL class of stable-baselines3 located in the official website `sb3 base RL class <https://stable-baselines3.readthedocs.io/en/master/modules/base.html>`_ or `Github repository <https://github.com/DLR-RM/stable-baselines3/blob/master/stable_baselines3/common/base_class.py>`_


Basic algorithm API
-------------------

Below is the documentation of the basic API of the RL algorithms inherited by every one of the included algorithms.

.. autoclass:: basic_model.BasicModel
  :members:
  :private-members:
  :noindex: