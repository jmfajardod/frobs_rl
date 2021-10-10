.. _train_model:

Training a model
====================

After creating the enviroment the user can simply import the file and initialize the enviroment with the Gym function *make*. After the enviroment is initialized the user can train any kind of model with any RL algorithm using the Gym env. In the Python script the user also needs to initialize the ROS node and can add any commands as neeeded. An example script is shown below:

.. code-block:: python

    from kobuki_maze_rl.task_env import kobuki_maze
    from frobs_rl.common import ros_gazebo
    import gym
    import rospy

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

Note that the string input of the Gym *make* function is the ID of the env used when registering it.

FRobs_RL Env Wrappers
---------------------

To change some properties of the enviroments without changing the class env code the user can use Gym Wrappers. These wrappers are used to change properties like the observation or action space of the enviroment without the need to change them directly in the class. This is useful when the user might want to normalize the observation or action space (to change the speed of learning) or when the user want to specify a limit of steps per episode. In **FRobs_RL** the previous wrappers are already included with the names:

- NormalizeActionWrapper
- NormalizeObservWrapper
- TimeLimitWrapper

To use them the user only needs to import them from the library and pass the initialized enviroment. An example where the three wrappers are used is shown below:

.. code-block:: python
  
    from kobuki_maze_rl.task_env import kobuki_maze
    from frobs_rl.common import ros_gazebo
    import gym
    import rospy

    from frobs_rl.wrappers.NormalizeActionWrapper import NormalizeActionWrapper
    from frobs_rl.wrappers.TimeLimitWrapper import TimeLimitWrapper
    from frobs_rl.wrappers.NormalizeObservWrapper import NormalizeObservWrapper

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

Included RL models
---------------------

In the next step the RL models included from `stable-baselines3 <https://stable-baselines3.readthedocs.io/en/master/>`_ in the **FRobs_RL** and how to use them is shown.

Enviroment Wrappers
---------------------

.. automodule:: NormalizeActionWrapper
  :members:

.. automodule:: NormalizeObservWrapper
  :members:

.. automodule:: TimeLimitWrapper
  :members: