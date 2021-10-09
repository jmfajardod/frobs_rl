.. _use_trained_model:

Using trained models
====================

After the user has trained the model they can use it without the need of an algorithm YAML parameter file and without the processes related to training like the saving of the replay buffer of the calculation of the loss function.

To use a trained model the user just only needs to import the type of the model algorithm from the **FRobs_RL** library and use the *load_trained* function. After the trained model is loaded the user can use the *predict* function to obtain the action based on the observation.

An example where a TD3 trained model is loaded and used in two episodes is shown below.

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
        save_path = pkg_path + "/models/dynamic/td3/"

        #-- TD3 trained
        model = TD3.load_trained(save_path + "trained_model")


        obs = env.reset()
        episodes = 2
        epi_count = 0
        while epi_count < episodes:
            action, _states = model.predict(obs, deterministic=True)
            obs, _, dones, info = env.step(action)
            if dones:
                epi_count += 1
                rospy.logwarn("Episode: " + str(epi_count))
                obs = env.reset()

        env.close()
        sys.exit()

.. autofunction:: basic_model.BasicModel.predict