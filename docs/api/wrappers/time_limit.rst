Time Limit Wrapper
========================

Enviroment wrapper to set a maximum number of steps per episode. If the agent does not succesfully end the task or reach the goal the episode will end and the property ``is_success`` will be set to 0.


Example use
---------------------

.. code-block:: python

    from kobuki_maze_rl.task_env import kobuki_maze
    from frobs_rl.common import ros_gazebo
    import gym
    import rospy

    from frobs_rl.wrappers.TimeLimitWrapper import TimeLimitWrapper

    if __name__ == '__main__':

        # Launch Gazebo 
        ros_gazebo.launch_Gazebo(paused=True, gui=False)

        # Start node
        rospy.logwarn("Start")
        rospy.init_node('kobuki_maze_train')

        # Launch the task environment
        env = gym.make('KobukiMazeEnv-v0')

        #--- Set max steps
        env = TimeLimitWrapper(env, max_steps=15000)


Wrapper documentation
---------------------

.. autoclass:: TimeLimitWrapper.TimeLimitWrapper
   :members:
   :private-members:
   :noindex: