Normalize Observation Wrapper
==============================

Enviroment wrapper to normalize the observation space. 

.. note::
    Only works for Box observation spaces.

Example use
---------------------

.. code-block:: python

    from kobuki_maze_rl.task_env import kobuki_maze
    from frobs_rl.common import ros_gazebo
    import gym
    import rospy

    from frobs_rl.wrappers.NormalizeObservWrapper import NormalizeObservWrapper

    if __name__ == '__main__':

        # Launch Gazebo 
        ros_gazebo.launch_Gazebo(paused=True, gui=False)

        # Start node
        rospy.logwarn("Start")
        rospy.init_node('kobuki_maze_train')

        # Launch the task environment
        env = gym.make('KobukiMazeEnv-v0')

        #--- Normalize observation space
        env = NormalizeObservWrapper(env)


Wrapper documentation
---------------------

.. autoclass:: NormalizeObservWrapper.NormalizeObservWrapper
   :members:
   :private-members:
   :noindex: