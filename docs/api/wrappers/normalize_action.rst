Normalize Action Wrapper
========================

Enviroment wrapper to normalize the action space. 

.. note::
    Only works for Box action spaces.

Example use
---------------------

.. code-block:: python

    from kobuki_maze_rl.task_env import kobuki_maze
    from frobs_rl.common import ros_gazebo
    import gym
    import rospy

    from frobs_rl.wrappers.NormalizeActionWrapper import NormalizeActionWrapper

    if __name__ == '__main__':

        # Launch Gazebo 
        ros_gazebo.launch_Gazebo(paused=True, gui=False)

        # Start node
        rospy.logwarn("Start")
        rospy.init_node('kobuki_maze_train')

        # Launch the task environment
        env = gym.make('KobukiMazeEnv-v0')

        #--- Normalize action space
        env = NormalizeActionWrapper(env)


Wrapper documentation
---------------------

.. autoclass:: NormalizeActionWrapper.NormalizeActionWrapper
   :members:
   :private-members:
   :noindex: