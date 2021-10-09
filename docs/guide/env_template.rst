.. _env_template:

Enviroment templates
====================

As mentioned in the previous step, **FRobs_RL** includes two templates to facilitate the creation of a new Gym enviroment. These templates can be found in the folder of  `templates <https://github.com/jmfajardod/frobs_rl/tree/main/src/frobs_rl/templates>`_ and are the two basic templates for the **CustomRobotEnv** and  **CustomTaskEnv** classes.


CustomRobotEnv
--------------

The **CustomRobotEnv** must the first class to be filled, as the **CustomTaskEnv** will inherit this class. In the template *init* function there are already many parameters included like the parameters to launch Gazebo along the class, the parameters related to the the URDF model loading of the robot, the controllers spawning, namespaces of the parameters, etc. An exert of the function is shown below:


.. code-block:: python

    """
    If spawning the robot using the given spawner then set the corresponding environment variables.
    """
    spawn_robot=False
    model_name_in_gazebo="robot"
    namespace="/robot"
    pkg_name=None
    urdf_file=None
    urdf_folder="/urdf"
    controller_file=None
    controller_list=None
    urdf_xacro_args=None
    rob_state_publisher_max_freq= None
    model_pos_x=0.0; model_pos_y=0.0; model_pos_z=0.0 
    model_ori_x=0.0; model_ori_y=0.0; model_ori_z=0.0; model_ori_w=0.0    

The user only needs to change this parameters and the inhered class **RobotBasicEnv** will spawn the robot with the position and orientation specified. An example of this is shown below:

.. code-block:: python

    spawn_robot=True
    model_name_in_gazebo="kobuki_robot"
    namespace="/"
    pkg_name="kobuki_maze_rl"
    urdf_file="kobuki_lidar.urdf.xacro"
    urdf_folder="/urdf"
    controller_file=None
    controller_list=[]
    urdf_xacro_args=None
    rob_state_publisher_max_freq=30
    model_pos_x=0.0; model_pos_y=10.0; model_pos_z=0.0 
    model_ori_x=0.0; model_ori_y=0.0; model_ori_z=0.0; model_ori_w=1.0

.. note::
    Even if the user decides to spawn the robot without the use of the **RobotBasicEnv** class they can use the functions availiable in **FRobs_RL** like *spawn_model_in_gazebo*, *gazebo_set_time_step*, *urdf_load_from_pkg*, etc.

After filling the *init* function of the **CustomRobotEnv** the user can fill the optional function *_check_subs_and_pubs_connection* if all publishers or subscribers of the enviroment are declared inside this class.

.. autoclass:: CustomRobotEnv.CustomRobotEnv
  :members: _check_subs_and_pubs_connection

Finally, the user only needs to change the name of the class so the **CustomTaskEnv** can inherit it. An example is shown below:

.. code-block:: python

    # With default name
    class CustomRobotEnv(robot_BasicEnv.RobotBasicEnv):

    # With new name to allow class inheritance
    class KobukiLIDAREnv(robot_BasicEnv.RobotBasicEnv):


CustomTaskEnv
-------------

After creating a **CustomRobotEnv** the user must fill the task related template **CustomTaskEnv**. The first change needed is the class that the Task enviroment will inherit and the name of the Task enviroment; the class to inherit must be appropiately imported and its name should be change in the class, an example is shown below:

.. code-block:: python

    # With default inherit
    from frobs_rl.templates import CustomRobotEnv # Replace with your own robot env
    class CustomTaskEnv(CustomRobotEnv.CustomRobotEnv):

    # With appropiate inherit of created RobotEnv
    from kobuki_maze_rl.robot_env import kobuki_lidar_env
    class KobukiMazeEnv(kobuki_lidar_env.KobukiLIDAREnv):

After this change the user must fill the *init* function of the class where the action and observations spaces must be filled according to the task, some supported spaces are the Discrete space or the Box space, which is a continous space with limit. An example of how to fill the spaces is shown below:

.. code-block:: python

    # Using Discrete spaces
    self.action_space = spaces.Discrete(n_actions)
    self.observation_space = spaces.Discrete(n_observations)

    # Using Box spaces
    self.action_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)
    self.observation_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

Also, the user can create different subscribers or publishers in the *init* function of this class whether is to obtain observations or to publish some markers, etc.

After filling the *init* function must fill the following functions:

- _send_action: The function used to send the commands to the robot.
- _get_observation: The function to execute when observations from the enviroment are needed.
- _get_reward: The function that calculates and returns the reward based on the action of the agent.
- _check_if_done: The function to check if the robot sucess finished the task or reached the goal.
- _set_episode_init_params (Optional): The function to set ROS or Gazebo initial parameters for the episode, e.g.: the initial position of the robot, the location of obstacles, etc.

.. autoclass:: CustomTaskEnv.CustomTaskEnv
  :members: _send_action, _get_observation, _get_reward, _check_if_done, _set_episode_init_params

.. note::
    Some examples of how to fill these functions can be seen in the resources repository `FRobs_RL Resources <https://github.com/jmfajardod/frobs_rl_resources>`_.

.. note::
    The user can also create other functions in the class according to their needs.

Finally, the user must change the Gym *register* function, located at the top of the file, so the enviroment gets properly registered and can be called from another Python script. The user must select an unique ID (String), they must change the entry point (the location of the file and name of the class) and optionally select a maximum number of steps per episode. An example of how to change the *register* function is shown below:

.. code-block:: python
    
    # Default register
    register(
        id='CustomTaskEnv-v0',
        entry_point='frobs_rl.templates.CustomTaskEnv:CustomTaskEnv',
        max_episode_steps=10000,
    )

    # Changed register
    register(
        id='KobukiMazeEnv-v0',
        entry_point='kobuki_maze_rl.task_env.kobuki_maze:KobukiMazeEnv',
        max_episode_steps=100000000000,
    )


Templates 
-------------

.. autoclass:: CustomRobotEnv.CustomRobotEnv 
  :members:
  :private-members:
  :noindex:

.. autoclass:: CustomTaskEnv.CustomTaskEnv
  :members:
  :private-members:
  :noindex: