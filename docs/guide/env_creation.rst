.. _env_creation:

Enviroment creation
===================

To create a new enviroment with a robotic platform and a task the user must create a class in which the robot and task is defined. To facilitate many of the required steps needed to create an *env* (enviroment) following the OpenAI Gym documentation **FRobs_RL** already has a class which implements most of the required functions needed by Gym. The class is called **RobotBasicEnv** and can be inherited by any enviroment regardless of the robot or task. The main functions already implemented in **RobotBasicEnv** are:

- step: This is the funcion that will be executed in each step of the RL loop.
- reset: The funcion called when the enviroment must be reset, wheter it may be because of the sucess or because the time limit has been reached.
- close: The funcion that is executed when the enviroment is closed. It mainly makes sure that all ROS Nodes and the Gazebo simulator are properly closed.

To create a new enviroment the user must inherit the **RobotBasicEnv** and fill the next functions:

- _send_action: The function used to send the commands to the robot.
- _get_observation: The function to execute when observations from the enviroment are needed.
- _get_reward: The function that calculates and returns the reward based on the action of the agent.
- _check_if_done: The function to check if the robot sucess finished the task or reached the goal.
- _check_subs_and_pubs_connection (Optional): The function that check if the ROS subscribers and publishers are connected and properly receiving or sending messages.
- _set_episode_init_params (Optional): The function to set ROS or Gazebo initial parameters for the episode, e.g.: the initial position of the robot, the location of obstacles, etc.

To create a new enviroment we recommend that the user creates two different classes a **CustomRobotEnv** and a **CustomTaskEnv**, in this way the principal funcions are separted in the following way:

- All processes related to the *robot* are located in the **CustomRobotEnv**, this can be the URDF model loading, controllers spawning, spawning the robot in the simulator, etc.
- The **CustomTaskEnv** inherits the **CustomRobotEnv** and are where all processes related directly to the task are implemented, this can be the way to send actions to the robot agent, the process to obtain observations, the reward funcion, etc.

The previous arquitecture has the advantage that a **CustomRobotEnv** can be reused in many tasks, reducing the amount of code needed to create a new enviroment

In the next step a guide will be shown to how to use the templates of the **CustomRobotEnv** and  **CustomTaskEnv** classes included in the **FRobs_RL** library. 

.. note::
    Althought the previous separation in **CustomRobotEnv** and  **CustomTaskEnv** is recommended, the user can program the enviroment in any way inheriting the **RobotBasicEnv**.

.. autoclass:: robot_BasicEnv.RobotBasicEnv
    :members:
    :private-members:
    :noindex: