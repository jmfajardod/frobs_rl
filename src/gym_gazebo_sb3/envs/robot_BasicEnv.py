#!/bin/python3

import gym
from gym import spaces
from gym_gazebo_sb3.common import ros_gazebo
from gym_gazebo_sb3.common import ros_controllers
from gym_gazebo_sb3.common import ros_node
from gym_gazebo_sb3.common import ros_launch
from gym_gazebo_sb3.common import ros_params
from gym_gazebo_sb3.common import ros_urdf
from gym_gazebo_sb3.common import ros_spawn
import rospy

class RobotBasicEnv(gym.Env):

    def __init__(   self, launch_gazebo=False, gazebo_init_paused=True, gazebo_use_gui=True, gazebo_recording=False, 
                    gazebo_freq=100, world_path=None, world_pkg=None, world_filename=None,
                    gazebo_max_freq=None, gazebo_timestep=None,
                    spawn_robot=False, model_name_in_gazebo="robot", namespace="/robot", pkg_name=None, urdf_file=None, 
                    controller_file=None, controller_list=None, urdf_xacro_args=None, rob_state_publisher_max_freq= None,
                    model_pos_x=0.0, model_pos_y=0.0, model_pos_z=0.0, 
                    model_ori_x=0.0, model_ori_y=0.0, model_ori_z=0.0, model_ori_w=0.0,
                    reset_controllers=False, reset_mode=1, step_mode=1, num_gazebo_steps=1):
        """
        Function to initialize the environment.

        @param launch_gazebo: If True, launch Gazebo at the start of the env.
        @type launch_gazebo: bool
        @param gazebo_init_paused: If True, Gazebo is initialized in a paused state.
        @type gazebo_init_paused: bool
        @param gazebo_use_gui: If True, Gazebo is launched with a GUI (through gzclient).
        @type gazebo_use_gui: bool
        @param gazebo_recording: If True, Gazebo is launched with a recording of the GUI (through gzclient).
        @type gazebo_recording: bool
        @param gazebo_freq: The publish rate of gazebo in Hz.
        @type gazebo_freq: int

        To use a custom world, one can use two options: 1) set the path directly to the world file or set the pkg name and world filename.
        @param world_path: If using a custom world then the path to the world.
        @type world_path: str        
        @param world_pkg: If using a custom world then the package name of the world.
        @type world_pkg: str
        @param world_filename: If using a custom world then the filename of the world.
        @type world_filename: str

        @param gazebo_max_freq: max update rate for gazebo in real time factor: 1 is real time, 10 is 10 times real time.
        @type gazebo_max_freq: float
        @param gazebo_timestep: The timestep of gazebo in seconds.
        @type gazebo_timestep: float

        @param spawn_robot: If True, the robot is spawned in the environment.
        @type spawn_robot: bool
        @param model_name_in_gazebo: The name of the model in gazebo.
        @type model_name_in_gazebo: str
        @param namespace: The namespace of the robot.
        @type namespace: str
        @param pkg_name: The package name where the robot model is located.
        @type pkg_name: str
        @param urdf_file: The path to the urdf file of the robot.
        @type urdf_file: str
        @param urdf_xacro_args: The arguments to be passed to the xacro parser.
        @type urdf_xacro_args: str

        @param controller_file: The path to the controllers YAML file of the robot.
        @type controller_file: str
        @param controller_list: The list of controllers to be launched.
        @type controller_list: list of str

        @param rob_state_publisher_max_freq: The maximum frequency of the ros state publisher.
        @type rob_state_publisher_max_freq: int

        @param model_pos_x: The x position of the robot in the world.
        @parma model_pos_y: The y position of the robot in the world.
        @param model_pos_z: The z position of the robot in the world.
        @param model_ori_x: The x orientation of the robot in the world.
        @param model_ori_y: The y orientation of the robot in the world.
        @param model_ori_z: The z orientation of the robot in the world.
        @param model_ori_w: The w orientation of the robot in the world.

        @param reset_controllers: If True, the controllers are reset at the start of each episode.
        @type reset_controllers: bool

        @param reset_mode:  If 1, reset Gazebo with a "reset_world" (Does not reset time)
                            If 2, reset Gazebo with a "reset_simulation" (Resets time)

        @param step_mode:   If 1, step Gazebo using the "pause_physics" and "unpause_physics" services.
                            If 2, step Gazebo using the "step_simulation" command.

        @param num_gazebo_steps: If using step_mode 2, the number of steps to be taken.
        """

        super(RobotBasicEnv, self).__init__()

        self.namespace = namespace
        self.num_gazebo_steps = num_gazebo_steps
        self.reset_controllers = reset_controllers
        self.controllers_list = controller_list
        self.reset_mode = reset_mode
        self.step_mode = step_mode

        # If Launch Gazebo, launch it
        if launch_gazebo:
            ros_gazebo.Launch_Gazebo(   paused=gazebo_init_paused, use_sim_time=True, use_gui=gazebo_use_gui,
                                        recording=gazebo_recording, pub_clock_frequency=gazebo_freq,
                                        custom_world_path=world_path, custom_world_package=world_pkg, custom_world_name=world_filename)

        # Set the max frequency and timestep of Gazebo
        if gazebo_max_freq is not None:
            ros_gazebo.Gazebo_set_max_update_rate(gazebo_max_freq)
        if gazebo_timestep is not None:
            ros_gazebo.Gazebo_set_time_step(gazebo_timestep)

        # If spawn robot, spawn it
        if spawn_robot:
            ros_spawn.Spawn_model_in_gazebo(pkg_name, urdf_file, controller_file, self.controllers_list,
                            ns=self.namespace, args_xacro=urdf_xacro_args, max_pub_freq=rob_state_publisher_max_freq, 
                            gazebo_name=model_name_in_gazebo, gaz_ref_frame="world", 
                            pos_x=model_pos_x, pos_y=model_pos_y, pos_z=model_pos_z,
                            ori_x=model_ori_x, ori_y=model_ori_y, ori_z=model_ori_z, ori_w=model_ori_w)

        if self.reset_controllers:
            ros_gazebo.Gazebo_unpause_physics()
            ros_controllers.Reset_controllers_srv(controller_list, ns=self.namespace)
            ros_gazebo.Gazebo_pause_physics()

        rospy.loginfo("Started RobotBasicEnv")

    def step(self, action):
        """
        Function to send an action to the robot and get the observation and reward.
        """
        
        rospy.loginfo("Step Env")

        self.info = {}
        self.observation = None
        self.reward = 0.0

        # If using pause and unpause services
        if self.step_mode == 1:
            ros_gazebo.Gazebo_unpause_physics()
            self._send_action(action)
            ros_gazebo.Gazebo_pause_physics()

        # If using the gazebo step command
        elif self.step_mode == 2:
            self._send_action(action)
            ros_gazebo.Gazebo_step_physics(steps=self.num_gazebo_steps)

        self.observation = self._get_observation()
        self.reward = self._get_reward()
        self.done   = self._check_if_done()

        return self.observation, self.reward, self.done, self.info

    def reset(self):
        """
        Function to reset the enviroment after an episode is done.
        """
        
        rospy.loginfo("Resetting Env")
        ros_gazebo.Gazebo_pause_physics()
        self._reset_gazebo()
        self.observation = self._get_observation()

        return self.observation  

    def close (self):
        """
        Function to close the environment when training is done.
        """
        
        rospy.loginfo("Closing Env")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")
        ros_gazebo.Close_Gazebo()
        ros_node.ROS_Kill_All_Nodes()
        ros_node.ROS_Kill_Master()
        ros_node.ROS_Kill_All_processes()

        print("Closed ROS and Env")

    #---------------------------------------------#
    #   Methods to be overridden in the child Env #

    def _send_action(self, action):
        """
        Function to send an action to the robot
        """
        raise NotImplementedError()
        

    def _get_observation(self):
        """
        Function to get the observation from the enviroment.
        """
        raise NotImplementedError()

    def _get_reward(self):
        """
        Function to get the reward from the enviroment.
        """
        raise NotImplementedError()
    
    def _check_if_done(self):
        """
        Function to check if the episode is done.
        
        If the episode has a success condition then set done as:
            self.info['is_success'] = 1.0
        """
        raise NotImplementedError()
        
    #------------------------------------------#
    #   Custom methods for the RobotBasicEnv   #

    def _reset_gazebo(self):
        """
        Function to reset the gazebo simulation.
        """
        
        # If resetting world (Does not reset time)
        if self.reset_mode == 1:
            ros_gazebo.Gazebo_reset_world()
        
        # If resetting simulation (Resets time)
        elif self.reset_mode == 2:
            ros_gazebo.Gazebo_reset_sim()

        if self.reset_controllers:
            ros_gazebo.Gazebo_unpause_physics()
            ros_controllers.Reset_controllers_srv(self.controllers_list, ns=self.namespace)
            ros_gazebo.Gazebo_pause_physics()

        ros_gazebo.Gazebo_unpause_physics()
        self._check_subs_and_pubs_connection()
        self._set_episode_init_params()
        ros_gazebo.Gazebo_pause_physics()

    #------------------------------------------#
    #   Custom methods for the RobotBasicEnv   #

    
    def _check_subs_and_pubs_connection(self):
        """
        Function to check if the gazebo and ros connections are ready
        """
        raise NotImplementedError()

    def _set_episode_init_params(self):
        """
        Function to set some parameters, like the position of the robot, at the begining of each episode.
        """
        raise NotImplementedError()