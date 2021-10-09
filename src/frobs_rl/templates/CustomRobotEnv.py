#!/bin/python3

from gym import spaces
from gym.envs.registration import register
from frobs_rl.envs import robot_BasicEnv
import rospy

#- Uncomment the library modules as neeeed
# from frobs_rl.common import ros_gazebo
# from frobs_rl.common import ros_controllers
# from frobs_rl.common import ros_node
# from frobs_rl.common import ros_launch
# from frobs_rl.common import ros_params
# from frobs_rl.common import ros_urdf
# from frobs_rl.common import ros_spawn

"""
Although it is best to register only the task environment, one can also register the
robot environment.
"""
# register(
#         id='CustomRobotEnv-v0',
#         entry_point='frobs_rl.templates.CustomRobotEnv:CustomRobotEnv',
#         max_episode_steps=10000,
#     )

class CustomRobotEnv(robot_BasicEnv.RobotBasicEnv):
    """
    Custom Robot Env, use this for all task envs using the custom robot.
    """

    def __init__(self):
        """
        Describe the robot used in the env.
        """
        rospy.loginfo("Starting Custom Robot Env")

        """
        If launching Gazebo with the env then set the corresponding environment variables.
        """
        launch_gazebo=False
        gazebo_init_paused=True
        gazebo_use_gui=True
        gazebo_recording=False 
        gazebo_freq=100
        gazebo_max_freq=None
        gazebo_timestep=None
        
        """
        If launching Gazebo with a custom world then set the corresponding environment variables.
        """
        world_path=None
        world_pkg=None
        world_filename=None
        
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
        
        """
        Set if the controllers in "controller_list" will be reset at the beginning of each episode, default is False.
        """
        reset_controllers=False

        """
        Set the reset mode of gazebo at the beginning of each episode: 1 is "reset_world", 2 is "reset_simulation". Default is 1.
        """
        reset_mode=1
        
        """
        Set the step mode of Gazebo. 1 is "using ROS services", 2 is "using step function of Gazebo". Default is 1.
        If using the step mode 2 then set the number of steps of Gazebo to take in each episode. Default is 1.
        """
        step_mode=1
        num_gazebo_steps=1 

        """
        Init the parent class with the corresponding variables.
        """
        super(CustomRobotEnv, self).__init__(   launch_gazebo=launch_gazebo, gazebo_init_paused=gazebo_init_paused, 
                    gazebo_use_gui=gazebo_use_gui, gazebo_recording=gazebo_recording, gazebo_freq=gazebo_freq, world_path=world_path, 
                    world_pkg=world_pkg, world_filename=world_filename, gazebo_max_freq=gazebo_max_freq, gazebo_timestep=gazebo_timestep,
                    spawn_robot=spawn_robot, model_name_in_gazebo=model_name_in_gazebo, namespace=namespace, pkg_name=pkg_name, 
                    urdf_file=urdf_file, urdf_folder=urdf_folder, controller_file=controller_file, controller_list=controller_list, 
                    urdf_xacro_args=urdf_xacro_args, rob_state_publisher_max_freq= rob_state_publisher_max_freq,
                    model_pos_x=model_pos_x, model_pos_y=model_pos_y, model_pos_z=model_pos_z, 
                    model_ori_x=model_ori_x, model_ori_y=model_ori_y, model_ori_z=model_ori_z, model_ori_w=model_ori_w,
                    reset_controllers=reset_controllers, reset_mode=reset_mode, step_mode=step_mode, num_gazebo_steps=num_gazebo_steps)

        """
        Define publisher or subscribers as needed.
        """
        # self.pub1 = rospy.Publisher('/robot/controller_manager/command', JointState, queue_size=1)
        # self.sub1 = rospy.Subscriber('/robot/joint_states', JointState, self.callback1)

        """
        If using the __check_subs_and_pubs_connection method, then un-comment the lines below.
        """
        # ros_gazebo.gazebo_unpause_physics()
        # self._check_subs_and_pubs_connection()
        # ros_gazebo.gazebo_pause_physics()

        """
        Finished __init__ method
        """
        rospy.loginfo("Finished Init of Custom Robot env")

    #------------------------------------------#
    #   Custom methods for the CustomRobotEnv  #

    def _check_subs_and_pubs_connection(self):
        """
        Function to check if the Gazebo and ROS connections are ready
        """
        return True

    #-------------------------------------------------------#
    #   Custom available methods for the CustomRobotEnv     #
    #   Although it is best to implement these methods in   #
    #   the Task Env, one can use them here if needed.      #

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

    def _set_episode_init_params(self):
        """
        Function to set some parameters, like the position of the robot, at the begining of each episode.
        """
        raise NotImplementedError()