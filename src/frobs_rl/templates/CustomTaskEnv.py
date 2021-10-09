#!/bin/python3

from gym import spaces
from gym.envs.registration import register
from frobs_rl.templates import CustomRobotEnv # Replace with your own robot env
import rospy

#- Uncomment the library modules as neeeed
# from frobs_rl.common import ros_gazebo
# from frobs_rl.common import ros_controllers
# from frobs_rl.common import ros_node
# from frobs_rl.common import ros_launch
# from frobs_rl.common import ros_params
# from frobs_rl.common import ros_urdf
# from frobs_rl.common import ros_spawn

register(
        id='CustomTaskEnv-v0',
        entry_point='frobs_rl.templates.CustomTaskEnv:CustomTaskEnv',
        max_episode_steps=10000,
    )

class CustomTaskEnv(CustomRobotEnv.CustomRobotEnv):
    """
    Custom Task Env, use this env to implement a task using the robot defined in the CustomRobotEnv
    """

    def __init__(self):
        """
        Describe the task.
        """
        rospy.loginfo("Starting Task Env")

        """
        Init super class.
        """
        super(CustomTaskEnv, self).__init__()

        """
        Define the action and observation space.
        """
        # self.action_space = spaces.Discrete(n_actions)
        # self.action_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

        # self.observation_space = spaces.Discrete(n_observations)
        # self.observation_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

        """
        Define subscribers or publishers as needed.
        """
        # self.pub1 = rospy.Publisher('/robot/controller_manager/command', JointState, queue_size=1)
        # self.sub1 = rospy.Subscriber('/robot/joint_states', JointState, self.callback1)

        """
        Finished __init__ method
        """
        rospy.loginfo("Finished Init of Custom Task env")

    #-------------------------------------------------------#
    #   Custom available methods for the CustomTaskEnv      #

    def _set_episode_init_params(self):
        """
        Function to set some parameters, like the position of the robot, at the beginning of each episode.
        """
        raise NotImplementedError()

    def _send_action(self, action):
        """
        Function to send an action to the robot
        """
        raise NotImplementedError()
        

    def _get_observation(self):
        """
        Function to get the observation from the environment.
        """
        raise NotImplementedError()

    def _get_reward(self):
        """
        Function to get the reward from the environment.
        """
        raise NotImplementedError()
    
    def _check_if_done(self):
        """
        Function to check if the episode is done.
        
        If the episode has a success condition then set done as:
            self.info['is_success'] = 1.0
        """
        raise NotImplementedError()
