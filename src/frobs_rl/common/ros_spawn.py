#!/bin/python3

import rospy
import time
from frobs_rl.common import ros_gazebo
from frobs_rl.common import ros_controllers
from frobs_rl.common import ros_node
from frobs_rl.common import ros_params
from frobs_rl.common import ros_urdf
from frobs_rl.common import ros_launch

def Init_robot_state_pub(namespace="/", max_pub_freq=None, launch_new_term=False) -> bool:
    """
    Funtion to initialize the robot state publisher.

    @param namespace: Namespace of the robot.
    @type namespace: str

    @param max_pub_freq: Maximum frequency of the publisher.
    @type max_pub_freq: float

    @param launch_new_term: Launch the process in a new terminal (Xterm).
    @type launch_new_term: bool

    @return: Return true if the publisher was initialized.
    """

    if max_pub_freq is not None:
        if namespace != "/":
            max_pub_freq = rospy.set_param(namespace + "/rob_st_pub/publish_frequency", max_pub_freq)
        else:
            max_pub_freq = rospy.set_param("/rob_st_pub/publish_frequency", max_pub_freq)

    return ros_node.ROS_Node_from_pkg("robot_state_publisher", "robot_state_publisher", launch_new_term=launch_new_term, name="rob_st_pub", ns=namespace)


def Spawn_model_in_gazebo(  pkg_name, model_urdf_file,
                            controllers_file, controllers_list,
                            ns="/", args_xacro=None, max_pub_freq=None,
                            gazebo_name="robot1", gaz_ref_frame="world",
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, ori_w=0.0, ori_x=0.0, ori_y=0.0, ori_z=0.0):

    # Load the model URDF in the parameter server
    if ros_urdf.URDF_load_from_pkg(pkg_name, model_urdf_file, "robot_description", ns=ns, args_xacro=args_xacro):
        rospy.loginfo("URDF file loaded successfully")
    else:
        rospy.loginfo("Error while loading URDF file")
        return False

    time.sleep(0.1)

    # Initialize the Robot State Publisher
    if Init_robot_state_pub(namespace=ns, max_pub_freq=max_pub_freq):
        rospy.loginfo("Robot state publisher initialized")
    else:
        rospy.loginfo("Error while initializing robot state publisher")
        return False

    time.sleep(0.1)

    # Spawn the model in gazebo
    if ros_gazebo.Gazebo_spawn_urdf_param("robot_description", model_name=gazebo_name, robot_namespace=ns, reference_frame=gaz_ref_frame,
                                        pos_x=pos_x, pos_y=pos_y, pos_z=pos_z, ori_w=ori_w, ori_x=ori_x, ori_y=ori_y, ori_z=ori_z,):
        rospy.loginfo("Model spawned successfully")
    else:
        rospy.loginfo("Error while spawning model")
        return False

    time.sleep(0.1)

    # Load the robot controllers from YAML files in the parameter server
    if ros_params.ROS_Load_YAML_from_pkg(pkg_name, controllers_file, ns=ns):
        rospy.loginfo("Robot controllers loaded successfully")
    else:
        rospy.loginfo("Error while loading robot controllers")
        return False

    time.sleep(0.1)

    # Spawn the controllers
    if ros_controllers.Spawn_controllers_srv(["joint_state_controller", "arm120_controller"], ns=ns):
        rospy.loginfo("Controllers spawned successfully")
    else:
        rospy.loginfo("Error while spawning controllers")
        return False

    return True

#Spawn_model_in_gazebo( "abb_irb120", "irb120.urdf.xacro", "irb120_pos_controller.yaml", ["joint_state_controller", "arm120_controller"],
#                        ns="/robot1", gazebo_name="robot1", pos_y=0.0)
