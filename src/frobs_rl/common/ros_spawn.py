#!/bin/python3

import rospy
import time
from frobs_rl.common import ros_gazebo
from frobs_rl.common import ros_controllers
from frobs_rl.common import ros_node
from frobs_rl.common import ros_params
from frobs_rl.common import ros_urdf

def init_robot_state_pub(namespace="/", max_pub_freq=None, launch_new_term=False) -> bool:
    """
    Funtion to initialize the robot state publisher.

    :param namespace: Namespace of the robot.
    :type namespace: str

    :param max_pub_freq: Maximum frequency of the publisher.
    :type max_pub_freq: float

    :param launch_new_term: Launch the process in a new terminal (Xterm).
    :type launch_new_term: bool

    :return: Return true if the publisher was initialized.
    :rtype: bool
    """

    if max_pub_freq is not None:
        if namespace != "/":
            rospy.set_param(namespace + "/rob_st_pub/publish_frequency", max_pub_freq)
        else:
            rospy.set_param("/rob_st_pub/publish_frequency", max_pub_freq)
            
    return ros_node.ros_node_from_pkg("robot_state_publisher", "robot_state_publisher", launch_new_term=launch_new_term, name="rob_st_pub", ns=namespace)


def spawn_model_in_gazebo(  pkg_name, model_urdf_file, 
                            controllers_file, controllers_list=[],
                            model_urdf_folder="/urdf", ns="/", args_xacro=None, max_pub_freq=None, rob_st_term=False,
                            gazebo_name="robot1", gaz_ref_frame="world", 
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, ori_w=0.0, ori_x=0.0, ori_y=0.0, ori_z=0.0):

    """
    Function to spawn a model in gazebo.

    :param pkg_name: Package name of the model.
    :type pkg_name: str

    :param model_urdf_file: Name of the model urdf file.
    :type model_urdf_file: str

    :param controllers_file: Name of the controllers file. If None then no controllers will be loaded.
    :type controllers_file: str

    :param controllers_list: List of the controllers to be loaded.
    :type controllers_list: list

    :param model_urdf_folder: Folder where the model urdf file is located. Default is "/urdf".
    :type model_urdf_folder: str

    :param ns: Namespace of the model. Default is "/".
    :type ns: str

    :param args_xacro: Arguments to be passed to xacro.
    :type args_xacro: list

    :param max_pub_freq: Maximum frequency of the robot state publisher.
    :type max_pub_freq: float

    :param rob_st_term: Launch the robot state publisher in a new terminal (Xterm).
    :type rob_st_term: bool

    :param gazebo_name: Name of the gazebo model.
    :type gazebo_name: str

    :param gaz_ref_frame: Reference frame of the gazebo model.
    :type gaz_ref_frame: str

    :param pos_x: X position of the gazebo model.
    :param pos_y: Y position of the gazebo model.
    :param pos_z: Z position of the gazebo model.
    :type pos_x: float
    :type pos_y: float
    :type pos_z: float

    :param ori_w: W orientation of the gazebo model.
    :param ori_x: X orientation of the gazebo model.
    :param ori_y: Y orientation of the gazebo model.
    :param ori_z: Z orientation of the gazebo model.
    :type ori_w: float
    :type ori_x: float
    :type ori_y: float
    :type ori_z: float

    :return: Return true if the model was spawned.
    :rtype: bool

    """

    # Load the model URDF in the parameter server
    if ros_urdf.urdf_load_from_pkg(pkg_name, model_urdf_file, "robot_description", folder=model_urdf_folder, ns=ns, args_xacro=args_xacro):
        rospy.logwarn("URDF file loaded successfully")
    else:
        rospy.logwarn("Error while loading URDF file")
        return False
    
    time.sleep(0.1)

    # Initialize the Robot State Publisher
    if init_robot_state_pub(namespace=ns, max_pub_freq=max_pub_freq, launch_new_term=rob_st_term):
        rospy.logwarn("Robot state publisher initialized")
    else:
        rospy.logwarn("Error while initializing robot state publisher")
        return False

    time.sleep(0.1)

    # Spawn the model in gazebo
    result_spawn, message = ros_gazebo.gazebo_spawn_urdf_param("robot_description", model_name=gazebo_name, robot_namespace=ns, reference_frame=gaz_ref_frame,
                                        pos_x=pos_x, pos_y=pos_y, pos_z=pos_z, ori_w=ori_w, ori_x=ori_x, ori_y=ori_y, ori_z=ori_z,)
    if result_spawn:
        rospy.logwarn("Model spawned successfully")
    else:
        rospy.logwarn("Error while spawning model")
        rospy.logwarn(message)
        return False

    time.sleep(0.1)

    if controllers_file is not None:
        # Load the robot controllers from YAML files in the parameter server
        if ros_params.ros_load_yaml_from_pkg(pkg_name, controllers_file, ns=ns):
            rospy.logwarn("Robot controllers loaded successfully")
        else:
            rospy.logwarn("Error while loading robot controllers")
            return False

        time.sleep(0.1)

        # Spawn the controllers
        if ros_controllers.spawn_controllers_srv(controllers_list, ns=ns):
            rospy.logwarn("Controllers spawned successfully")
        else:
            rospy.logwarn("Error while spawning controllers")
            return False
    
    return True
