#!/bin/python3

import rospy
import rospkg
import subprocess
import time 

"""
Functions related to the handling of ROS nodes.
"""

def ros_node_from_pkg(pkg_name, node_name, launch_master=False, launch_master_term=True, launch_new_term=True, name=None, ns="/", output="log") -> bool:
    """
    Function to launch a ROS node from a package.

    :param pkg_name: Name of the package to launch the node from.
    :type pkg_name: str

    :param node_name: Name of the node to launch.
    :type node_name: str

    :param launch_master: If ROSMASTER is not running launch it.
    :type launch_master: bool

    :param launch_master_term: If launch ROSMASTER do it in an external terminal.
    :type launch_master_term: bool

    :param launch_new_term: Launch the process in a new terminal (Xterm).
    :type launch_new_term: bool

    :param name: Name to give the node to be launched.
    :type name: str

    :param ns: Namespace to give the node to be launched.
    :type ns: str

    :param output:  log, screen, or None.
    :type output: str

    :return: True if the node was launched, False otherwise.
    :rtype: bool
    """

    rospack = rospkg.RosPack()
    try:
        rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return False

    if launch_master:
        print("Launching Master")
        try:
            rospy.get_master().getPid()
        except ConnectionRefusedError:
            print("Master not running")
            if launch_master_term:
                subprocess.Popen("xterm -e 'roscore' ", shell=True)
            else:
                subprocess.Popen("roscore", shell=True)
            time.sleep(5.0)
        else:
            print("Master is running")

    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        print("Master not running")
        return False
    else:
        print("Master is running")

    term_command = "rosrun " + pkg_name + " " + node_name

    if name is not None:
        term_command += " __name:=" + str(name)

    term_command += " __ns:=" + str(ns)
    term_command += " __log:=" + str(output)

    if launch_new_term:
        term_command = "xterm -e ' " + term_command + "'"
    
    subprocess.Popen(term_command, shell=True)
    time.sleep(5.0)

    return True

def ros_kill_node(node_name) -> bool:
    """
    Function to kill a ROS node.

    :param node_name: Name of the node to kill.
    :type node_name: str

    :return: True if the node was killed, False otherwise.
    :rtype: bool
    """

    term_command = "rosnode kill " + node_name
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True

def ros_kill_all_nodes() -> bool:
    """
    Function to kill all running ROS nodes.

    :return: True if all nodes were killed, False otherwise.
    :rtype: bool
    """

    term_command = "rosnode kill -a"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True

def ros_kill_master() -> bool:
    """
    Function to kill the ROS master.

    :return: True if the master was killed, False otherwise.
    :rtype: bool
    """

    try:
        rospy.get_master().getPid()
    except ConnectionRefusedError:
        print("Master not running")
        return True
    else:
        print("Master is running")
        term_command = "rosnode kill -a"
        subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
        time.sleep(0.5)
        term_command = "killall -9 rosout roslaunch rosmaster nodelet"
        subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()

        return True

def ros_kill_all_processes() -> bool:
    """
    Function to kill all running ROS related processes.

    :return: True if all processes were killed, False otherwise.
    :rtype: bool
    """

    term_command = "killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True
