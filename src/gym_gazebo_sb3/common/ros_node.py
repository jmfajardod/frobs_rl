#!/bin/python3

from rosgraph.names import namespace
import rospy
import rospkg
import os
import subprocess
import roslaunch
import time


def ROS_Node_from_pkg(pkg_name, node_name, launch_master=False, name=None, ns="/", output="log") -> bool:
    """
    Function to launch a ROS node from a package.

    @param pkg_name: Name of the package to launch the node from.
    @type pkg_name: str

    @param node_name: Name of the node to launch.
    @type node_name: str

    @param launch_master: If ROSMASTER is not running launch it.
    @type launch_master: bool

    @param name: Name to give the node to be launched.
    @type name: str

    @param ns: Namespace to give the node to be launched.
    @type ns: str

    @param output:  log, screen, or None.
    @type output: str

    @return: True if the node was launched, False otherwise.
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
        except:
            print("Master not running")
            subprocess.Popen("xterm -e 'roscore' ", shell=True)
            time.sleep(5.0)
        else:
            print("Master is running")

    try:
        rospy.get_master().getPid()
    except:
        print("Master not running")
        return False
    else:
        print("Master is running")

    term_command = "rosrun " + pkg_name + " " + node_name

    if name is not None:
        term_command += " __name:=" + str(name)

    term_command += " __ns:=" + str(ns)
    term_command += " __log:=" + str(output)

    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True)
    time.sleep(5.0)

    return True

def ROS_Kill_Node(node_name) -> bool:
    """
    Function to kill a ROS node.

    @param node_name: Name of the node to kill.
    @type node_name: str

    @return: True if the node was killed, False otherwise.
    """

    term_command = "rosnode kill " + node_name
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True

def ROS_Kill_All_Nodes() -> bool:
    """
    Function to kill all running ROS nodes.

    @return: True if all nodes were killed, False otherwise.
    """

    term_command = "rosnode kill -a"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True

def ROS_Kill_Master() -> bool:
    """
    Function to kill the ROS master.

    @return: True if the master was killed, False otherwise.
    """

    try:
        rospy.get_master().getPid()
    except:
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

def ROS_Kill_All_processes() -> bool:
    """
    Function to kill all running ROS related processes.

    @return: True if all processes were killed, False otherwise.
    """

    term_command = "killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    return True
