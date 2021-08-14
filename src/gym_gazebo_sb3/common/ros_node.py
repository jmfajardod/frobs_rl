#!/bin/python3

import rospy
import rospkg
import os
import subprocess
import time


def ROS_Node_from_pkg(pkg_name, node_name, launch_master=False, name=None, ns=None) -> bool:
    """
    Function to launch a ROS node from a package.
    @param pkg_name: Package name.
    @type pkg_name: str

    @param node_name: Node executable name.
    @type node_name: str

    @param launch_master: If ROSMASTER is not running launch it.
    @type launch_master: bool

    @param name: Name to give the node to be launched.
    @type name: str

    @param ns: Namespace to give the node to be launched.
    @type ns: str

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
        print("Launch Master")
        
        try:
            rospy.get_master().getPid()
        except:
            print("Master not running")
            subprocess.Popen("roscore", shell=True)
        else:
            print("Master is running")

    term_command = "rosrun " + pkg_name + " " + node_name

    if name is not None:
        term_command += " __name:=" + name

    if ns is not None:
        term_command += " __ns:=" + ns

    subprocess.Popen(term_command, shell=True)
    return True

def ROS_Node_from_path(node_path, launch_master=True, name=None, ns=None) -> bool:
    """
    Function to launch a ROS node from a path.
    @param node_path: Path to the node executable.
    @type node_path: str

    @param launch_master: If ROSMASTER is not running launch it.
    @type launch_master: bool

    @param name: Name to give the node to be launched.
    @type name: str

    @param ns: Namespace to give the node to be launched.
    @type ns: str

    @return: True if the node was launched, False otherwise.
    """

    if os.path.exists(node_path) is False:
        print("Node " + node_path + " does not exists")
        return False

    if launch_master:
        print("Launch Master")
        
        try:
            rospy.get_master().getPid()
        except:
            print("Master not running")
            subprocess.Popen("roscore", shell=True)
        else:
            print("Master is running")

    term_command = "rosrun " + node_path

    if name is not None:
        term_command += " __name:=" + name

    if ns is not None:
        term_command += " __ns:=" + ns

    subprocess.Popen(term_command, shell=True)
    return True

def ROS_Kill_Node(node_name) -> bool:
    """
    Function to kill a ROS node.

    @param node_name: Name of the node to kill.
    @type node_name: str

    @return: True if the node was killed, False otherwise.
    """

    term_command = "rosnode kill " + node_name
    subprocess.Popen(term_command, shell=True).wait()
    return True

def ROS_Kill_All_Nodes() -> bool:
    """
    Function to kill all running ROS nodes.

    @return: True if all nodes were killed, False otherwise.
    """

    term_command = "rosnode kill -a"
    subprocess.Popen(term_command, shell=True).wait()
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
        subprocess.Popen(term_command, shell=True).wait()
        time.sleep(0.5)
        term_command = "killall -9 rosout roslaunch rosmaster nodelet"
        subprocess.Popen(term_command, shell=True).wait()

        return True

def ROS_Kill_All_processes() -> bool:
    """
    Function to kill all running ROS related processes.

    @return: True if all processes were killed, False otherwise.
    """

    term_command = "killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient"
    subprocess.Popen(term_command, shell=True).wait()
    return True
    

