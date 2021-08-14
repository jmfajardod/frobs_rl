#!/bin/python3

import rospy
import rospkg
import os
import subprocess

def ROS_Launch_from_pkg(pkg_name, launch_file, args=None) -> bool:
    """
    Function to execute a roslaunch from package with args.
    @param pkg_name: Name of the package where the launch file is located.
    @type pkg_name: str
    
    @param launch_file: Name of the launch file.
    @type launch_file: str
    
    @param args: Args to pass to the launch file.
    @type args: str

    @return: True if the launch file was executed.
    """

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return False

    file_path = pkg_path + "/launch/" + launch_file
    if os.path.exists(pkg_path + "/launch/" + launch_file) is False:
        print("Launch file " + launch_file + " in " + file_path + " does not exists")
        return False

    term_command = "roslaunch " + pkg_name + " " + launch_file

    if args is not None:
        term_command = term_command + " " + args

    subprocess.Popen(term_command, shell=True)
    return True

def ROS_Launch_from_path(launch_file_path, args=None) -> bool:
    """
    Function to execute a roslaunch from a path with args.
    @param launch_file_path: Path of the launch file.
    @type launch_file_path: str

    @param args: Args to pass to the launch file.
    @type args: str

    @return: True if the launch file was executed.
    """

    if os.path.exists(launch_file_path) is False:
        print("Launch file " + launch_file_path + " does not exists")
        return False

    term_command = "roslaunch " + launch_file_path

    if args is not None:
        term_command = term_command + " " + args

    subprocess.Popen(term_command, shell=True)
    return True

def ROS_Kill_Launch_Process() -> bool:
    """
    Function to kill all roslaunch processes.

    @return: True if the roslaunch processes were killed.
    """
    term_command = "killall -9 roslaunch"
    subprocess.Popen(term_command, shell=True).wait()
    return True