#!/bin/python3

import rospy
import rospkg
import os
import rosparam

def ros_load_yaml_from_pkg(pkg_name, file_name, ns='/') -> bool:
    """
    Fetch a YAML file from a package and load it into the ROS Parameter Server.

    :param pkg_name: name of package.
    :type  pkg_name: str

    :param file_name: name of file.
    :type  file_name: str

    :param ns: namespace to load parameters into.
    :type  ns: str

    :return: True if file was loaded, false otherwise.
    :rtype: bool
    """

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return False

    file_path = pkg_path + "/config/" + file_name
    if os.path.exists(pkg_path + "/config/" + file_name) is False:
        print("Config file " + file_name + " in " + file_path + " does not exists")
        return False

    paramlist=rosparam.load_file(file_path)
    
    for params, namespace in paramlist:
        rosparam.upload_params(ns,params)

    return True

def ros_load_yaml_from_path(file_path, ns='/') -> bool:
    """
    Fetch a YAML file from a path and load it into the ROS Parameter Server.

    :param file_path: path to file.
    :type  file_path: str

    :param ns: namespace to load parameters into.
    :type  ns: str

    :return: True if file was loaded, false otherwise.
    :rtype: bool
    """

    if os.path.exists(file_path) is False:
        print("Config file " + file_path + " does not exists")
        return False

    paramlist=rosparam.load_file(file_path)
    
    for params, namespace in paramlist:
        rosparam.upload_params(ns,params)

    return True
