#!/bin/python3

import rospy
import rospkg
import os
import xacro

def urdf_load_from_pkg(pkg_name, model_name, param_name, folder="/urdf", ns=None, args_xacro=None) -> bool:
    """
    Function to load a URDF from a ROS package to the parameter server.

    :param pkg_name: The ROS package name.
    :type pkg_name: str

    :param model_name: The model file name.
    :type model_name: str

    :param param_name: The parameter name.
    :type param_name: str

    :param folder: The folder where the model is located. Default: "/urdf"
    :type folder: str

    :param ns: The namespace of the parameter.
    :type ns: str

    :param args_xacro: The xacro arguments in a list. Eg: ['arg1:=True','arg2:=10.0']
    :type args_xacro: list of str.

    :return: True if the URDF was loaded, False otherwise.
    :rtype: bool
    """

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return False

    file_path = pkg_path + folder + "/" + model_name

    if os.path.exists(file_path) is False:
        print("Error: model path does not exist")
        return False

    list_args = [file_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)
    
    if ns is not None and ns != "/":
        final_param_name = ns + "/" + param_name
    else:
        final_param_name = param_name

    rospy.set_param(final_param_name, model_string)
    return True

def urdf_load_from_path(model_path, param_name, ns=None, args_xacro=None) -> bool:
    """
    Function to load a URDF from a file to the parameter server.

    :param model_path: The model file path.
    :type model_path: str

    :param param_name: The parameter name.
    :type param_name: str

    :param ns: The namespace of the parameter.
    :type ns: str

    :param args_xacro: The xacro arguments in a list. Eg: ['arg1:=True','arg2:=10.0']
    :type args_xacro: list of str.

    :return: True if the URDF was loaded, False otherwise.
    :rtype: bool
    """

    if os.path.exists(model_path) is False:
        print("Error: model path does not exist")
        return False

    list_args = [model_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    if ns is not None and ns != "/":
        final_param_name = ns + "/" + param_name
    else:
        final_param_name = param_name

    rospy.set_param(final_param_name, model_string)
    return True

def urdf_parse_from_pkg(pkg_name, model_name, folder="/urdf", args_xacro=None) -> str:
    """
    Function to parse a URDF from a ROS package and return the URDF string.

    :param pkg_name: The ROS package name.
    :type pkg_name: str

    :param model_name: The model file name.
    :type model_name: str

    :param folder: The folder where the model is located. Default: "/urdf"
    :type folder: str

    :param args_xacro: The xacro arguments in a list. Eg: ['arg1:=True','arg2:=10.0']
    :type args_xacro: list of str.

    :return: The URDF string or None if the pkg or file was not found.
    :rtype: str
    """
    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return None

    file_path = pkg_path + folder + "/" + model_name

    if os.path.exists(file_path) is False:
        print("Error: model path does not exist")
        return None

    list_args = [file_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    return model_string

def urdf_parse_from_path(model_path, args_xacro=None) -> bool:
    """
    Function to parse a URDF from a file and return the URDF string.

    :param model_path: The model file path.
    :type model_path: str

    :param args_xacro: The xacro arguments in a list. Eg: ['arg1:=True','arg2:=10.0']
    :type args_xacro: list of str.

    :return: The URDF string or None if the file was not found.
    :rtype: str
    """

    if os.path.exists(model_path) is False:
        print("Error: model path does not exist")
        return None

    list_args = [model_path] 
    if args_xacro is not None:
        list_args = list_args + args_xacro

    opts, input_file_name = xacro.process_args(list_args)
    model = xacro.process_file(input_file_name, **vars(opts))
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    return model_string
