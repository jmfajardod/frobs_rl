#!/bin/python3

import rospy
from controller_manager_msgs.srv import *

def load_controller_srv(controller_name, ns=None, max_retries=5) -> bool:
    """
    Function to load a controller on the namespace.

    :param controller_name: name of the controller to load
    :type controller_name: string

    :param ns: namespace
    :type ns: string

    :param max_retries: number of retries to load the controller.
    :type max_retries: int

    :return: true if the controller is loaded.
    :rtype: bool
    """

    if ns is not None:
        srv_name = ns + '/controller_manager/load_controller'
    else:
        srv_name = '/controller_manager/load_controller'

    rospy.wait_for_service(srv_name)
    client_srv = rospy.ServiceProxy(srv_name, LoadController)
    
    try: 
        for ii in range(max_retries):
            srv_request = LoadControllerRequest(name=controller_name)
            resp1 = client_srv(srv_request)
            if resp1.ok:
                return True
            
        if resp1.ok is False:
            print("Controller " + controller_name + " could not be loaded.")
        
        return resp1.ok
    
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        return False

def load_controller_list_srv(controller_list,ns=None, max_retries=5) -> None:
    """
    Function to load a list of controllers on the namespace.

    :param controller_list: list of controllers to load
    :type controller_list: list of strings

    :param ns: namespace
    :type ns: string

    :param max_retries: number of retries to load the controller.
    :type max_retries: int
    """

    for controller in controller_list:
        load_controller_srv(controller, ns=ns, max_retries=max_retries)

def unload_controller_srv(controller_name,ns=None, max_retries=5) -> bool:
    """
    Function to unload a controller on the namespace.

    :param controller_name: name of the controller to unload
    :type controller_name: string

    :param ns: namespace
    :type ns: string

    :param max_retries: number of retries to unload the controller.
    :type max_retries: int

    :return: true if the controller is unloaded.
    :rtype: bool
    """

    if ns is not None:
        srv_name = ns + '/controller_manager/unload_controller'
    else:
        srv_name = '/controller_manager/unload_controller'

    rospy.wait_for_service(srv_name)
    client_srv = rospy.ServiceProxy(srv_name, UnloadController)
    
    try: 
        for ii in range(max_retries):
            srv_request = UnloadControllerRequest(name=controller_name)
            resp1 = client_srv(srv_request)
            if resp1.ok:
                return True

        if resp1.ok is False:
            print("Controller " + controller_name + " could not be unloaded.")    
        
        return resp1.ok

    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        return False

def unload_controller_list_srv(controller_list,ns=None, max_retries=5) -> None:
    """
    Function to unload a list of controllers on the namespace.

    :param controller_list: list of controllers to unload
    :type controller_list: list of strings

    :param ns: namespace
    :type ns: string

    :param max_retries: number of retries to unload the controller.
    :type max_retries: int
    """
    for controller in controller_list:
        unload_controller_srv(controller, ns=ns, max_retries=max_retries)

def switch_controllers_srv( start_controllers, stop_controllers, ns=None, 
                            strictness=1, start_asap=False, timeout=3.0, max_retries=5) -> bool:
    """
    Function to switch controllers on the namespace.

    :param start_controllers: list of controllers to start
    :type start_controllers: list of strings

    :param stop_controllers: list of controllers to stop
    :type stop_controllers: list of strings

    :param ns: namespace
    :type ns: string

    :param strictness: strictness of the controller manager: BEST_EFFORT or STRICT (1 and 2, respectively).
    :type strictness: int

    :param start_asap:  start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise.
    :type start_asap: bool

    :param timeout: the timeout in seconds before aborting pending controllers. Zero for infinite.
    :type timeout: float

    :param max_retries: number of retries to switch the controller.
    :type max_retries: int

    :return: true if the operation is successful.
    :rtype: bool
    """

    if ns is not None:
        srv_name = ns + '/controller_manager/switch_controller'
    else:
        srv_name = '/controller_manager/switch_controller'

    rospy.wait_for_service(srv_name)
    client_srv = rospy.ServiceProxy(srv_name, SwitchController)
    
    try: 
        for ii in range(max_retries):
            srv_request = SwitchControllerRequest(  start_controllers=start_controllers,stop_controllers=stop_controllers,
                                                    strictness=strictness,start_asap=start_asap,timeout=timeout)
            resp1 = client_srv(srv_request)
            if resp1.ok:
                return True
        
        if resp1.ok is False:
            print("Controllers could not be switched.")

        return resp1.ok
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
        return False

def start_controllers_srv(start_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:
    """
    Function to start controllers on the namespace.

    :param start_controllers: list of controllers to start
    :type start_controllers: list of strings

    :param ns: namespace
    :type ns: string

    :param strictness: strictness of the controller manager: BEST_EFFORT or STRICT (1 and 2, respectively).
    :type strictness: int

    :param start_asap:  start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise.
    :type start_asap: bool

    :param timeout: the timeout in seconds before aborting pending controllers. Zero for infinite.
    :type timeout: float

    :return: true if the operation is successful.
    :rtype: bool
    """

    return switch_controllers_srv(start_controllers, [], ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

def stop_controllers_srv(stop_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:
    """
    Function to start controllers on the namespace.

    :param stop_controllers: list of controllers to stop
    :type stop_controllers: list of strings

    :param ns: namespace
    :type ns: string

    :param strictness: strictness of the controller manager: BEST_EFFORT or STRICT (1 and 2, respectively).
    :type strictness: int

    :param start_asap:  start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise.
    :type start_asap: bool

    :param timeout: the timeout in seconds before aborting pending controllers. Zero for infinite.
    :type timeout: float

    :return: true if the operation is successful.
    :rtype: bool
    """

    return switch_controllers_srv([], stop_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

def reset_controllers_srv(reset_controllers, max_retries=10, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:
    """
    Function to reset controllers on the namespace.

    :param reset_controllers: list of controllers to reset
    :type reset_controllers: list of strings

    :param max_retries: number of times to retry resetting a controller before giving up.
    :type max_retries: int

    :param ns: namespace
    :type ns: string

    :param strictness: strictness of the controller manager: BEST_EFFORT or STRICT (1 and 2, respectively).
    :type strictness: int

    :param start_asap:  start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise.
    :type start_asap: bool

    :param timeout: the timeout in seconds before aborting pending controllers. Zero for infinite.
    :type timeout: float

    :return: true if the operation is successful.
    :rtype: bool
    """

    done_switch_off = False
    for ii in range(max_retries):
        done_switch_off = stop_controllers_srv(reset_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)
        if done_switch_off:
            break
    
    if not done_switch_off:
        return False

    done_switch_on = False
    for ii in range(max_retries):
        done_switch_on = start_controllers_srv(reset_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)
        if done_switch_on:
            break

    if not done_switch_on:
        return False
    
    return True

def spawn_controllers_srv(spawn_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:
    """
    Function to spawn controllers on the namespace.

    :param spawn_controllers: list of controllers to spawn
    :type spawn_controllers: list of strings

    :param ns: namespace
    :type ns: string

    :param strictness: strictness of the controller manager: BEST_EFFORT or STRICT (1 and 2, respectively).
    :type strictness: int

    :param start_asap:  start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise.
    :type start_asap: bool

    :param timeout: the timeout in seconds before aborting pending controllers. Zero for infinite.
    :type timeout: float

    :return: true if the operation is successful.
    :rtype: bool
    """

    load_controller_list_srv(spawn_controllers, ns=ns)

    return start_controllers_srv(spawn_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

def kill_controllers_srv(kill_controllers, ns=None, strictness=1, start_asap=False, timeout=3.0) -> bool:
    """
    Function to kill controllers on the namespace.

    :param kill_controllers: list of controllers to kill
    :type kill_controllers: list of strings

    :param ns: namespace
    :type ns: string

    :param strictness: strictness of the controller manager: BEST_EFFORT or STRICT (1 and 2, respectively).
    :type strictness: int

    :param start_asap:  start the controllers as soon as their hardware dependencies are ready, will wait for all interfaces to be ready otherwise.
    :type start_asap: bool

    :param timeout: the timeout in seconds before aborting pending controllers. Zero for infinite.
    :type timeout: float

    :return: true if the operation is successful.
    :rtype: bool
    """

    res = stop_controllers_srv(kill_controllers, ns=ns, strictness=strictness, start_asap=start_asap, timeout=timeout)

    if res:
        unload_controller_list_srv(kill_controllers, ns=ns)
        return True
    else:
        return False