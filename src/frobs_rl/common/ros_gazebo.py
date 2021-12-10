#!/bin/python3

import rospy
import rospkg
import os
import subprocess
import time
import xacro
from rospy.service import ServiceException

from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.srv import DeleteModelRequest, SpawnModelRequest
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.srv import GetModelStateRequest, SetModelStateRequest
from gazebo_msgs.msg import ODEPhysics, ModelState
from geometry_msgs.msg import Vector3
from std_srvs.srv import Empty
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Twist


def launch_Gazebo(  paused=False, use_sim_time=True, gui=True, recording=False, debug=False, verbose=False, output='screen',
                    custom_world_path=None, custom_world_pkg=None, custom_world_name=None, respawn_gazebo=False,
                    pub_clock_frequency=100, server_required=False, gui_required=False, launch_new_term=True) -> bool:
    """
    Launch Gazebo using the ROS network.
    
    :param paused: if True, init gzserver paused.
    :type paused: bool

    :param use_sim_time: if True, use the simulation time.
    :type use_sim_time: bool

    :param gui: if True, launch Gazebo with the GUI (gzclient).
    :type gui: bool

    :param recording: if True, record the data from gazebo.
    :type recording: bool

    :param debug: if True, use the debug configuration.
    :type debug: bool

    :param verbose: if True, print debug information.
    :type verbose: bool

    :param output: choose the output method for gazebo (screen, log).
    :type output: str [screen, log]

    :param custom_world_path: if not None, use this world path.
    :type custom_world_path: str

    :param custom_world_pkg: if the custom_world_path is None, use a world file from this package, specified in custom_world_name.
    :type custom_world_pkg: str

    :param custom_world_name: if the custom_world_path is None, use the world file with this name from the custom_world_pkg.
    :type custom_world_name: str

    :param respawn_gazebo: if True, gazebo will be respawned if it is killed, default is False.
    :type respawn_gazebo: bool

    :param pub_clock_frequency: the frequency of the clock publisher (in Hz)
    :type pub_clock_frequency: int

    :param server_required: if True, the launch file will wait until gzserver is running.
    :type server_required: bool

    :param gui_required: if True, the launch file will wait until gzclient is running.
    :type gui_required: bool

    :param launch_new_term: Launch the gazebo node in a new terminal (Xterm).
    :type launch_new_term: bool

    :return: True if the launch was successful, False otherwise.
    :rtype: bool

    """

    rospack = rospkg.RosPack()
    try:
        rospack.get_path('gazebo_ros')
    except rospkg.common.ResourceNotFound:
        rospy.logerr("The package gazebo_ros was not found")
        return False

    term_command = "roslaunch gazebo_ros empty_world.launch "

    # Init world paused
    term_command += " paused:="+str(paused)

    # Use sim time
    term_command += " use_sim_time:="+str(use_sim_time)

    # Enable/Disable gui
    term_command += " gui:="+str(gui)

    # Enable/Disable recording
    term_command += " recording:="+str(recording)

    # Enable/Disable debug
    term_command += " debug:="+str(debug)

    # Enable/Disable verbose
    term_command += " verbose:="+str(verbose)

    # Select output type (screen or log)
    term_command += " output:="+str(output)

    # Select world
    if custom_world_path is not None:
        term_command += " world_name:="+str(custom_world_path)
    elif custom_world_pkg is not None and custom_world_name is not None:
        try:
            world_pkg_path = rospack.get_path(custom_world_pkg)
        except rospkg.common.ResourceNotFound:
            rospy.logwarn("Package where world file is located was NOT FOUND")

        world_file_path = world_pkg_path + "/worlds/" + custom_world_name
        term_command += " world_name:="+str(world_file_path)

    # Enable re-spawning of gazebo 
    term_command += " respawn_gazebo:="+str(respawn_gazebo)

    # Set gazebo_ros publish rate
    term_command += " pub_clock_frequency:="+str(pub_clock_frequency)

    # Set if gazebo server is required
    term_command += " server_required:="+str(server_required)

    # Set if gui is required
    term_command += " gui_required:="+str(gui_required)

    print(term_command)
        
    # Execute command
    if launch_new_term:
        term_command = "xterm -e ' " + term_command + "'"
    
    subprocess.Popen(term_command, shell=True)
    time.sleep(5.0)

    rospy.wait_for_service('/gazebo/pause_physics')

    return True


def close_Gazebo() -> bool:
    """
    Function to close gazebo if its running.

    :return: True if gazebo was closed, False otherwise.
    :rtype: bool
    """

    term_command = "rosnode kill /gazebo /gazebo_gui"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()
    time.sleep(0.5)

    term_command = "killall -9 gzserver gzclient"
    subprocess.Popen("xterm -e ' " + term_command + "'", shell=True).wait()

    return True

def gazebo_set_max_update_rate(max_update_rate) -> bool:
    """
    Function to set the max update rate for gazebo in real time factor: 1 is real time, 10 is 10 times real time.

    :param max_update_rate: the max update rate for gazebo in real time factor.
    :type max_update_rate: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    rospy.wait_for_service("/gazebo/set_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    orig_physics = client_get_physics()

    new_physics = SetPhysicsPropertiesRequest(  time_step=orig_physics.time_step,
                                                max_update_rate=((1.0/orig_physics.time_step)*max_update_rate),
                                                gravity=orig_physics.gravity,
                                                ode_config=orig_physics.ode_config)
    
    client_set_physics = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
    result_gaz_upd = client_set_physics(new_physics)
    rospy.logdebug("Gazebo physics was succesful: " + str(result_gaz_upd))

    return result_gaz_upd.success

def gazebo_get_max_update_rate() -> float:
    """
    Function to get the current max update rate.

    :return: the max update rate.
    :rtype: float
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    physics = client_get_physics()

    return physics.max_update_rate

def gazebo_set_time_step(new_time_step) -> bool:
    """
    Function to set the time step for gazebo.

    :param new_time_step: the new time step.
    :type new_time_step: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """
    
    rospy.wait_for_service("/gazebo/get_physics_properties")
    rospy.wait_for_service("/gazebo/set_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    orig_physics = client_get_physics()

    max_update_rate = orig_physics.max_update_rate / (1.0/orig_physics.time_step)

    new_physics = SetPhysicsPropertiesRequest(  time_step=new_time_step,
                                                max_update_rate=((1.0/new_time_step)*max_update_rate),
                                                gravity=orig_physics.gravity,
                                                ode_config=orig_physics.ode_config)
    
    client_set_physics = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
    result_gaz_upd = client_set_physics(new_physics)
    rospy.logdebug("Gazebo physics was succesful: " + str(result_gaz_upd))

    return result_gaz_upd.success

def gazebo_get_time_step() -> float:
    """
    Function to get the current time step for gazebo.

    :return: the time step.
    :rtype: float
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    physics = client_get_physics()

    return physics.time_step

def gazebo_set_gravity(x, y, z) -> bool:
    """
    Function to set the gravity for gazebo.

    :param x: the x component of the gravity vector.
    :type x: float

    :param y: the y component of the gravity vector.
    :type y: float

    :param z: the z component of the gravity vector.
    :type z: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    rospy.wait_for_service("/gazebo/set_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    orig_physics = client_get_physics()

    gravity = Vector3()
    gravity.x = x
    gravity.y = y
    gravity.z = z

    new_physics = SetPhysicsPropertiesRequest(  time_step=orig_physics.time_step,
                                                max_update_rate=orig_physics.max_update_rate,
                                                gravity=gravity,
                                                ode_config=orig_physics.ode_config)
    
    client_set_physics = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
    result_gaz_upd = client_set_physics(new_physics)
    rospy.logdebug("Gazebo physics was succesful: " + str(result_gaz_upd))

    return result_gaz_upd.success

def gazebo_get_gravity() -> Vector3:
    """
    Function to get the current gravity vector for gazebo.

    :return: the gravity vector.
    :rtype: Vector3
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    physics = client_get_physics()

    return physics.gravity

def gazebo_set_ODE_physics( auto_disable_bodies, sor_pgs_precon_iters, sor_pgs_iters, sor_pgs_w, sor_pgs_rms_error_tol,
                            contact_surface_layer, contact_max_correcting_vel, cfm, erp, max_contacts) -> bool:
    """
    Function to set the ODE physics for gazebo.

    :return: True if the command was sent and False otherwise.
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    rospy.wait_for_service("/gazebo/set_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    orig_physics = client_get_physics()

    ode_config = ODEPhysics()
    ode_config.auto_disable_bodies = auto_disable_bodies
    ode_config.sor_pgs_precon_iters = sor_pgs_precon_iters
    ode_config.sor_pgs_iters = sor_pgs_iters
    ode_config.sor_pgs_w = sor_pgs_w
    ode_config.sor_pgs_rms_error_tol = sor_pgs_rms_error_tol
    ode_config.contact_surface_layer = contact_surface_layer
    ode_config.contact_max_correcting_vel = contact_max_correcting_vel
    ode_config.cfm = cfm
    ode_config.erp = erp
    ode_config.max_contacts = max_contacts

    new_physics = SetPhysicsPropertiesRequest(  time_step=orig_physics.time_step,
                                                max_update_rate=orig_physics.max_update_rate,
                                                gravity=orig_physics.gravity,
                                                ode_config=orig_physics.ode_config)
    
    client_set_physics = rospy.ServiceProxy("/gazebo/set_physics_properties", SetPhysicsProperties)
    result_gaz_upd = client_set_physics(new_physics)
    rospy.logdebug("Gazebo physics was succesful: " + str(result_gaz_upd))

    return result_gaz_upd.success

def gazebo_get_ODE_physics() -> ODEPhysics:
    """
    Function to get the current ODE physics for gazebo.

    :return: the ODE physics.
    :rtype: ODEPhysics
    """

    rospy.wait_for_service("/gazebo/get_physics_properties")
    client_get_physics = rospy.ServiceProxy("/gazebo/get_physics_properties", GetPhysicsProperties)
    physics = client_get_physics()

    return physics.ode_config

def gazebo_set_default_properties() -> bool:
    """
    Function to set the default gazebo properties.

    :return: True if the command was sent and False otherwise.
    """

    counter_sucess = 0

    if gazebo_set_time_step(0.001):
        counter_sucess += 1
    if gazebo_set_max_update_rate(1.0):
        counter_sucess += 1
    if gazebo_set_gravity(0.0, 0.0, -9.81):
        counter_sucess += 1
    if gazebo_set_ODE_physics(False, 0, 50, 1.3, 0.0, 0.001, 0.0, 0.0, 0.2, 20):
        counter_sucess += 1

    if counter_sucess == 4:
        return True
    else:
        return False

def gazebo_reset_sim(retries=5) -> bool:
    """
    Function to reset the simulation, which reset models to original poses AND reset the simulation time.

    :param retries: The number of times to retry the service call.
    :type retries: int

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/reset_simulation")
    client_srv = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

    for retry in range(retries):
        try:
            client_srv()
            return True
        except rospy.ServiceException as e:
            print ("/gazebo/reset_simulation service call failed")
        
    return False

def gazebo_reset_world(retries=5) -> bool:
    """
    Function to reset the world, which reset models to original poses WITHOUT resetting the simulation time.

    :param retries: The number of times to retry the service call.
    :type retries: int

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/reset_world")
    client_srv = rospy.ServiceProxy("/gazebo/reset_world", Empty)

    for retry in range(retries):
        try:
            client_srv()
            return True
        except rospy.ServiceException as e:
            print ("/gazebo/reset_world service call failed")
        
    return False

def gazebo_pause_physics(retries=5) -> bool:
    """
    Function to pause the physics in the simulation.

    :param retries: The number of times to retry the service call.
    :type retries: int

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/pause_physics")
    client_srv = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

    for retry in range(retries):
        try:
            client_srv()
            return True
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")
        
    return False

def gazebo_unpause_physics(retries=5) -> bool:
    """
    Function to unpause the physics in the simulation.

    :param retries: The number of times to retry the service call.
    :type retries: int

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/unpause_physics")
    client_srv = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    
    for retry in range(retries):
        try:
            client_srv()
            return True
        except rospy.ServiceException as e:
            print ("/gazebo/pause_physics service call failed")
        
    return False

def gazebo_step_physics(steps=1) -> bool:
    """
    Function to step the physics in the simulation.

    :param steps: The number of times to step the simulation.
    :type steps: int

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """
    rospy.wait_for_service("/gazebo/pause_physics")

    term_command = "gz world -m " + str(steps)
    subprocess.Popen(term_command, shell=True).wait()
    return True

def gazebo_delete_model(model_name) -> bool:
    """
    Function to delete a model from the simulation.

    :param model_name: The name of the model to delete.
    :type model_name: str

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/delete_model")
    client_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    srv_model = DeleteModelRequest(model_name=model_name)

    result = client_srv(srv_model)

    return result.success

def gazebo_spawn_urdf_path( model_path, model_name="robot1", robot_namespace="/", reference_frame="world",
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a URDF file.

    :param model_path: The path to the URDF file.
    :type model_path: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: [True if the command was sent and False otherwise, Status message].
    :rtype: [bool, str]
    """
    
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    if os.path.exists(model_path) is False:
        print("Error: model path does not exist")
        return False

    model = xacro.process_file(model_path)
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success, result.status_message
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_urdf_pkg(  pkg_name, file_name, file_folder="/urdf", model_name="robot1", robot_namespace="/", reference_frame="world",
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a URDF file.

    :param pkg_name: Name of the package to import the URDF file from.
    :type pkg_name: str

    :param file_name: Name of the URDF file to import.
    :type file_name: str

    :param file_folder: Folder where the URDF file is located. Defaults to "/urdf".
    :type file_folder: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: [True if the command was sent and False otherwise, Status message].
    :rtype: [bool, str]
    """
    
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return False

    file_path = pkg_path + file_folder + "/" + file_name

    if os.path.exists(file_path) is False:
        print("Error: model path does not exist")
        return False

    model = xacro.process_file(file_path)
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success, result.status_message
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_urdf_string(   model_string, model_name="robot1", robot_namespace="/", reference_frame="world",
                                pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                                ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a URDF file.

    :param model_string: URDF string to import.
    :type model_string: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used.
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: [True if the command was sent and False otherwise, Status message].
    :rtype: [bool, str]
    """
    
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success, result.status_message
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_urdf_param(    param_name, model_name="robot1", robot_namespace="/", reference_frame="world",
                                pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                                ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a URDF in the param server.

    :param param_name: Name of model to spawn
    :type param_name: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used.
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: [True if the command was sent and False otherwise, Status message].
    :rtype: [bool, str]
    """

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    final_param_name = robot_namespace + "/" + param_name

    if rospy.has_param(final_param_name) is False:
        print("Error: Parameter " + final_param_name +" does not exist")
        return False

    model_string = rospy.get_param(final_param_name)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success , result.status_message
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_sdf_path( model_path, model_name="robot1", robot_namespace="/", reference_frame="world",
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a SDF file.

    :param model_path: The path to the SDF file.
    :type model_path: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """
    
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    if os.path.exists(model_path) is False:
        print("Error: model path does not exist")
        return False

    model = xacro.process_file(model_path)
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_sdf_pkg(  pkg_name, file_name, file_folder="/sdf", model_name="robot1", robot_namespace="/", reference_frame="world",
                            pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                            ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a SDF file.

    :param pkg_name: Name of the package to import the SDF file from.
    :type pkg_name: str

    :param file_name: Name of the SDF file to import.
    :type file_name: str

    :param file_folder: Folder where the SDF file is located. Default is "/sdf".
    :type file_folder: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """
    
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    rospack = rospkg.RosPack()
    try:
        pkg_path = rospack.get_path(pkg_name)
        rospy.logdebug("Package FOUND...")
    except rospkg.common.ResourceNotFound:
        rospy.logerr("Package NOT FOUND")
        return False

    file_path = pkg_path + file_folder + "/" + file_name

    if os.path.exists(file_path) is False:
        print("Error: model path does not exist")
        return False

    model = xacro.process_file(file_path)
    encoding = {}
    model_string = model.toprettyxml(indent='  ', **encoding)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_sdf_string(   model_string, model_name="robot1", robot_namespace="/", reference_frame="world",
                                pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                                ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a SDF file.

    :param model_string: SDF string to import.
    :type model_string: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used.
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """
    
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_spawn_sdf_param(    param_name, model_name="robot1", robot_namespace="/", reference_frame="world",
                                pos_x=0.0, pos_y=0.0, pos_z=0.0, 
                                ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=1.0) -> bool:
    """
    Function to spawn a model from a SDF in the param server.

    :param param_name: Name of model to spawn
    :type param_name: str

    :param model_name: Name of model to spawn
    :type model_name: str

    :param robot_namespace: change ROS namespace of gazebo-plugins.
    :type robot_namespace: str

    :param reference_frame: Name of the model/body where initial pose is defined. If left empty or specified as "world", gazebo world frame is used.
    :type reference_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    client_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

    final_param_name = robot_namespace + "/" + param_name

    if rospy.has_param(final_param_name) is False:
        print("Error: Parameter " + final_param_name +" does not exist")
        return False

    model_string = rospy.get_param(final_param_name)

    srv_model = SpawnModelRequest(  model_name=model_name,
                                    model_xml=model_string,
                                    robot_namespace=robot_namespace,
                                    initial_pose=Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z),
                                                        orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w)),
                                    reference_frame=reference_frame)
    
    try:
        result = client_srv(srv_model)
        return result.success
    except ServiceException:
        print("Error: Package used in model was not found, source the workspace in all terminals.")
        return False

def gazebo_get_model_state(model_name, relative_entity_name="world"):
    """
    Function to get the state of a model.

    :param model_name: Name of model to get the state of.
    :type model_name: str

    :param relative_entity_name: Return pose and twist relative to this entity (an entity can be a model, body, or geom).
    :type relative_entity_name: str

    :return: The header of the message, the pose of the model, the twist of the model, and success.
    """

    rospy.wait_for_service("/gazebo/get_model_state")
    client_srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    srv_msg = GetModelStateRequest(model_name=model_name, relative_entity_name=relative_entity_name)

    try:
        result = client_srv(srv_msg)
        return result.header, result.pose, result.twist, result.success
    except ServiceException:
        print("Error processing the request")
        return Header(), Pose(), Twist(), False

def gazebo_set_model_state(model_name, ref_frame="world", pos_x=0.0, pos_y=0.0, pos_z=0.0, ori_x=0.0, ori_y=0.0, ori_z=0.0, ori_w=0.0,
                            lin_vel_x=0.0, lin_vel_y=0.0, lin_vel_z=0.0, ang_vel_x=0.0, ang_vel_y=0.0, ang_vel_z=0.0, sleep_time=0.05) -> bool:
    """
    Function to set the model name in gazebo.

    :param model_name: Name of model to set.
    :type model_name: str

    :param ref_frame: Reference frame of model.
    :type ref_frame: str

    :param pos_x: x position of model in model's reference frame
    :type pos_x: float
    :param pos_y: y position of model in model's reference frame
    :type pos_y: float
    :param pos_z: z position of model in model's reference frame
    :type pos_z: float

    :param ori_x: X part of Quaternion of model orientation in model's reference frame.
    :type ori_x: float
    :param ori_y: Y part of Quaternion of model orientation in model's reference frame.
    :type ori_y: float
    :param ori_z: Z part of Quaternion of model orientation in model's reference frame.
    :type ori_z: float
    :param ori_w: W part of Quaternion of model orientation in model's reference frame.
    :type ori_w: float

    :param lin_vel_x: X part of linear velocity of model in model's reference frame.
    :type lin_vel_x: float
    :param lin_vel_y: Y part of linear velocity of model in model's reference frame.
    :type lin_vel_y: float
    :param lin_vel_z: Z part of linear velocity of model in model's reference frame.
    :type lin_vel_z: float

    :param ang_vel_x: X part of angular velocity of model in model's reference frame.
    :type ang_vel_x: float
    :param ang_vel_y: Y part of angular velocity of model in model's reference frame.
    :type ang_vel_y: float
    :param ang_vel_z: Z part of angular velocity of model in model's reference frame.
    :type ang_vel_z: float

    :param sleep_time: Time to sleep bewteen sending request and getting response.
    :type sleep_time: float

    :return: True if the command was sent and False otherwise.
    :rtype: bool
    """

    rospy.wait_for_service("/gazebo/set_model_state")
    client_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    msg_pose = Pose(  position=Point(x=pos_x, y=pos_y, z=pos_z), orientation=Quaternion(x=ori_x, y=ori_y, z=ori_z, w=ori_w))
    msg_twist = Twist(  linear=Vector3(x=lin_vel_x, y=lin_vel_y, z=lin_vel_z), angular=Vector3(x=ang_vel_x, y=ang_vel_y, z=ang_vel_z))

    srv_msg = ModelState(model_name=model_name, pose=msg_pose, twist=msg_twist, reference_frame=ref_frame)

    srv_request = SetModelStateRequest(model_state=srv_msg)

    try:
        result = client_srv(srv_request)
        time.sleep(sleep_time)
        return result.success
    except ServiceException:
        print("Error processing the request")
        return False
