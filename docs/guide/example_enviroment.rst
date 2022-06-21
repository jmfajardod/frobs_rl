.. _example_enviroment:

Example enviroment
==================

In this page we show how to create a robotic manipulator enviroment with a reacher task and how to then train it.

Creating a RobotEnv
~~~~~~~~~~~~~~~~~~~~~

The first thing that we need to do is to create a class that inherits from the `RobotBasicEnv` class. To create this file we can copy the `CustomRobotEnv` found in the `templates` folder of the library. Within this class we will be defining the general specifications of the robot like is URDF model, its controllers, the positions where it will be spawned in the Gazebo simulator, among other things. Althought the file contains many functions we only need to focus on the `__init__` function and the `_check_subs_and_pubs_connection` function. The `__init__` function is the one that will be called when we create an instance of the class. The `_check_subs_and_pubs_connection` function can be used to check if the subscribers and publishers are connected to the correct topics.


.. code-block:: python

    class CustomRobotEnv(robot_BasicEnv.RobotBasicEnv):
        def __init__(self):
            # Define class variables and functions
            # Initialize parent class
            super(CustomRobotEnv, self).__init__(launch_gazebo, gazebo_init_paused, gazebo_use_gui)
        def _check_subs_and_pubs_connection(self):
            # Checks ROS and Gazebo connections
            raise NotImplementedError()


In our example for the reacher task an ABB IRB120 is used as the robot. The URDF model, the controllers and the spawn position are defined in the `__init__` function. As we are using MoveIt some additional functions are defined in the `ABBIRB120MoveItEnv` class.

.. code-block:: python

    class ABBIRB120MoveItEnv(robot_BasicEnv.RobotBasicEnv):
        """
        Superclass for all ABB IRB120 environments.
        """

        def __init__(self):
            """
            Initializes a new ABBIRB120Env environment.

            Sensor Topic List:
            * /joint_states : JointState received for the joints of the robot

            Actuators Topic List:
            * MoveIt! : MoveIt! action server is used to send the joint positions to the robot.
            """
            rospy.loginfo("Starting ABBIRB120MoveIt Env")
            ros_gazebo.gazebo_unpause_physics()

            """
            Robot model and controllers parameters
            """
            self.model_name_in_gazebo="robot1"
            self.namespace="/robot1"
            pkg_name="abb_irb120"
            urdf_file="irb120.urdf.xacro" 
            urdf_xacro_args=['use_pos_ctrls:=true']
            model_pos_x=0.0; model_pos_y=0.0; model_pos_z=0.0 
            controller_file="irb120_pos_controller.yaml"
            self.controller_list=["joint_state_controller","arm120_controller"]

            """
            Use parameters to allow multiple robots in the environment.
            """
            if rospy.has_param('/ABB_IRB120_Reacher/current_robot_num'):
                num_robot = int(rospy.get_param("/ABB_IRB120_Reacher/current_robot_num") + 1)
                self.model_name_in_gazebo = "robot" + str(num_robot)
                self.namespace = "/robot" + str(num_robot)
                
            else:
                num_robot = 0
                self.model_name_in_gazebo = "robot" + str(num_robot)
                self.namespace = "/"
            
            rospy.set_param('/ABB_IRB120_Reacher/current_robot_num', num_robot)
            model_pos_x = int(num_robot / 4) * 2.0
            model_pos_y = int(num_robot % 4) * 2.0
            model_pos_z = 0.0

            """
            Set if the controllers in "controller_list" will be reset at the beginning of each episode, default is False.
            """
            reset_controllers=False

            """
            Set the reset mode of gazebo at the beginning of each episode: 1 is "reset_world", 2 is "reset_simulation". Default is 1.
            """
            reset_mode=1
            
            """
            Set the step mode of Gazebo. 1 is "using ros services", 2 is "using step function of gazebo". Default is 1.
            If using the step mode 2 then set the number of steps of Gazebo to take in each episode. Default is 1.
            """
            step_mode=1

            """
            Init the parent class with the corresponding variables.
            """
            super(ABBIRB120MoveItEnv, self).__init__(   launch_gazebo=False, spawn_robot=True, 
                        model_name_in_gazebo=self.model_name_in_gazebo, namespace=self.namespace, pkg_name=pkg_name, 
                        urdf_file=urdf_file, controller_file=controller_file, controller_list=self.controller_list, urdf_xacro_args=urdf_xacro_args,
                        model_pos_x=model_pos_x, model_pos_y=model_pos_y, model_pos_z=model_pos_z, 
                        reset_controllers=reset_controllers, reset_mode=reset_mode, step_mode=step_mode)

            """
            Define publisher or subscribers as needed.
            """
            if self.namespace is not None and self.namespace != '/':
                self.joint_state_topic = self.namespace + "/joint_states"
            else:
                self.joint_state_topic = "/joint_states"

            self.joint_names = [ "joint_1",
                                "joint_2",
                                "joint_3",
                                "joint_4",
                                "joint_5",
                                "joint_6"]

            self.joint_state_sub = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)
            self.joint_state = JointState()

            """
            Init MoveIt
            """
            ros_launch.ros_launch_from_pkg("abb_irb120_reacher","moveit_init.launch", args=["namespace:="+str(self.namespace)])
            rospy.wait_for_service("/move_group/trajectory_execution/set_parameters")
            print(rostopic.get_topic_type("/planning_scene", blocking=True))
            print(rostopic.get_topic_type("/move_group/status", blocking=True))

            """
            If using the _check_subs_and_pubs_connection method, then un-comment the lines below.
            """
            self._check_subs_and_pubs_connection()

            #--- Start MoveIt Object
            self.move_abb_object = MoveABB()

            """
            Finished __init__ method
            """
            rospy.loginfo("Finished Init of ABBIRB120MoveIt Env")
            ros_gazebo.gazebo_pause_physics()

To check if the topics and services are working, we use the `_check_subs_and_pubs_connection` method.

.. code-block:: python

    def _check_subs_and_pubs_connection(self):
        """
        Function to check if the gazebo and ros connections are ready
        """
        self._check_joint_states_ready()
        return True
    
    
    def _check_joint_states_ready(self):
        """
        Function to check if the joint states are received
        """
        ros_gazebo.gazebo_unpause_physics()
        print( rostopic.get_topic_type(self.joint_state_topic, blocking=True))
        rospy.logdebug("Current "+ self.joint_state_topic +" READY=>" + str(self.joint_state))
            
        return True


Creating a TaskEnv
~~~~~~~~~~~~~~~~~~

After creating the `RobotEnv` class which contains the general functions for the robot, we create the `TaskEnv` class which contains the specific functions for the task. As a task must always have a robot the `TaskEnv` class inherits from the `RobotEnv` class. Within this class we define the specific functions for the task, like the observation, the reward, the reset, the action space, the observation space, the goal and the termination. To create a file for the task we can copy the template file `CustomTaskEnv` from the `templates` folder.

The template has different functions for the observation, the reward, the reset, the action space, the observation space, the goal and the termination. It will also register the enviroment to the OpenAI Gym environment list so that it can be used as a part of the OpenAI Gym library.


.. code-block:: python

    register(
        id='CustomTaskEnv-v0',
        entry_point='frobs_rl.templates.CustomTaskEnv:CustomTaskEnv',
        max_episode_steps=10000,
    )

    class CustomTaskEnv(CustomRobotEnv.CustomRobotEnv):
        """
        Custom Task Env, use this env to implement a task using the robot defined in the CustomRobotEnv
        """

        def __init__(self):
            """
            Describe the task.
            """
            rospy.loginfo("Starting Task Env")

            """
            Init super class.
            """
            super(CustomTaskEnv, self).__init__()

            """
            Define the action and observation space.
            """
            # self.action_space = spaces.Discrete(n_actions)
            # self.action_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

            # self.observation_space = spaces.Discrete(n_observations)
            # self.observation_space = spaces.Box(low=0, high=1, shape=(1,), dtype=np.float32)

            """
            Define subscribers or publishers as needed.
            """
            # self.pub1 = rospy.Publisher('/robot/controller_manager/command', JointState, queue_size=1)
            # self.sub1 = rospy.Subscriber('/robot/joint_states', JointState, self.callback1)

            """
            Finished __init__ method
            """
            rospy.loginfo("Finished Init of Custom Task env")

        #-------------------------------------------------------#
        #   Custom available methods for the CustomTaskEnv      #

        def _set_episode_init_params(self):
            """
            Function to set some parameters, like the position of the robot, at the beginning of each episode.
            """
            raise NotImplementedError()

        def _send_action(self, action):
            """
            Function to send an action to the robot
            """
            raise NotImplementedError()
            

        def _get_observation(self):
            """
            Function to get the observation from the environment.
            """
            raise NotImplementedError()

        def _get_reward(self):
            """
            Function to get the reward from the environment.
            """
            raise NotImplementedError()
        
        def _check_if_done(self):
            """
            Function to check if the episode is done.
            
            If the episode has a success condition then set done as:
                self.info['is_success'] = 1.0
            """
            raise NotImplementedError()


For our example task, we begin by defining the observation and action space in the `__init__` method, along with initializing the `ABBIRB120MoveItEnv`, which is our `RobotEnv`.

.. code-block:: python

    class ABBIRB120ReacherEnv(abb_irb120_moveit.ABBIRB120MoveItEnv):

        def __init__(self):
            """
            Reacher enviroment for the ABB IRB120 robot.
            """
            rospy.logwarn("Starting ABBIRB120ReacherEnv Task Env")

            """
            Load YAML param file
            """
            ros_params.ros_load_yaml_from_pkg("abb_irb120_reacher", "reacher_task.yaml", ns="/")
            self.get_params()

            """
            Define the action and observation space.
            """
            #--- Define the ACTION SPACE
            # Define a continuous space using BOX and defining its limits
            self.action_space = spaces.Box(low=np.array(self.min_joint_values), high=np.array(self.max_joint_values), dtype=np.float32)

            #--- Define the OBSERVATION SPACE
            observations_high_goal_pos_range = np.array(np.array([self.position_goal_max["x"], self.position_goal_max["y"], self.position_goal_max["z"]]))
            observations_low_goal_pos_range  = np.array(np.array([self.position_goal_min["x"], self.position_goal_min["y"], self.position_goal_min["z"]]))

            observations_high_vec_EE_GOAL = np.array([1.0, 1.0, 1.0])
            observations_low_vec_EE_GOAL  = np.array([-1.0, -1.0, -1.0])

            #- With Vector from EE to goal, Goal pos and joint angles
            high = np.concatenate([observations_high_vec_EE_GOAL, observations_high_goal_pos_range, self.max_joint_values, ])
            low  = np.concatenate([observations_low_vec_EE_GOAL,  observations_low_goal_pos_range,  self.min_joint_values, ])

            #--- Observation space
            self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32) 

            #-- Action space for sampling
            self.goal_space = spaces.Box(low=observations_low_goal_pos_range, high=observations_high_goal_pos_range, dtype=np.float32)

            """
            Define subscribers or publishers as needed.
            """

            self.goal_subs  = rospy.Subscriber("goal_pos", Point, self.goal_callback)
            if self.training:
                ros_node.ros_node_from_pkg("abb_irb120_reacher", "pos_publisher.py", name="pos_publisher", ns="/")
                rospy.wait_for_service("set_init_point")
                self.set_init_goal_client = rospy.ServiceProxy("set_init_point", SetLinkState)

            """
            Init super class.
            """
            super(ABBIRB120ReacherEnv, self).__init__()

            """
            Finished __init__ method
            """
            rospy.logwarn("Finished Init of ABBIRB120ReacherEnv Task Env")


After defining the observation and action space we define the methods to execute an action and get the observation.

.. code-block:: python

    def _send_action(self, action):
        """
        The action are the joint positions
        """
        rospy.logwarn("=== Action: {}".format(action))

        #--- Make actions as deltas
        action = self.joint_values + action
        action = np.clip(action, self.min_joint_values, self.max_joint_values)

        self.movement_result = self.set_trajectory_joints(action)
        if not self.movement_result:
            rospy.logwarn("Movement_result failed with the action of : " + str(action))
        

    def _get_observation(self):
        """
        It returns the position of the EndEffector as observation.
        And the distance from the desired point
        Orientation for the moment is not considered
        """

        #--- Get Current Joint values
        self.joint_values = self.get_joint_angles()

        #--- Get current goal
        current_goal = self.goal

        #--- Get EE position
        ee_pos_v = self.get_ee_pose() # Get a geometry_msgs/PoseStamped msg
        self.ee_pos = np.array([ee_pos_v.pose.position.x, ee_pos_v.pose.position.y, ee_pos_v.pose.position.z])

        #--- Vector to goal
        vec_EE_GOAL = current_goal - self.ee_pos
        vec_EE_GOAL = vec_EE_GOAL / np.linalg.norm(vec_EE_GOAL)

        obs = np.concatenate((
            vec_EE_GOAL,             # Vector from EE to Goal
            current_goal,            # Position of Goal
            self.joint_values        # Current joint angles
            ),
            axis=None
        )

        rospy.logwarn("OBSERVATIONS====>>>>>>>"+str(obs))
        return obs.copy()

Then we proceed by defining the reward method and the method for episode termination.

.. code-block:: python

    def _get_reward(self):
        """
        Given a success of the execution of the action
        Calculate the reward: binary => 1 for success, 0 for failure
        """

        #--- Get current EE pos 
        current_pos = self.ee_pos # If using ARRAY

        #- Init reward
        reward = 0

        #- Check if the EE reached the goal
        done = False
        done = self.calculate_if_done(self.movement_result, self.goal, current_pos)
        if done:
            if self.pos_dynamic is False:
                rospy.logwarn("SUCCESS Reached a Desired Position!")
                self.info['is_success'] = 1.0

            #- Success reward
            reward += self.reached_goal_reward
        else:
            #- Distance from EE to Goal reward
            dist2goal = scipy.spatial.distance.euclidean(current_pos, self.goal)
            reward += - self.mult_dist_reward*dist2goal 

            #- Constant reward
            reward += self.step_reward

        self.pub_marker.publish(self.goal_marker)

        #- Check if joints are in limits
        joint_angles = np.array(self.joint_values)
        min_joint_values = np.array(self.min_joint_values)
        max_joint_values = np.array(self.max_joint_values)
        in_limits = np.any(joint_angles<=(min_joint_values+0.0001)) or np.any(joint_angles>=(max_joint_values-0.0001))
        reward += in_limits*self.joint_limits_reward

        rospy.logwarn(">>>REWARD>>>"+str(reward))

        return reward
    
    def _check_if_done(self):
        """
        Check if the EE is close enough to the goal
        """

        #--- Get current EE based on the observation
        current_pos = self.ee_pos # If using ARRAY

        #--- Function used to calculate 
        done = self.calculate_if_done(self.movement_result, self.goal, current_pos)
        if done:
            rospy.logdebug("Reached a Desired Position!")

        #--- If the position is dynamic the episode is never done
        if self.pos_dynamic is True:
            done = False

        return done

.. note:: 
    It is important to say that we can add as many methods to the class as we require.


Finally, we add the register function at the beginning of the file to register the enviroment within the OpenAI Gym library.

.. code-block:: python

    register(
        id='ABBIRB120ReacherEnv-v0',
        entry_point='abb_irb120_reacher.task_env.irb120_reacher:ABBIRB120ReacherEnv',
        max_episode_steps=10000
    )

Setting the training parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After we have defined the Task enviroment we need to create a YAML file with the training parameters. Based on the algorithm that we choose we can copy the template file from the `config` folder of `FRobs_RL` and modify it.

In this example we will use the `TD3` algorithm and we will set the number of training steps to 100000. We will also set a saving frequency of the model, so the trained model is being constantly saved and not only at the end of the training. The saving frequency will be set to 5000 steps. We will set the log folder to `TD3_New`, in this folder the tensorboard logs will be saved.Finally, we can set parameters to use a custom policy architecture and the `TD3` specific parameters. After all the changes are made to the template the file will look as follows:

.. code-block:: yaml

    model_params:

        # Training 
        training_steps: 100000      # The number of training steps to perform

        # Save params
        save_freq: 5000
        save_prefix: td3_model
        trained_model_name: trained_model
        save_replay_buffer: False

        # Load model params - Used to load a trained model and continue training
        load_model: False
        model_name: trained_model_01_09_2021_10_11_11

        # Logging parameters
        log_folder: TD3_New
        log_interval: 1 # The number of episodes between logs
        reset_num_timesteps: False # If true, will reset the number of timesteps to 0 every training 

        # Use custom policy - Only used when new model is created
        use_custom_policy: False
        policy_params:
            net_arch: [400, 300] # List of hidden layer sizes
            activation_fn: relu  # relu, tanh, elu or selu
            features_extractor_class: FlattenExtractor # FlattenExtractor, BaseFeaturesExtractor or CombinedExtractor
            optimizer_class: Adam # Adam, Adadelta, Adagrad, RMSprop or SGD

        # UseAction noise
        use_action_noise: True # For now only Gaussian noise is supported
        action_noise:
            mean: 0.0
            sigma: 0.01

        # TD3 parameters
        td3_params:
            learning_rate: 0.001
            buffer_size: 1000000
            learning_starts: 100
            batch_size: 100
            tau: 0.005
            gamma: 0.99
            gradient_steps: -1
            policy_delay: 2
            target_policy_noise: 0.2
            target_noise_clip: 0.5
            train_freq:
            freq: 20
            unit: step  # episode or step

Training the model
~~~~~~~~~~~~~~~~~~~~

Finally, we will need to create a simple script to train the model which calls the `train` function of the `ABBIRB120ReacherEnv` class and uses the `TD3` algorithm with the parameters defined in the YAML file. 

In the following script, the Gazebo simulator is launched, then the environment is created and some wrappers are applied to the environment. The environment is then trained using the `TD3` algorithm with the parameters defined in the YAML file. And finally, the trained model is saved.

.. code-block:: python

    if __name__ == '__main__':

    # Kill all processes related to previous runs
    ros_node.ros_kill_all_processes()

    # Launch Gazebo 
    ros_gazebo.launch_Gazebo(paused=True, gui=False)

    # Start node
    rospy.logwarn("Start")
    rospy.init_node('train_irb120_reacher')

    # Launch the task environment
    env = gym.make('ABBIRB120ReacherEnv-v0')

    #--- Normalize action space
    env = NormalizeActionWrapper(env)

    #--- Normalize observation space
    env = NormalizeObservWrapper(env)

    #--- Set max steps
    env = TimeLimitWrapper(env, max_steps=100)
    env.reset()

    #--- Set the save and log path
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("abb_irb120_reacher")
    
    #-- TD3
    save_path = pkg_path + "/models/static_reacher/td3/" # Path where the model will be saved
    log_path = pkg_path + "/logs/aux/td3/"               # Path where the logs will be saved
    model = TD3(env, save_path, log_path, config_file_pkg="abb_irb120_reacher", config_filename="td3_aux.yaml")
    
    #-- Train the model and save it
    model.train()
    model.save_model()
    model.close_env()

    sys.exit()