Welcome to FROBS_RL documentation!
===================================

**FROBS_RL** (Flexible Robotics Reinforcement Learning Library)  is a flexible robotics reinforcement learning (RL) library. It is primarly designed to be used in robotics applications using the ROS framework. It is written in **Python** and uses libraries based on the *PyTorch* framework to handle the machine learning. The library uses *OpenAI Gym* to create and handle the RL environments, *stable-baselines3* to provide state-of-the-art RL algorithms, *Gazebo* to simulate the physical environments, and *XTerm* to display and launch many of the **ROS** nodes and processes in a lightweight terminal.

FRobs_RL is stored in a Github repository: https://github.com/jmfajardod/frobs_rl

Main goals
--------------

- Provide a framework to easily train and deploy RL algorithms in robotics applications using the ROS middleware.
- Provide a framework to easily create RL enviroments for any type of task.
- Provide a framework to easily use, test or create state-of-the-art RL algorithms in robotics applications.

.. note::

   This project is under active development.

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   self
   guide/installation
   guide/env_creation
   guide/env_template
   guide/train_model
   guide/rl_models
   guide/use_trained_model
   guide/example_enviroment

.. toctree::
   :maxdepth: 2
   :caption: API
   
   api/index
