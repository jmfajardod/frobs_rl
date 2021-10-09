.. _install:

Installation
============

**FRobs_RL** have been only tested in Ubuntu 20.04, althought it may work in other OS its performance have not been tested in other OS.


Prerequisites
-------------

ROS
~~~

As **FRobs_RL** uses `ROS <https://www.ros.org>`_ as the middleware to interact between the learning algorithm and the robotic hardware the user must have it installed. **FRobs_RL** have been tested using the *Noetic* distribution, the official documentation to install ROS Noetic can be found her `Noetic Installation <http://wiki.ros.org/noetic/Installation>`_.

An abridged version of the commands to install *ROS* including the *Gazebo* simulator can be seen below.

.. code-block:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo apt install python3-rosdep
    sudo rosdep init
    rosdep update

Catkin Tools
~~~~~~~~~~~~

We recommend that the user install `Catkin Tools <https://catkin-tools.readthedocs.io/en/latest/installing.html>`_ to ease the setup of ROS workspaces and compilation of packages. To install it execute the commands:

.. code-block:: bash

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install python3-catkin-tools

XTerm
~~~~~

**FRobs_RL** uses the *XTerm* terminal to launch different processes like ROS Nodes or the Gazebo simulator. To install *XTerm* execute:

.. code-block:: bash

    sudo apt install xterm


Workspace creation and library compilation
------------------------------------------

To use the library it is necessary to download and compile the library package using Catkin. To create a new ROS workspace called *rl_ws* and compile the **frobs_rl** package, use the following commands:

.. code-block:: bash

    cd ~
    mkdir -p rl_ws/src
    cd rl_ws
    git clone https://github.com/jmfajardod/frobs_rl src/frobs_rl -b main
    rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
    catkin config --extend /opt/ros/${ROS_DISTRO}
    catkin build
    source devel/setup.bash

After downloading the package and compiling it using Catkin it is necessary to download the *Python* dependencies. To download them using **pip** execute the following commands:

.. note::

    By default **pip** will install a **PyTorch** version without GPU support, if your computer has a supported GPU follow the instructions on the official `PyTorch website <https://pytorch.org>`_

.. code-block:: bash
    
    roscd frobs_rl # Only works the setup.bash has been sourced
    python3 -m pip install -r requirements.txt

