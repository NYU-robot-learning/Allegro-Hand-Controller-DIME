# allegro_hand_ros

Allegro Hand ROS Noetic
================================

This is a modified release to control Allegro Hand with ROS Noetic.
Mostly, it is based on the original release of Allegro Hand ros package (by simlab robotics).

You can find original release of the [hand ros package][1].
[1]: https://github.com/simlabrobotics/allegro_hand_ros_v4

It provides a python package that can control the hand directly.

It also provides the BHand library directly in this package (including both
32-bit and 64-bit versions, though 32-bit systems will need to update the
symlink manually).

At this point no effort has been made to be backwards compatible. Some of the
non-compatible changes between the two version are:

 - Put all of the controllers into one *package* (allegro_hand_controllers) and
   made each controller a different node (allegro_node_XXXX): grasp and pd.
 - Single launch file with arguments instead of multiple launch files with
   repeated code.
 - Both the parameter and description files are now ROS packages, so that
   `rospack find` works with them.
 - Added logger, position and torque controllers.
 - Modified the PD hand controller with gravity compensation with a better loop rate that sets the joint state to
   the desired joint state.

Installing the PCAN driver
--------------------------

Before using the hand, you must install the pcan drivers. This assumes you have
a peak-systems pcan to usb adapter.

1. Install these packages

    sudo apt-get install libpopt-dev ros-kinetic-libpcan

2. Download driver: https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.12.0.tar.gz

Install the drivers:

    tar -xvzf peak-linux-driver-8.12.0.tar.gz
    cd peak-linux-driver-8.12.0.tar.gz
    make clean
    make NET=NO_NETDEV_SUPPORT
    sudo make install
    sudo /sbin/modprobe pcan

Test that the interface is installed properly with:

     cat /proc/pcan

You should see some stuff streaming.

When the hand is connected, you should see pcanusb0 or pcanusb1 in the list of
available interfaces:

    ls -l /dev/pcan*

If you do not see any available files, you may need to run:

    sudo ./driver/pcan_make_devices 2

from the downloaded pcan folder: this theoretically creates the devices files if
the system has not done it automatically.

3. Download PCAN API: https://www.peak-system.com/quick/BasicLinux

Install the API:

    tar -xvzf pcan_basic_linux-4.1.1
    cd PCAN_Basic_Linux-4.1.1/pcanbasic
    make
    sudo make install


Setting up the ROS Noetic workspace
------------------------------------

1. Installing ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu

2. Creating a catkin workspace: 
```
    source /opt/ros/noetic/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
```
Make sure your ROS_PACKAGE_PATH is set properly.
```
    source opt/ros/noetic/setup.bash
    echo $ROS_PACKAGE_PATH /home/youruser/catkin_ws/src:/opt/ros/noetic/share
```


Setting up the Allegro Hand Controller
------------------------------------

1. Clone the repository
```
    cd ~/catkin_ws/src/
    git clone https://github.com/NYU-robot-learning/Allegro-hand-controller-noetic
    cd Allegro-hand-controller-noetic/src/
    git clone https://bitbucket.org/robot-learning/ll4ma_kdl
    git clone https://bitbucket.org/robot-learning/ll4ma_robots_description

```
2. Build the sources
```
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
```
3. Quick start
```
    roslaunch allegro_hand allegro_hand.launch 
```

4. Using the python package to run poses
```
    cd ~/catkin_ws/src/Allegro-hand-controller-noetic/
    python src/allegro_hand/scripts/allegro_poser.py
```

Launch file instructions:
------------------------

There is now a single file,
[allegro_hand.launch](allegro_hand/launch/allegro_hand.launch)
that starts the hand. It takes many arguments, but at a minimum you must specify
the handedness:

    roslaunch allegro_hand allegro_hand.launch

Optional (recommended) arguments:

          NUM:=0|1|...
          ZEROS:=/path/to/zeros_file.yaml
          HAND:=right|left (default is right)
          CONTROLLER:=pd|grasp (default is pd)
          RESPAWN:=true|false   Respawn controller if it dies.
          KEYBOARD:=true|false  (default is true)
          AUTO_CAN:=true|false  (default is true)
          CAN_DEVICE:=/dev/pcanusb1 | /dev/pcanusbNNN  (ls -l /dev/pcan* to see open CAN devices)
          VISUALIZE:=true|false  (Launch rviz)
          JSP_GUI:=true|false  (show the joint_state_publisher for *desired* joint angles)

Note on `AUTO_CAN`: There is a nice script `detect_pcan.py` which automatically
finds an open `/dev/pcanusb` file. If instead you specify the can device
manually (`CAN_DEVICE:=/dev/pcanusbN`), make sure you *also* specify
`AUTO_CAN:=false`. Obviously, automatic detection cannot work with two hands.

The second launch file is for visualization, it is included in
`allegro_hand.launch` if `VISUALIZE:=true`. Otherwise, it can be useful to run
it separately (with `VISUALIZE:=false`), for example if you want to start rviz separately
(and keep it running):

    roslaunch allegro_hand allegro_viz.launch

Note that you should also specify the hand `NUM` parameter in the viz launch if
the hand number is not zero.

Packages
--------

 * **allegro_hand** A python client that enables direct control of the hand in
                    python code.
 * **allegro_hand_driver** Driver for talking with the allegro hand.
 * **allegro_hand_controllers** Different nodes that actually control the hand.
 The AllegroNode class handles all the generic driver comms, each class then
 implements `computeDesiredTorque` differently (and can have various topic
 subscribers):
   * grasp: Apply various pre-defined grasps, including gravity compensation.
   * pd: Joint space control: save and hold positions.
 * **allegro_hand_description** xacro descriptions for the kinematics of the
     hand, rviz configuration and meshes.
 * **allegro_hand_keyboard** Node that sends the commanded grasps. All commands
     are available with the grasp controller, only some are available with the
     other controllers.
 * **allegro_hand_parameters** All necessary parameters for loading the hand:
   * gains_pd.yaml: Controller gains for PD controller.
   * initial_position.yaml: Home position for the hand.
   * zero.yaml: Offset and servo directions for each of the 16 joints, and some
           meta information about the hand.
   * zero_files/ Zero files for all hands.
 * **bhand** Library files for the predefined grasps, available in 32 and 64 bit
     versions. 64 bit by default, update symlink for 32 bit.
 * **ll4ma_kdl** Kinematics Dynamics Library made by LL4MA lab for generating the 
     gravity compensation torque and PD torque. (Can be separately downloaded from
     https://bitbucket.org/robot-learning/ll4ma_kdl)
 * **ll4ma_robots_description** urdfs and xacro descriptions for the kinematics of 
     the hand (primarily to support the kdl package). (Can be separately downloaded 
     from https://bitbucket.org/robot-learning/ll4ma_robots_description)


Note on polling (from Wonik Robotics): The preferred sampling method is utilizing the
Hand's own real time clock running @ 333Hz by polling the CAN communication
(polling = true, default). In fact, ROS's interrupt/sleep combination might
cause instability in CAN communication resulting unstable hand motions. We have used a 
loop rate of 300Hz.


Useful Links
------------

 * [Allegro Hand wiki](http://wiki.wonikrobotics/AllegroHand/wiki).
 * [ROS wiki for original package with Kinetic](http://www.ros.org/wiki/allegro_hand_ros_v4).


Controlling More Than One Hand
------------------------------

When running more than one hand using ROS, you must specify the number of the
hand when launching.

    roslaunch allegro_hand.launch HAND:=right ZEROS:=parameters/zero0.yaml NUM:=0 CAN_DEVICE:=/dev/pcan0 AUTO_CAN:=false

    roslaunch allegro_hand.launch HAND:=left  ZEROS:=parameters/zero1.yaml NUM:=1 CAN_DEVICE:=/dev/pcan1 AUTO_CAN:=false


Known Issues:
-------------

While all parameters defining the hand's motor/encoder directions and offsets
fall under the enumerated "allegroHand_#" namespaces, the parameter
"robot_description" defining the kinematic structure and joint limits remains
global. When launching a second hand, this parameter is overwritten. I have yet
to find a way to have a separate enumerated "robot_decription" parameter for
each hand. If you have any info on this, please advise.
