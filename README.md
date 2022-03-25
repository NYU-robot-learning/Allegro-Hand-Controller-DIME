# Allegro Hand Controller - DIME
This repository contains the information to setup the ROS Noetic based Allegro Hand controller package which is a part of the official implementation of [DIME](https://arxiv.org/abs/2203.13251). We would advice you to run the `setup.sh` file in the base repository for convenience and use this as a debugging tool.

## Contents
1. [Requirements](#requirements)
2. [Installing Drivers](#driver-installation)
3. [Setting up DART Sim](#dart-sim)
4. [Launching the Controller](#launch-controller)

## Installation Requirements <a name="requirements"></a>
1. LibPCAN package
2. Allegro Hand v4
   
## Installing Drivers <a name="driver-installation"></a>
Before setting up the controller, we need to ensure that we have all the drivers necessary for establishing communication over the PCAN channel. Make sure to install the following dependencies first before setting up the allegro controller package.

1. Install these C++ dependencies:
   ```
   sudo apt-get install cmake gcc g++ libpopt-dev
   ```
2. Set up the necessary drivers using the `allegro_driver_setup.sh` script in #include-base-url-here. 
3. Test the installation of the drivers using the following command:
   ```
   cat /proc/pcan
   ```
   You will be able to see the pcan driver stream data if you are connected to a PCAN device.
4. You can also run the following command to list all the PCAN devices connected to the system:
   ```
   ls -l /dev/pcan*
   ```
5. If you do not see any available files, you may need to run:
   ```
   sudo ./driver/pcan_make_devices 2
   ```

## Setting up DART Sim <a name="dart-sim"></a>
We need to setup DART Sim for the ll4ma-kdl packages to run. Run the following commands:
```
sudo apt-add-repository ppa:dartsim/ppa
sudo apt-get update
sudo apt-get install libdart6-all-dev
```

## Launching the Controller <a name="launch-controller"></a>
After setting up the drivers and DART sim, you can run catkin_make from the base controller directory (where you have both - Kinova JACO arm controller and Allegro Hand controller). Do the following from this directory:
```
cd <base-controller-dir>
catkin_make
```
After you make the binaries, you need to source the setup files using the below command:
```
source <base-controller-dir>/devel/setup.bash
```
Then you can launch the Allegro Hand's roslaunch file to start controlling the roslaunch file.
```
roslaunch allegro_hand allegro_hand.launch
```
If you want to launch the visualizer along with the roslaunch file, you can run the following command:
```
roslaunch allegro_hand allegro_hand.launch VISUALIZE:=true
```

## Citation

If you use this repo in your research, please consider citing the paper as follows:
```
@article{arunachalam2022dime,
  title={Dexterous Imitation Made Easy: A Learning-Based Framework for Efficient Dexterous Manipulation},
  author={Sridhar Pandian Arunachalam and Sneha Silwal and Ben Evans and Lerrel Pinto},
  journal={arXiv preprint arXiv:2203.13251},
  year={2022}
}
```
