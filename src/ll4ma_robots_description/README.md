# LL4MA Robots Description    
This package contains the description files and models for robots and end-effectors used in our lab, including URDFs and mesh files (STLs, DAEs, etc.).

## Installation
Clone this repository into the `src` folder of a cakin workspace and build as usual

    git clone git@bitbucket.org:robot-learning/ll4ma_robots_description.git     # SSH
    git clone https://bitbucket.org/robot-learning/ll4ma_robots_description.git # HTTP
    
## Structure
We use [xacro](http://wiki.ros.org/xacro) to build the final URDFs since, among other nice features, it offers the ability to incrementally build up URDFs to include only the desired components. Each manipulator receives a single `<MANIPULATOR>.robot.xacro` file in the `robots` folder that serves as the main interface to adding features and components to the manipulator. Xacro arguments appear at the top of each file and indicate the options available for the manipulator. We currently have the following configurations:

### Manipulators
 * KUKA LBR4+
 * Baxter
### End-Effectors
 * Allegro
 * ReFlex
 * Push-stick
### Sensors
 * Optoforce 6-axis force/torque sensor