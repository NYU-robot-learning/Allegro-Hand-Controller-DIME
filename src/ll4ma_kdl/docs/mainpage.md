<div id="table-of-contents">
<h2>Table of Contents</h2>
<div id="text-table-of-contents">
<ul>
<li><a href="#sec-1">1. Contributors:</a></li>
<li><a href="#sec-2">2. Avoid the following:</a></li>
<li><a href="#sec-3">3. How do I use it?</a></li>
<li><a href="#sec-4">4. Current implementation:</a>
<ul>
<li><a href="#sec-4-1">4.1. KDL:</a>
<ul>
<li><a href="#sec-4-1-1">4.1.1. Tree:</a></li>
<li><a href="#sec-4-1-2">4.1.2. Kinematics:</a></li>
<li><a href="#sec-4-1-3">4.1.3. Dynamics:</a></li>
</ul>
</li>
</ul>
</li>
<li><a href="#sec-5">5. Dev notes:</a>
<ul>
<li><a href="#sec-5-1">5.1. <span class="todo TODO">TODO</span> Cleanup list:</a></li>
</ul>
</li>
<li><a href="#sec-6">6. Future work:</a>
<ul>
<li><a href="#sec-6-1">6.1. Integrations:</a></li>
</ul>
</li>
</ul>
</div>
</div>


# Contributors:<a id="sec-1" name="sec-1"></a>

-   Balakumar Sundaralingam
-   Hunter Brown
-   Adam Conkey

# Avoid the following:<a id="sec-2" name="sec-2"></a>

-   Do not use vector<double> to store joint configurations as KDL requires Eigen::VectorXd as the type and this will lead to typecasting. The old code was rewritten to avoid this exact issue.

# How do I use it?<a id="sec-3" name="sec-3"></a>

Look at this file `src/kdl_test_node.cpp` to see an example. Use this code in your own repo by linking to this package.

Two types of KDL objects can be created:
-   manipulator: single base, single end-effector.
-   robot: multiple base, multiple end-effector

# Current implementation:<a id="sec-4" name="sec-4"></a>

## KDL:<a id="sec-4-1" name="sec-4-1"></a>

The standard kinematics and dynamics library (KDL) which is also a part of OROCOS allowing for realtime controllers.

### Tree:<a id="sec-4-1-1" name="sec-4-1-1"></a>

The main component of KDL is building a kdl tree. This can be done in two ways- loading a urdf file or reading the robot description from the ros parameter server. We load it from a urdf file.

### Kinematics:<a id="sec-4-1-2" name="sec-4-1-2"></a>

The forward kinematics and Jacobian can be obtained using the following functions:

### Dynamics:<a id="sec-4-1-3" name="sec-4-1-3"></a>

KDL has functions to compute the matrices which are part of the inverse dynamics controller, namely- inertia(H), coriollis(V) and gravity(G). These matrices can be used to compute joint torques required to compensate for the robot's dynamics. 

# Dev notes:<a id="sec-5" name="sec-5"></a>

## TODO Cleanup list:<a id="sec-5-1" name="sec-5-1"></a>

-   Template for eigen/std vectors, need to come up with a custom struct?.
-   shared ptr instead of pointer in initialization.
-   benchmarking

# Future work:<a id="sec-6" name="sec-6"></a>

## Integrations:<a id="sec-6-1" name="sec-6-1"></a>

-   DART library
-   TRAC IK
