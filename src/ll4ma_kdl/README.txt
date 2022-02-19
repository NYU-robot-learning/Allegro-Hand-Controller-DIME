	   __________________________________________________

	       KINEMATIC AND DYNAMIC SOLVERS FOR ROBOTIC
			      MANIPULATORS

		     LL4MA Lab, University of Utah
	   __________________________________________________

Table of Contents
_________________

1 Contributors:
2 Avoid the following:
3 How do I use it?
4 Current implementation:
.. 4.1 KDL:
..... 4.1.1 Tree:
..... 4.1.2 Kinematics:
..... 4.1.3 Dynamics:
5 Dev notes:
.. 5.1 TODO Cleanup list:
6 Future work:
.. 6.1 Integrations:


1 Contributors:
===============

  + Balakumar Sundaralingam
  + Hunter Brown
  + Adam Conkey


2 Avoid the following:
======================

  + Do not use vector<double> to store joint configurations as KDL
    requires Eigen::VectorXd as the type and this will lead to
    typecasting. The old code was rewritten to avoid this exact issue.


3 How do I use it?
==================

  Look at this file `src/kdl_test_node.cpp' to see an example. Use this
  code in your own repo by linking to this package.

  Two types of KDL objects can be created:
  + manipulator: single base, single end-effector.
  + robot: multiple base, multiple end-effector


4 Current implementation:
=========================

4.1 KDL:
~~~~~~~~

  The standard kinematics and dynamics library (KDL) which is also a
  part of OROCOS allowing for realtime controllers.


4.1.1 Tree:
-----------

  The main component of KDL is building a kdl tree. This can be done in
  two ways- loading a urdf file or reading the robot description from
  the ros parameter server. We load it from a urdf file.


4.1.2 Kinematics:
-----------------

  The forward kinematics and Jacobian can be obtained using the
  following functions:


4.1.3 Dynamics:
---------------

  KDL has functions to compute the matrices which are part of the
  inverse dynamics controller, namely- inertia(H), coriollis(V) and
  gravity(G). These matrices can be used to compute joint torques
  required to compensate for the robot's dynamics.


5 Dev notes:
============

5.1 TODO Cleanup list:
~~~~~~~~~~~~~~~~~~~~~~

  + Template for eigen/std vectors, need to come up with a custom
    struct?.
  + shared ptr instead of pointer in initialization.
  + benchmarking


6 Future work:
==============

6.1 Integrations:
~~~~~~~~~~~~~~~~~

  - DART library
  - TRAC IK
