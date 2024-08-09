# ARC Project - Robot PC side

This framework is a part of the ARC project to create a platform for teleoperating robots on a wide scale. Even if the main goal of the ARC project is for caregiving, this framework is made adaptable to many situations.

It is divided into several packages, and they are all in Python (but this is not a limitation, if a C++ package is required `_cpp` will be suffixed to its name). The packages have been built in order to be easily adaptable, e.g. for new devices, with few modifications (ideally just adding files). This repository is for the Robot PC, i.e. the PC connected to the robot, receiving the orders from the Operator PC. The code for this PC is in another repository: (ARC Operator PC Unity)[https://github.com/Yoshida-Lab-TUS/arc-operator-pc-unity].

*TODO Explain the project, with a graph of PCs.*

The current repository is composed of the following packages:

* `arc_gui`: this is the GUI used to make the project more usable. It allows the user to change the parameters easily, and then it will call the launch file of the `robot_control` package. TODO
* `communication_interface`: to have a project that is not dependent on ROS only, the user can select how the messages are sent between the PCs.
* `robot_control`: this is the main package. To run the project, you need to use the launch file of this package, that is calling the others. It sends the command to the robot according to the command received.
* `video_publisher`: this package is used to stream the video from the robot PC to the operator PC. The method of streaming can be selected.

## GUI ##

More details are in the package README file.

## Communication ##

## Robot Control ##

## Video ##

TODO
