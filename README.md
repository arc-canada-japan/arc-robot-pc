# ARC Project - Robot PC side

/!\ README FILE WORK IN PROGRESS

This framework is a part of the ARC project to create a platform for teleoperating robots on a wide scale. Even if the main goal of the ARC project is for caregiving, this framework is made adaptable to many situations.

It is divided into several packages, and they are all in Python (but this is not a limitation, if a C++ package is required `_cpp` will be suffixed to its name). The packages have been built in order to be easily adaptable, e.g. for new devices, with few modifications (ideally just adding files). This repository is for the Robot PC, i.e. the PC connected to the robot, receiving the orders from the Operator PC. The code for this PC is in another repository: [ARC Operator PC Unity](https://github.com/arc-canada-japan/arc-operator-pc-unity) (not public yet).

The current repository is composed of the following packages:

* `arc_gui`: it have been moved to a separate package available here: [ARC GUI](https://github.com/arc-canada-japan/arc-gui)
* `communication_interface`: to have a project that is not dependent on ROS only, the user can select how the messages are sent between the PCs.
* `robot_control`: this is the main package. To run the project, you need to use the launch file of this package, that is calling the others. It sends the command to the robot according to the command received.
* `video_publisher`: this package is used to stream the video from the robot PC to the operator PC. The method of streaming can be selected.

You can have more information of each package in their respective README files (TODO).

## How to install ##

Once this project is cloned, you will also need to install the ROS package [ROS TCP endpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) from the branch `main-ros2`.

```git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git --branch main-ros2```

After the download, you have to build it with `colcon build` and to source it. Either manually each time, of by putting it in the `.bashrc` file.

## How to build ##

Since this project is in Python, you do not it to build it *per se*. However, you need to use the `concol build` command to copy the code, and parametring ROS to be able to use it.
When it's done, you can source the project.

## How to run ##

### With command line ###

You can use the following command to use this project:

```
ros2 launch robot_control controller.launch.py robot_name:=XXX streaming_method:=YYY communication_interface:=ZZZ
```

You should replace `XXX`, `YYY` and `ZZZ` by the name of your robot, streaming method and communication interface, respectively. To know the available options, you can look in the `config` folders of the `robot_control`, `video_publisher` and `communication_interface` respectively. The name is the same as the config file, without the extension (case sensitive).
You can also add the argument `simulation_only:=True` to test only the communication and the controller compution. The real robot will not move, but the joint values will still be exchanged.

To add your own robot, streaming method or communication interface, please read the README file of each package.


### With GUI ###

See [ARC GUI](https://github.com/arc-canada-japan/arc-gui)

## Server Repository ##

See [ARC Server](https://github.com/arc-canada-japan/arc-zmq-server).

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

