# Gesture interpretation

## Description
This is the ROS package _tracker_ for the recognition and interpretation of pointing gestures, following the methods presented in the paper:

_A. C. A. Campos and B. V. Adorno, "Development of Human-Robot Communication Technologies for Future Interaction Experiments," 2020 Latin American Robotics Symposium (LARS), 2020 Brazilian Symposium on Robotics (SBR) and 2020 Workshop on Robotics in Education (WRE), Natal, Brazil, 2020, pp. 1-6, doi: 10.1109/LARS/SBR/WRE51543.2020.9306965_. ([link](https://ieeexplore.ieee.org/document/9306965))

The application tracks the human joints poses, obtained with a RGD-D camera (Kinect), to define the pointing gestures and interpret them using the poses of the objects in the environment. More information about the nodes and classes of the application can be found [here](https://gitlab.com/adorno-at-macro/experimental-robotics/human-robot-communication/gesture-recognition/-/wikis/General-information). 

The whole application runs the following nodes:

### openni_tracker
A ROS [package](http://wiki.ros.org/openni_tracker) to get the information from Kinect about human joints. It publishes a tf topic with the joints poses. Information about the installation and usage of the openni_tracker package can be found [here](https://gitlab.com/adorno-at-macro/experimental-robotics/human-robot-communication/gesture-recognition/-/wikis/openni_tracker:-Installation-and-usage).

### tracker_tf_listener
A [node](/nodes/tracker_tf_listener.py) to read the human joints poses obtained with the openni_tracker and publish them in PoseStamped format. The node is needed because we need to use Python2 to access the tf topic, while the other scripts run with Python3.

### gesture_interpretation

The [node](nodes/gesture_interpretation.py) is responsible for the recognition and interpretation of human pointing gestures, using the human joints poses and the poses of objects or regions of interest in the environment.

## How to run
With the package already in your ROS catkin workspace, you can run the whole application using a launch file. First, you need to set the parameters in [launch_all.launch](launch/launch_all.launch) (the file contains instructions) and then, on a terminal, run:


```
roslaunch tracker launch_all.launch
```

For the experiments, there is a specific [script](nodes/main.py) to control the application. To run, you should update the [launch_tracker.launch](launch/launch_tracker.launch) file and launch using [launch_main.launch](launch/launch_main.launch), as shown below:

```
roslaunch tracker launch_main.launch
```
If using the script for the experiments, you can enable or disable the application publishing in the /tracker_commands topic.
