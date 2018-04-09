# Project: Programming a Real Self-Driving Car

## Team Members

| | Name | email
--- | ---- | -----
Team Lead | Vaibhav Kachore | vaibhav.kachore@gmail.com 
Team Member 1 | Ayham Zaza | learn.ayham@zoho.com
Team Member 2 | Hiroyuki Hasebe | crystalline407@gmail.com
Team Member 3 | Jan Tomecek | jan.tomecek.jt@gmail.com
Team Member 4 | Pepar Hugo | peparhugo@gmail.com

[//]: # (Image References)

[image1]: ./imgs/waypoint_updater.png "waypoint_updater"

## Introduction

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Components

### Waypoint Updater


**Subscriber**
- /current_pose
- /base_waypoints
- /traffic_waypoints

**Publisher**
- /final_waypoints

This module publishes a subset of /base_waypoints to /final_waypoints. The first waypoint in the list is the first waypoint that is currently ahead of the car. Also, we need to adjust the target velocities for the waypoints leading up to red traffic lights in order to bring the vehicle to a smooth and full stop.

The vehicle decelerates the speed according to the following formula:

```python
# dist: the distance between the waypoint and the stop line.
vel = math.sqrt(self.max_decel * dist)
if vel < 1.:
  vel = 0.
# wp.twist.twist.linear.x: the target x direction linear velocity.
p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
```

The following is a graph with the x axis as the distance to the stop line and the y axis as the speed.

![alt text][image1]

### Control

**Subscriber**
- /current_velocity
- /twist_cmd
- /vehicle/dbw_enabled

**Publisher**
- /vehicle/steering_cmd
- /vehicle/throttle_cmd
- /vehicle/brake_cmd

This module publishes three commands throttle, brake and steering.

Throttle and brake values are calculated by the following steps:

1. To reduce noise, apply a low pass filter to the current velocity.
2. Calculate the difference between the current velocity and the target velocity, and calculate the throttle value with the PID controller.
3. If the target velocity and the current velocity are almost 0, set the throttle to zero and apply the meximum braking.
4. If the target velocity is lower than the current velocity, calculate the brake value by taking care of the vehicle mass, the wheel radius and the velocity error.

Steering value is calculated by the following steps:

1. To reduce noise, apply a low pass filter to the current velocity.
2. Pass the linear velocity, the angular velocity and the current  velocity to the yaw controller and calculate the throttle value.
3. Apply low pass filter to the output steering value.

For low pass filters and PID filter, We chose each parameter as follows.

**PID filter**

Kp | Ki | Kd | min | max
-- | -- | -- | --- | ---
0.3 | 0.1 | 0. | 0. | 0.2

**Low pass filter (current velocity)**

tau | ts
--- | --
0.5 | 0.02

**Low pass filter (steering)**

tau | ts
--- | --
1 | 1


### Perception

The main task of the Perception module is detection and classification of obstacles in front of the vehicle.
In case of this project, a front camera is used to analyze surrounding area of vehicle only and obstacles are traffic lights.

The Perception module processes several input signals:

- /current\_pose - position of the vehicle
- /base\_waypoints - list of waypoints
- /image\_color - image from the front camera
- traffic\_light\_config.yaml - list of stop line positions before each traffic lights

Workflow of this module is following:


1. Vehicle’s waypoint - Based on vehicle's position on the track [/current\_pose], find the closest waypoint [/base_waypoints] with use of Euclidian distance.
2. Traffic light’s waypoint - Based on stop lines position in the map (located at sim\_traffic\_light\_config.yaml), find the closest waypoint with use of Euclidian distance.
3. Detect closest Traffic light’s waypoint in front of the vehicle.
4. Measure distance between vehicle’s waypoint and the closest traffic light’s waypoint.
5. If the distance is smaller than some threshold, take an image from vehicles’s camera [/image\_color] and continue with next steps. If there is no close traffic light start from step 1 again.
6. Run Traffic Light Detector and classifier for the camera’s image [from tl\_classfier.py]
7. If red light was detected, send information about closest red-light traffic light waypoint to Waypoint Updater Node [/traffic\_waypoint]

#### Traffic lights detection and classification
Detection and classification of traffic lights on a captured image from front camera is done by Deep learning technique. Two algorithms and architectures were tested:

- Single shot detector v2 (SSD v2) algorithm based on Inception architecture
- Faster RCNN algorithm based on Resnet101 architecture

For both of these options, pre-trained models from Tensorflow detection model zoo were used. These models were pretrained on COCO dataset which already contains traffic lights.
To improve accuracy of detection and classification the models were further retrained on labeled images from simulator and rosbag videos. These images were collected and labeled by other Udacity students from previous classes.
The used dataset is located [here](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view). 

TensorFlow Object detection API was used for the retraining step. For both mentioned algorithms and images from simulator 10000 global steps were used. Using AWS p2x.large server, 
the re-training phase took ~3hours. To make the models suited for Udacity usecases, number of classification classes was changed to 4 (Red, Yellow, Green, Unknown), maximal detections per class to 3 and maximal detections to 4 for Faster-RCNN model.
For real-world usecase, the pretrained models for simulator images were further trained on real-world images from rosbag for another 10000 steps. 
The configuration of TensorFlow object detection API was inspired by ColdKnight's blog and repository [here](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI).
Even if Faster-RCNN inference speed is ~3x slower than SSDv2 model, Faster-RCNN model was used because of much better accuracy. 

Examples of detection and recognition:

![image alt text](imgs/green_sim.png)

![image alt text](imgs/red_sim.png)

![image alt text](imgs/green_real.png)

![image alt text](imgs/green_real2.png)

![image alt text](imgs/red_real.png)

## Preparation

You have to download the following model files

- [for simulator](https://drive.google.com/open?id=1Rv-Uyjw80t1cl1l-nFBn0_dcT3NU34On)
- [for realworld](https://drive.google.com/file/d/10HT1FlAp9GRw_PFzrL4GvW0jJ2JDshBR/view?usp=sharing)

Put these files in the directory ros/src/tl_detector/light_classification/classifiers/.

If you downloaded the release file, you don't have to do that. (already contains these files)

## Installation

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

## Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

### How to run rviz from docker container

1. Run the command `xhost +local:docker` on the host.

2. Run the docker container.
```bash
nvidia-docker run -p 4567:4567 -v /tmp/.X11-unix/:/tmp/.X11-unix -v /home/paperspace/CarND-Capstone:/capstone -v /tmp/log:/root/.ros/log --rm -it <image name>
```

3. Download the rviz config file from [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/default.rviz).

4. Copy default.rviz file to ~/.rviz/ in the docker container.

5. Run the command `roscore` (for replay) or `roslaunch launch/site.launch` (for testing traffic lights).

6. In the another terminal, run the command `docker ps`, and confirm the container id.

7. Run the following command:
```bash
docker exec -it -e DISPLAY=$DISPLAY <container id> bash
```

8. Run `rviz`

9. In the third terminal, run the following command:
```bash
docker exec -it <container id> bash
```

10. Run
```bash
rosbag play -l <bag file>
```

11. If you would like to view color image, enable the Image checkbox in the Display panel, set the Image Topic to "/image_color" and click the Image tab.

## Debugging

### How to output a log

- You can use rospy.logdebug, rospy.loginfo, rospy.logwarn, rospy.logerr, rospy.logfatal.
- These logs will be written to files in ~/.ros/log.
- Logs that are equal or higher than warn are also output to the console.
- For real time log monitoring, you can also execute `rostopic echo /rosout`. (if loglevel >= loginfo)

### How to read messages which are transmitted on a topic

- ou can use `rostopic echo` command.
- For example, if you execute the following command, messages sent to /twist_cmd are output to the console.
```bash
$ rostopic echo /twist_cmd
```

## Tips

### When using a docker image, an error occurred with "IndexError: Tuple index out of range".

You have to update the pillow version.

```bash
$ pip install pillow==4.3.0
```

For more details, see https://discussions.udacity.com/t/issue-simulator-talking-to-docker/411204.

### How many traffic lights exist in the simulator?

The total number is 8.

 x | y | waypoint index
--- | --- | --------------
1172.183 | 1186.299 | 318
1584.065 | 1156.953 | 784
2126.353 | 1550.636 | 2095
2178.291 | 1819.328 | 2625
1469.499 | 2946.97 | 6322
797.9147 | 2905.59 | 7036
160.8088 | 2279.929 | 8565
363.378 | 1553.731 | 9773
