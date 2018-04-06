# Project: Programming a Real Self-Driving Car

## Team Members

| | Name | email
--- | ---- | -----
Team Lead | Vaibhav Kachore | vaibhav.kachore@gmail.com 
Team Member 1 | Ayham Zaza | lear.ayham@zoho.com
Team Member 2 | Hiroyuki Hasebe | crystalline407@gmail.com
Team Member 3 | Jan Tomecek | jan.tomecek.jt@gmail.com
Team Member 4 | Pepar Hugo | peparhugo@gmail.com

## Introduction

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Components

### Waypoint Updater

*TODO*

### Control

*TODO*

### Perception

*TODO*

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
- | - | --------------
1172.183 | 1186.299 | 318
1584.065 | 1156.953 | 784
2126.353 | 1550.636 | 2095
2178.291 | 1819.328 | 2625
1469.499 | 2946.97 | 6322
797.9147 | 2905.59 | 7036
160.8088 | 2279.929 | 8565
363.378 | 1553.731 | 9773
