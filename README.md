### Project Explanation

The following changes have been applied to the codebase:

#### dbw_node.py
The drive-by-wire node is responsible for using data from the /twist_cmd, /current_velocity and /vehicle/dbw_enabled topics to control the brake, steering and throttle. Those are controlled by sending proper messages to the specific topics.
The node subscribes to the required topics, and the callbacks just store the information passed in the callbacks message attribute.
A twist controller is used in the loop to fetch the proper values for throttle, brake and steer and if not in manual mode (dbw_enabled) those values are published to the proper topics.
The twist controller is initialized with default values provided by configuration values.

#### twist_controller.py
The implementation of the twist controller uses the following provided implementations:
* YawController (yaw_controller property): controller used to find the steering value
* PID (throttle_pid property): PID controller used to find the proper throttle value

The generated values:
* throttle: using the PID controller with the current velocity error and the time delta (difference between the previous time and the current time)
* brake: if the PID controller generates a negative value, this is used as the break value
* steer: the value returned by the yaw controller

If the throttle value is positive, the brake value is set to 0. If the throttle value is negative, throttle is set to 0 and brake is set to the negative throttle value.

There's a slight modifications to the brake value: some properties of the car (mass, fuel capacity and wheel radius) are used to generate a more realistic break value.

#### waypoint_updater.py
The trigger is the pose_cb callback function. This method is called every time a new car position is consumed from the /current_pose topic. Whenever a message is received, the send_final_waypoints method is called.
The logic of this method is:
* receive the waypoint closest to the current car position using the get_next_waypoint_index method
* generate a new array of waypoints in front of the car. The first waypoint is the one of the current cars location. All next waypoints, up to LOOKAHEAD_WPS are included in this array
* set all the velocities of the nodes to the max values (The PID controller handles acceleration and brake; so no problem with max values)
* if there is a red light detected in the next LOOKAHEAD_WPS number of waypoints, linearly decrease the velocity of the nodes starting at the current one, till the one representing the light index, so it ends at 0.
* create a Lane object, representing the waypoints, and publish them to the /final_waypoints topic

To find the closest waypoint to a specific location (current cars position, light position, ...) the get_next_waypoint_index method is implemented. This will iterate over all waypoints, and finds the one with the closest distance in front of the car.

#### tl_classifier.py
The input of the classifier is an image from the camera, and it should detect whether or not a traffic light is visible, and if it's showing a red light. There are multiple possible ways of implementing this: machine learning, opencv, ...

On this project, opencv is used to check if red lights are available. Logic explained in https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/ is used to check if there are any red lights in the image.

#### tl_detector.py
The most important update in this class is the get_light_state: this method will iterate over the stop line positions in the stop_line_positions array, and find the closest one. If it's in the front of the car, the get_light_state method is used to find the state of the light (red or not). To find the state, the previous classifier is used.

### Improvements:
* closest distance to the car can be optimised much more

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

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

### Usage

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
