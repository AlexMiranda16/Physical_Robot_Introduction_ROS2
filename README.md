# Physical_Robot_Introduction_ROS2

This repository contains three ROS2 demos to teach the user how to work with ROS2 to develop programs for physical robots. This tutorial will explain how to setup and run each demo for the robot depicted below. The objective of this guide is to give a first contact to the robot by running and testing these demos, while at the same time get more used to ROS2 environment. This programs were specifically made for this robot so, it is assumed that you are using one equal to the one shown on the image below.

<img src="https://user-images.githubusercontent.com/60965257/190620499-43fed76a-25cf-47f2-94da-b5b48753c434.jpg" width=50% height=50%>


The first demo tests the motors, controlling the robot using a keyboard, and checking if the motor current and encoders are published.
The second demo tests if the camera and the image processing is properly working, by reading the distances measured at the front and back of the robot.
The third and last demo is a simple reactive robot demo, where the robot uses the camera to check for object at his front and avoids them.

These demos are independent from each other, and if you want to run one demo when another is running, you should stop the running one first before starting the demo wanted.

This repository was developed during Alexandre Miranda's master dissertation, at the Faculty of Engineering of the University of Porto (FEUP).

# 1. Prerequisites

One obvious prerequisite is to have at least one robot equal to the one used to develop these demos.

Regarding software, you need to install some tools, mainly:
  - Raspbian OS (Debian Buster version)
  - ROS2 Foxy Fitzroy (and some packages described below)
  - Arduino IDE
  - Visual Studio Code (optional)
  - VNC Server (optional but recomended for remote access to the robot's Operating System, both for developing, running and debbuging programs)
 
You will also need to configure device identification ports attributed by the OS.

For you to know how to install all the software and how to create fixed tty device ports, you can download the following PDF file. This guide provides additional information to possible installation and configuration problems.

[Software_Installation_Env_Config.pdf](https://github.com/AlexMiranda16/Physical_Robot_Introduction_ROS2/files/9583154/Software_Installation_Env_Config.pdf)

Besides the software, you need to create a new ROS2 workspace to place all the robot packages. In this example, the ROS2 workspace is "dev_ws" but you can give it another name.

Open a terminal and create a new directory. Then, go to the "src" directory and clone the repository. Check the dependencies of the workspace and then build it using the colcon command.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/AlexMiranda16/Physical_Robot_Introduction_ROS2
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

As the instalation and configuration guide refers, you need to add additional packages to your workspace.

Download the following packages to the src folder of your workspace directory. When downloaded, compile the workspace again.
```
cd ~/dev_ws/src
git clone --branch foxy https://github.com/ros-perception/vision_opencv.git
git clone --branch foxy https://github.com/ros-perception/image_common.git
git clone --branch ros2 https://github.com/ros-perception/image_transport_plugins.git
cd ..
rosdep install --from-paths src -r -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
The next step is to install the RaspiCam, an API for using the Raspberry Pi camera with OpenCV. The compilation process guide is explained on the ReadMe file on the RaspiCam repository, located here: <https://github.com/cedricve/raspicam>.

Do not forget that in every new terminal opened, you need to source the setup script of this workspace. Alternatively, you can add the source command to the .bashrc file in a similar way to that shown in section A.3 of the intallation guide.

```
source install/setup.bash
```

For more information about the ROS2 workspace creation, please visit "[Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)".


# 2. Demo 1 - Keyboard robot motor control

**Objective**: Test the motors' operation.

The program controls the robot movement using the "wasd" keyboard keys:
  - w - foward;
  - a - left rotation;
  - s - backward;
  - d - right rotation;
  - To stop the motors, any other key can be pressed.
  
If not done yet, use the Arduino IDE to upload the code to the Arduino board you are using (must be an ATMega328 microcontroller board).

Assuming the projects are already compiled (if not, use the colcon build command previously mentioned in your workspace directory), open two terminal windows and go to your workspace diretory and source the script file in both of them.

```
cd ~/dev_ws/
source install/setup.bash
```

Now, in one terminal start the motor control program with the following command. It is in this terminal that you will insert the keyboard commands to move the robot.

```
ros2 run motor_test_control demo
```

On the other terminal, start the "serial_com" node from the "rpi_uno" package. This package is responsable for the communication between the Raspberry Pi and the Arduino. This program subscribes to the topic that sends the movement command for each motor, and send it to the Arduino.
This node also receives information from the Arduino, namely the motor current and the odometry, and publish it.

```
ros2 run rpi_uno serial_com
```
The robot now should be ready to go!

To check if everything is running properly, with both nodes running, run the ``ros2 node list`` and ``ros2 topic list`` commands on one new terminal. You should see the following nodes and topics running.

![Demo1_node_topics](https://user-images.githubusercontent.com/60965257/190594444-a0310837-16d3-4e9c-bc04-63d100a22258.JPG)

If you want to check if the "serial_com" node is publishing the current and odometry from both motors, you can use the ``ros2 topic echo`` command. 
In two new separate terminals, run:

```
ros2 topic echo /motor_current
ros2 topic echo /motor_odometry
```

The following video shows an example on how the application should work. You can see on the top left the two terminals. The left one prints the encoder values received by the arduino, and the right one prints the current. You can also see how the robot should behave by pressing different letters. If you haven't done yet, give it a try!


https://user-images.githubusercontent.com/60965257/190596889-5334ae5b-3bf6-450e-a1cf-424cafd11dd7.mp4


**Additional Notes:**

All messages have a letter that works as an identification and a number that corresponds to the value we want to send or read. In this case, each letter represents a motor and the type of data we are sending/receiving, and the number translates its value. All messages have the same order: ``left_motor&value right_motor&value`` (first is the letter and value of the left motor, followed by a space and then the letter and value of the right motor).
This next table clarifies the different letters meaning.

| Channel  | Meaning |
| :---: | ------------- |
| L  | Command left motor speed  |
| R  | Command right motor speed  |
| i  | Current of the left motor  |
| j  | Current of the right motor  |
| l  | Encoder value of the left motor  |
| r  | Encoder value of the right motor  |


# 3. Demo 2 - Camera based virtual LiDAR test

**Objective**: test the camera and the applied image processing algorithm.

This demo prints on the terminal the distance of the nearest object both at the front and at the rear of the robot.

If not done yet, use the Arduino IDE to upload the code to the Arduino board you are using (must be an ATMega328 microcontroller board).

Assuming the projects are already compiled (if not, use the colcon build command previously mentioned in your workspace directory), open two terminal windows and go to your workspace diretory and source the script file in both of them.

```
cd ~/dev_ws/
source install/setup.bash
```

Now, in one terminal start the camera range test program with following command. This terminal will display the frontal and rear measurements.

```
ros2 run camera_lidar_range_test demo
```

On the other terminal, run the camera simulated LiDAR. This node is the one responsible for gathering the camera image, do some processing on it and with that it simulates a LiDAR sensor, and publish the distances calculated around the robot.

```
ros2 run fisheye_camera_simulated_sensors_ros2 simulated_lidar
```

Instead of running both ``ros2 run`` to execute this application, it can be used the launch file created. Stop both nodes if they are still running and in one terminal run the next ``ros2 launch`` command:

```
ros2 launch camera_lidar_range_test range_demo.launch.xml
```

Again, to check if everything is running properly, with both nodes running, run the ``ros2 node list`` and ``ros2 topic list`` commands on one new terminal. You should see the following nodes and topics running.

![Demo2_node_topics](https://user-images.githubusercontent.com/60965257/190601008-9b5fea76-4365-43f6-ae48-f2cd3bfdaec1.JPG)

If everything is working as pretended, the demo should behave similarly to the video shown. On the top left of the video you can see the distance measured both by the front and back of the robot. Note that the values aren't precise and therefore do not represented the actual distance. This must be seen more as a scale, where the bigger the number, the further the objects are.


https://user-images.githubusercontent.com/60965257/190601560-2dfdb772-2660-4139-8ebb-a952cbab0ec2.mp4

**Additional Notes**

Tall objects might give wrong measures. In the video you can see that the hand down and up at a similar distance give different measurements.


# 4. Demo 3 - Reactive robot demo

**Objective**: Run a simple reactive robot that avoids colision with objects.

The robot moves forward until a nearby object is identified in front of it. The robot then rotates to the left until the object is no longer in its front and proceeds to move forward in the new direction.

If not done yet, use the Arduino IDE to upload the code to the Arduino board you are using (must be an ATMega328 microcontroller board).

Assuming the projects are already compiled (if not, use the colcon build command previously mentioned in your workspace directory), open one terminal window and go to your workspace diretory and source the script file.

```
cd ~/dev_ws/
source install/setup.bash
```

Now, start the reactive robot program with the following ``ros2 launch`` command. This launch file will start three nodes:

- The reactive robot demo node. Evaluates the camera information and commands the movement for the robot to make.
- The simulated LiDAR node. As previsouly explained on the demo2 program, is responsible for the camera image processing, simulating a virtual LiDAR by calculating the distance of objects aroud the robot.
- The Raspberry Pi - Arduino communication node. Also explained previouly. Ensures the communication between the Rasperry and the Arduino, by sending movement commands for each motor, and receiving the current and odometry of each one.

```
ros2 launch reactive_robot_demo reactive.launch.xml
```

In a couple of seconds, the robot should start moving by itself.

Again, to check if everything is running properly, with both nodes running, run the ``ros2 node list`` and ``ros2 topic list`` commands on one new terminal. You should see the following nodes and topics running.

![Demo3_node_topics](https://user-images.githubusercontent.com/60965257/190604640-a879503b-804c-40cf-a9ca-e0a9be6e9811.JPG)

If the robot is doing what it is supposed to do in this demo, you should see him behave similarly to this video. He will turn left to avoid obstacles when he is near them.

https://user-images.githubusercontent.com/60965257/190605027-d67acebc-124f-40ca-b0a2-27d1a983f239.mp4





