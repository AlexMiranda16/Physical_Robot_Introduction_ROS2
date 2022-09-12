# Physical_Robot_Introduction_ROS2

This repository contains three ROS2 demos with different functionalities for the robot developed during the dissertation. Here will be explained how to setup and run each one of them. This programs were specifically made for this robot so, it is assumed that you are using one equal to the one presented.

## (METER FOTO DO ROBO)

The first demo tests the motors, controlling the robot using a keyboard, and checking if the motor current and encoders are published.
The second demo tests if the camera and the image processing is properly working, by reading the distances measured at the front and back of the robot.
The third and last demo is a simple reactive robot demo, where the robot uses the camera to check for object at his front and avoids them.

These demos are independent from each other, and if you want to run one demo when another is running, you should stop the running one first before starting the demo wanted.


# 1. Prerequisites

One obvious prerequisite is to at least have one robot equal to the one used to develop these demos.



Besides the software, you need to create a new ROS2 workspace to place all the robot packages. In this example, the ROS2 workspace was named "dev_ws" but you can give it another name.

Open a terminal and create a new directory. Then, go to the "src" directory and clone the repository. Check the dependencies of the workspace and then build it using the colcon command.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/AlexMiranda16/Physical_Robot_Introduction_ROS2
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Don't forget that in every new terminal opened, you need to source the setup script of this workspace. Alternatively, you can add the source command to the .bashrc file in a similar way to that shown in the intallation guide.

```
source install/setup.bash
```

For more information about the ROS2 workspace creation visit "[Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)".




# 2. Demo 1 - Keyboard robot motor control

This first demo has the purpose to test the motors operation.

The program controls the robot movement using the "wasd" keyboard keys
  (w - foward;
  a - left rotation;
  s - backward;
  d - right rotation;).
To stop the motors, any other key can be pressed.
  
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

The demo now should be working. Try to press the keys to check if the robot is moving.

If you want to check if the "serial_com" node is publishing the current and odometry from both motors, you can use the ``ros2 topic echo`` command. 
In two new separate terminals, run:

```
ros2 topic echo /motor_current
ros2 topic echo /motor_odometry
```

**Note:**
All messages have a letter that works as an identification and a number that corresponds to the value we want to send or read. In this case, each letter represents a motor and the type of data we are sending/receiving, and the number translates its value. All messages have the same order: ``left_motor_value right_motor_value`` (first is the letter and value of the left motor, followed by a space and then the letter and value of the right motor).



# 3. Demo 2 - Camera based virtual LiDAR test

The second demo has the objective to test the camera and the applied image processing.

This demo prints on the terminal the distance of the nearest object both at the front and at the rear of the robot.

Assuming the projects are already compiled (if not, use the colcon build command previously mentioned in your workspace directory), open two terminal windows and go to your workspace diretory and source the script file in both of them.

```
cd ~/dev_ws/
source install/setup.bash
```

Now, in one terminal start the camera range test program with following command. This terminal will display the frontal and rear measurements.

```
ros2 run motor_test_control demo
```

On the other terminal, run the camera simulated LiDAR. This node is the one responsible for gathering the camera image, do some processing on it and with that it simulates a LiDAR sensor, and publish the distances calculated around the robot.

```
ros2 run fisheye_camera_simulated_sensors_ros2 simulated_lidar
```

Instead of running both ``ros2 run`` to execute this application, it can be used the launch file created. Stop both nodes if they are still running and in one terminal run the next ``ros2 launch`` command:

```
ros2 launch camera_lidar_range_test range_demo.launch.xml
```

# 4. Demo 3 - Reactive robot demo

This last program will run a simple reactive robot that will avoid colision with objects. The robot moves forward until a nearby object is identified in front of him. The robot then rotates to the left until the object is no longer in his front and proceeds to move forward in the new direction.

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





