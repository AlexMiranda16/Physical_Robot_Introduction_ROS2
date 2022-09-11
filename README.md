# Physical_Robot_Introduction_ROS2


Repository for the demo files and guide to setup and run them.

# 1. Prerequesites


Besides the software, you need to create a new ROS2 workspace to place all the robot packages. In this example, the ROS2 workspace was named "dev_ws" but you can give it another name.

Open a terminal and create a new directory. Then, go to the "src" directory and clone the repository. Check the dependencies of the workspace and then build it using the colcon command.

```
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
(METER O GIT CLONE)
cd ..
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Don't forget that in every new terminal opened, you need to source the setup script of this workspace. Alternatively, you can add the source command to the .bashrc file in a similar way to that shown in the intallation guide.

```
source install/setup.bash
```

For more information about the ROS2 workspace creation visit "[Creating a workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)".




# 3. Demo 1 - Keyboard robot motors control

This first demo has the purpose to test the motors operation.

The program controls the robot movement using the "wasd" keyboard keys
  (w - foward;
  a - left rotation;
  s - backward;
  d - right rotation;).
  To stop the motors, any other key can be pressed.
  
  With the terminal
