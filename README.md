# ROS2 Cheatsheet

Cheatsheet for all basic ROS2 Humble commands.

Table of contents

-   Creating a workspace
-   Creating a package
-   Topics
-   Publishers and Subscribers
-   Services
-   Action Server and Client

# 1. Creating a workspace

To work with ros2, you first needed to create a "workspace" which is just a directory suffixed by "\_ws" for good practice (such as "ros2_ws", "turtlesim_ws"). To begin, navigate to the location wherein you want your ros2 workspace to reside and run the following commands.

```
mkdir ros2_ws
cd ros2_ws
mkdir src
cd ..
```

Now we created a workspace directory with an src directory where all of our packages will be. It is time to build it. Building your workspace should always be done after creating packages, adding new files or editing files in existing packages, etc.

```
colcon build
```

Every time we build our workspace, we need to source the `setup.bash` file found in `ros2_ws/install`. So instead of running this command every time, we can add it to our `.bashrc` file.

```
gedit ~/.bashrc
```

Type this command at the end of the file

```
source ~/ros2_ws/src/install/local_setup.bash
```

Now you have a workspace and it'll be sourced every time you open a terminal!

# 2. Creating a package

In order to start working in your workspace, you need to create a package which will hold all of your code.