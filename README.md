# ME 795.02: Robotic Systems: Modeling, Planning, and Control

## Date Modified: 08/28/2026
Maintainer: Contact the instructor

# Subdirectory Structure:

- `core`: support code that I provde, including helper functions for publishing commands to the robot
- `lib`: This is where you will implement algorithms to solve the projects. 
- `projects`: This is where testing code will live and will use code in the `core` and `lib` directories. 
- `ros`: this contains all necessary code to launch the simulation. You will not need to work in this directory. We are using the `franka_description` ros package as a subtree in git. This preserves the git history, but does not require working directly with a submodule. For this class, we will not do any updating to this repository. 

# Native Install Instructions (Not required for Virtual Machine Instructions)
If you are already running Ubuntu 24.04 and want to install the class software on your own you can follow these instructions. Note that this documentation is what I used to set up the VM image and rely heavily on my experiences setting these types of tools up.
If you run into issues I will do my best to try and help resolve them, but the recommendation for this class is to use the virtual machine.

## Operating System:
We will run Ubuntu 24.04 and ROS2 Jazzy along with Gazebo Harmonic

### Step 1:

- Install ROS2 Jazzy desktop full [link](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- Do bask working test in the setup instructions
- Put `source /opt/ros/jazzy/setup.bash` in ~/.bashrc 

### Step 2:

- Install emacs, vim, and VS Code (essentially install your preferred text editor)

### Step 3:

- Install ROS Jazzy packages that are managed by package manager and will be in `/opt/ros/jazzy` directory. To install packages you will type:

`$ sudo apt-get <PACKAGE NAME>`

Packages to install: 
1) `ros-jazzy-moveit`
2) `ros-jazzy-navigation2`
3) `ros-jazzy-turtlebot4-desktop`
4) `ros-jazzy-turtlebot4-simulator ros-jazzy-irobot-create-nodes`
5) `ros-jazzy-ros-gz ros-jazzy-sdformat-urdf`
6) `ros-jazzy-gz-ros2-control`
**Note**: some of these packages have dependencies and install other packages and it may say that you already have the newest version when you go to install


### Step 4: 
