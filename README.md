# Tandem Tree ExploreR
Per aqui podem posar una intro

# Related publication
Posar per citar es paper

# Installation
This code has been tested on Ubuntu 64-bit 20.04.6 LTS with ROS Noetic (desktop-full).

### Install Dependencies
```bash
sudo apt install python3-wstool \
python3-catkin-tools \
ros-noetic-cmake-modules \
protobuf-compiler \
autoconf \
ros-noetic-geographic-msgs \
python-is-python3 \
ros-noetic-mavros-msgs \
ros-noetic-octomap-server \
ros-noetic-teleop-twist-joy
```

### Create workspace
```bash
mkdir -p <workspace_name>/src
cd <workspace_name>
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```
### Install packages
Assuming you are in the workspace directory:
```bash
cd src
git clone https://github.com/Taucrates/ttr
cd ..
wstool init . ./src/ttr/ttr_installer.rosinstall
wstool update
catkin build
```

# Usage
In a console execute the following command to start up the simulation in the indoor AT scenario and the TTR explorer:
```bash
roslaunch ttr MAV_exploration_indoor_at.launch
```

Now in the TTR ui, on RViz, press first the button "Take-off" and when the UAV has taken off press "Start" to begin the exploration, in the following image you can see the user interface:

<img width="343" height="188" alt="TTR_ui" src="https://github.com/user-attachments/assets/6bb8eaf1-c5ad-4e5d-a2ee-d366984ad8b6" />
