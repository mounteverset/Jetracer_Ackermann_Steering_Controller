# ROS Control x Jetracer

## Installation

### Setup for the Waveshare Jetracer

#### Bill of Materials 

| Part | Description |
| ------ | ------ |
| Waveshare Jetracer AI Kit | Autonomous Car with a Jetson Nano 4GB ([Description on Waveshare Webpage](https://www.waveshare.com/jetracer-ai-kit.htm)) |
| Arduino Uno | Microcontroller board based on the ATmega328P |
| 12V DC Encoder Motor | Replaces the standard DC Motors in the back of the Jetracer ([Description on the Robotshop Website](https://www.robotshop.com/de/de/12v-dc-motor-251rpm-mit-encoder.html)) |
| Base Plate | Replaces the standard aluminium base plate of the Jetracer to make room for the encoder motors (.stl file found in /construction folder) |
| Gearmotor Bracket | Needed to mount the new motors to the new base plate |
| RPLIDAR A1M8 | 360° Laser Scanner [Description on Slamtec Website](https://www.slamtec.com/en/Lidar/A1) |
| M2.5 & M3 Hex Spacer and Bolts | To mount the LIDAR on top of the Jetson

### Hardware Wiring

To wire up the Arduino to the Rear Motors correctly the 6 Pins have to be connected correctly. 

<details><summary>Encoder Pinout</summary>
![Encoder](/resources/encoder.png)
</details>

<details><summary>Connections to Arduino</summary>
![Encoder to Arduino](/resources/encoder_arduino.jpg)
</details>

### Software Setup

1. Follow the steps laid out in the Waveshare [Jetracer Wiki](https://www.waveshare.com/wiki/JetRacer_AI_Kit)
2. Install ROS Melodic Full on both the Jetson and your Workstation as described on the [ROS.org Website](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
3. Create a new catkin workspace with:
    ```
    mkdir catkin_ws
    cd catkin_ws 
    mkdir src
    ```
4. Clone this repo into the src folder
    ```
    cd src
    git clone https://gitlab.rz.htw-berlin.de/s0570976/ros-control-x-jetracer.git
    git submodule init
    git submodule update
    cd ros_control_boilerplate
    git fetch
    git checkout melodic-devel
    cd ~/catkin_ws
    ```
5. Install all needed dependencies with rosdep
    ```
    sudo apt-get install python-rosdep
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
6. Install all needed ROS functionalities on the workstation (ROS master)
    ```
    sudo apt-get install ros-melodic-navigation
    sudo apt-get install ros-melodic-gmapping
    ```
7. Install all needed ROS functionalities on the Jetson
    ```
    sudo apt-get install ros-melodic-ackermann-steering-controller
    sudo apt-get install ros-melodic-ackermann-steering-msgs
    sudo apt-get install ros-melodic-rplidar-ros
    sudo apt-get install ros-melodic-joint-state-controller
    sudo apt-get install ros-melodic-robot-state-publisher 
    ```
8. Build the workspace and source (Workspace and Jetson)
    ```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```
9. To make the nodes visible in the ROS network the IP adress of the host device has to be added to the environment variables. This is best done via an export statement in the terminal. To find the IP adress "ifconfig"
Adding the statement to the .bashrc both on the Host and Client makes sense, e.g.

    For the host:

    ```
    export ROS_MASTER_URI=http://192.168.178.53:11311
    export ROS_HOSTNAME=192.168.178.53
    ```
    
    For the client:


    ```
    export ROS_MASTER_URI=http://192.168.178.53:11311
    export ROS_HOSTNAME=192.168.178.31
    ```

## Usage

### Jetson

- Launch the required nodes and parameters (each in a new and sourced terminal) with: 

    ```
    roslaunch fennec_bringup fennec_bringup.launch
    rosrun rosserial_python serial_node.py  _port:=/dev/ttyACM0 _baud:=115200
    ```

### Workstation

- Launch the required nodes and parameters (each in a new and sourced terminal) with: 

    ```
    roscore
    roslaunch fennec_bringup laptop_bringup.launch
    ```

- You can then see the robot representation and the generated map in Rviz and send navigation goals to let the robot drive there







