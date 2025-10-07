
## Authors

- [@José Craveiro](https://github.com/Malicancas)

- [@Guilherme Filipe](https://github.com/guilassas)



# 🤖 S.U.C.A.T.A 2.0 - Unified System for Autonomous Cartography and ArUco Tracking

**SUCATA** is a 4-wheeled robot developed with the **ROS 2 Jazzy** framework, designed for simulation in **Gazebo Harmonic** and control via `ros2_control`. The project aims to facilitate testing, sensor integration (such as LIDAR and camera for reading ArUco markers) and different operation methods.

## Demonstration

<p float="left">
  <img src="assets/chassiNovo.jpg" width="250" alt="Sucata"/>
  <img src="assets/globalCostmap.gif" width="250" alt="Slam"/>
</p>

## 📃 Project Status

- Robot configured and simulated in Gazebo ✅
    - Functional control via `teleop_keyboard` ✅
    - Functional control via `Nunchuck` ✅
    - Navigation ✅
    - SLAM ✅
    - Depth camera - No integration in Navigation for now
    - ArUco detection - Docking algorithm needs to be worked on
    - YOLO11n - Algorithm needs work




## 📦 External Packages Used

- [**LIDAR**](https://github.com/Hokuyo-aut/urg_node2) (`urg_node2`)
- [**ArUco Markers**](https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco) (`ros2_aruco`)
- [**Asus Xtion Pro**](https://github.com/ros-drivers/openni2_camera) (`openni2_camera`)

## 💥 Components List
Components used in this project:

| | Component |
| --| --|
|1| Raspberry Pi 4B (4 GB)|
|2| SanDisk 32 GB SD Card (minimum)|
|3| [Skeleton Bot 4WD hercules mobile robotic platform](https://wiki.seeedstudio.com/Skeleton_Bot-4WD_hercules_mobile_robotic_platform/)|
|4| [IMU MPU6050](https://pt.aliexpress.com/item/1005008404467219.html?src=google&pdp_npi=4@dis!EUR!1.92!1.38!!!!!@!12000044896717467!ppc!!!&gQT=2)|
|5| LiDAR URG-04LX|
|6| Raspberry Pi camera module 3|
|7| Asus Xtion Pro|
|8| 50mm 12V Fan|
|9| [50mm Fan Stand](https://www.printables.com/model/443438-raspberry-pi-4-bracket-for-50mm-fan)|
|10| LiPo 4S 2400mAh 14.8V Battery|
|11| Modified Nunchuck (see Nunchuck repository)|
|12| USB Dock with external power supply|
|13| TPLink MR3020 Router (optional)|



## Installation
The following commands assume that the user already has a functional ROS2 environment and Gazebo Harmonic, consult the [documentation](https://docs.ros.org/en/jazzy/index.html) for more information.

**Simulation setup:**
```bash
# Create workspace
mkdir -p ros2ws/src
cd ros2ws/src

# Clone main repository
git clone https://github.com/Malicancas/sucata

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Follow the steps to install Gazebo Harmonic
https://gazebosim.org/docs/harmonic/install_ubuntu/

```
**Build and simulation:**
```bash
# Inside the /ros2ws folder
colcon build --symlink-install
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/my-local-models/
ros2 launch sucata launch_sim.launch.py
```

**Raspberry Pi setup:**

For the following configurations, it is recommended to connect to the Raspberry Pi through an SSH connection.

```bash
mkdir -p ros2ws/src
cd ros2ws/src

# Clone the RPI branch
git clone --single-branch --branch RPI https://github.com/Malicancas/sucata.git

# Package for ArUco detection
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git

# Package for LiDAR
git clone https://github.com/Hokuyo-aut/urg_node2.git

# Package for depth camera
git clone https://github.com/ros-drivers/openni2_camera

```


**For BLE controller use (Nunchuck):**

Clone the repository mentioned below and follow the instructions.
```bash
git clone https://github.com/Malicancas/nunchuck-BLE
```
