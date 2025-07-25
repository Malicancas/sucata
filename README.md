
## Autores

- [@José Craveiro](https://github.com/Malicancas)

- [@Guilherme Filipe](https://github.com/guilassas)



# Sucata - ROS 2 Jazzy


Projeto open source de robótica móvel com ROS 2 para simulação, controle e integração de sensores num robô de 4 rodas.

## Descrição

O **Sucata** é um robô de 4 rodas desenvolvido com o framework **ROS 2 (Jazzy)**, pensado para simulação no **Gazebo Harmonic** e controle via `ros2_control`. O projeto visa facilitar testes, integração de sensores (como LIDAR, camera para leitura de ArUco markers, camera de profundidade e IMU) e diferentes métodos de operação.

## Demonstração

<p float="left">
  <img src="assets/gazebo.png" width="250" alt="Simulação no Gazebo"/>
  <img src="assets/aruco.png" width="250" alt="Detecção de ArUco"/>
  <img src="assets/nunchuck.jpeg" width="250" alt="Controle Nunchuck BLE"/>
</p>

## Funcionalidades do Projeto

- Mapeamento SLAM com LiDAR 2D e sensores de profundidade ✅

- Navegação autônoma usando Navigation2 ✅

- Docking automático com reconhecimento de marcadores ArUco ✅

- Controle manual via joystick personalizado ✅

- Fusão sensorial (IMU + odometria) com EKF ✅

## Tecnologias Utilizadas

- **ROS 2 Jazzy** (Ubuntu 24.04)
- **Gazebo Harmonic**
- `ros2_control`
- `ros2_controllers`
- `robot_state_publisher`
- `joint_state_controller`
- `nav2_bringup`
- `nav2_amcl`
- `slam_toolbox`
- `twist mux`
- `robot_localization` 
- **LIDAR** (`urg_node2`)
- **Depht Camera** (`openni2_camera`)
- **ArUco Markers** (`ros2_aruco`)

## Instalação

### Clone o repositório e dependências:

```bash
mkdir -p ros2ws/src
cd ros2ws/src
git clone https://github.com/Malicancas/sucata.git
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
git clone https://github.com/Hokuyo-aut/urg_node2.git
```

### Build e execução:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros2_ws/src/sucata/modelos #altera o caminho dos modelos do Gazebo para as boxs
cd ..
colcon build
source install/setup.bash
ros2 launch sucata launch_sim.launch.py
```

### Para uso do controle BLE (Nunchuck):

```bash
git clone https://github.com/Malicancas/nunchuck-BLE
# Siga as instruções no repositório para configuração
```
