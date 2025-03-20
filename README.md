# Manipulation d'un UR3e sous ROS2 Humble

Dans ce tuto nous réaliserons l'installation et utiliserons un robot UR3e sous ROS2. Nous verrons comment utiliser Moveit! mais aussi comment réaliser un premier programme python.

# Ressources utilisées
- Linux Ubuntu 22.0.4
- ROS2 Humble
- Python 3.10.12
- Drivers UR pour ROS2 Humble
- MoveIt! / Rviz

# Installation de ROS2 Humble
Pour plus de détails sur l'installation, rendez-vous sur le site officiel de ROS2 Humble : 
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

**Définir les paramètres régionaux**
```
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

**Initialiser les sources**
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Installer les packages ROS 2**
```
sudo apt update
```
```
sudo apt upgrade
```
```
sudo apt install ros-humble-desktop
```
ROS2 Humble est désormais installé
