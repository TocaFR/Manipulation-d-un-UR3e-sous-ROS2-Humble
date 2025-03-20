# Manipulation d'un UR3e sous ROS2 Humble

Dans ce tuto nous réaliserons l'installation et utiliserons un robot UR3e sous ROS2. Nous verrons comment utiliser Moveit! mais aussi comment réaliser un premier programme python. Nous avons connecté le robot ainsi que le PC sous Ubuntu 22.04 en Ethernet sur un routeur. 

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
**Lancer ROS**
ROS2 Humble est désormais installé, afin de manipuler ROS il faut le sourcer à chaque nouveau terminal créé grâce à cette commande :
```
source /opt/ros/humble/setup.bash
```


Afin d'éviter de retaper la commande à chaque fois, on peut simplement modifier le fichier .bashrc afin d'exécuter la commande à chaque nouveau terminal automatiquement :
```
cd
gedit .bashrc
```
Un fichier texte va s'ouvrir, rajouter simplement la commande ``source /opt/ros/humble/setup.bash`` tout à la fin de ce fichier. 

# Installation des drivers UR pour ROS2
Plus d'informations sur le github suivant : https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
Nous utiliserons directement les drivers fournis par Universal Robots afin de contrôler le robot avec ROS2. Nous les installons via cette commande : 
```
sudo apt-get install ros-rolling-ur
```
Cependant, le robot nécessite un module complémentaires (aussi appelé URCaps) permettant d'éxecuter des commandes par contrôle externe (Par addresse IPv4).


