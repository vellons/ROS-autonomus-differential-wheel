# ROS (Robot Operating System)
Elenco di comandi utili per utilizzare ROS con Docker
Esempi ROS: https://github.com/ros/ros_tutorials/tree/noetic-devel/rospy_tutorials
PDF utile per gli esempi: https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwiN7dKyoL7zAhWhM-wKHVS6ANUQFnoECAYQAQ&url=http%3A%2F%2Fricopic.one%2Frobotics%2Frobotics_partial_06_03_Running_and_launching_ROS_nodes.pdf&usg=AOvVaw2WV9vL9A1EiwuoN8C2y58Q

TOOL grafico su ubuntu: sudo apt-get install -y rviz 
Elenco porte aperte: netstat -tulpn | grep LISTEN


## Installazione
- Installare docker (su rapberry usare lo script ufficiale e aggiungere pi al gruppo docker)
- $ docker pull ros:noetic-ros-base (per raspberry) [https://hub.docker.com/_/ros?tab=description]
- $ docker images (per vedere le immagini scaricate)


## Run immagine per testare
- $ docker run -it ros:noetic-ros-base /bin/bash
$ export ROS_MASTER_URI=http://localhost:11311    !!!
$ roscore &
& rostopic list
$ rospack list-names
$ exit


## Run immagine con volume condivisto
- $ docker run -v "/home/pi/ros/:/root/ros/" -it ros:noetic-ros-base /bin/bash
- (uppure login) docker exec -it <NAME> /bin/bash


Su ubuntu: source /opt/ros/melodic/setup.bash
## Run immagine con rplidar
- ls -l /dev | grep ttyUSB
- sudo chmod 666 /dev/ttyUSB0
- docker run --privileged -v "/dev/ttyUSB0:/dev/ttyUSB0" -v "/home/pi/rplidar_ros/:/root/ros/" -p 11311:11311 -it ros:noetic-ros-base /bin/bash
docker run --network host --privileged -v "/dev/ttyUSB0:/dev/ttyUSB0" -v "/home/pi/rplidar_ros/:/root/ros/" -it ros:noetic-ros-base /bin/bash
- catkin_make
- source devel/setup.bash
- roslaunch rplidar_ros rplidar.launch
In un altro terminale:
- rostopic echo -n1 /scan
- rosrun rplidar_ros rplidarNodeClient

docker run --network host --privileged -v "/dev/ttyUSB0:/dev/ttyUSB0" -v "/home/pi/hector_slam/:/root/ros/" -it ros:noetic-robot /bin/bash
Robot:
export ROS_IP=192.168.1.120
export ROS_MASTER_URI=http://192.168.1.120:11311
Ubuntu:
export ROS_IP=192.168.1.132
export ROS_MASTER_URI=http://192.168.1.120:11311


## Creazione paccketto base (Nella cartella di sviluppo)
$ catkin_create_pkg talker_listener rospy (per creare una nuova cartella)
$ chmod +x *.py
$ catkin_make
$ source devel/setup.bash   !!!
$ roscd
$ rosrun rospy_tutorials talker




# ROS2
docker pull ros:foxy-ros-base
docker run -v "/home/pi/ros2/:/root/ros/" -it ros:foxy-ros-base /bin/bash


https://automaticaddison.com/how-to-build-an-indoor-map-using-ros-and-lidar-based-slam/


Ros.h arduino: https://www.youtube.com/watch?v=6F5mNGdBhlU

roslaunch rosserial_python serial_node.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rostopic echo cmd_vel

docker run --network host --privileged -v "/dev/serial0:/dev/serialArduino" -v "/home/pi/serial/:/root/ros/" -it noetic-ros-base-erl /bin/bash




## autonomus_differential_wheel
Installa docker e docker-compose sul tuo raspberry 64 bit
Guida: https://dev.to/elalemanyo/how-to-install-docker-and-docker-compose-on-raspberry-pi-1mo
Login ed entra nella cartella del progetto. 
Tutti i dispositivi (Arduino e lidar) devono essere staccati

Verifica che nel file docker-compose gli indirizzi IP sono corretti

Arduino si connette solitamente:  /dev/serial0 o 1
Il lidar si connette solitamente: /dev/ttyUSB0
Attacca Arduino e aspetta 20 secondi

Tieni il lidar staccato per verificare che la porta non esiste
$ ls -l /dev | grep ttyUSB

Connetti il lidar per verificare la presenza
$ ls -l /dev | grep ttyUSB
$ sudo chmod 666 /dev/ttyUSB0
$ ls -l /dev | grep ttyUSB

Per testare la comunicazione con il lidar esegui:
$ cat /dev/ttyUSB0
se il lidar si ferma la comunicazione avviene correttamente


Entra nel container (parte automaticamente serial_node)
$ docker-compose run ros-run
Dovresti vedere il warn "Arduino connected" dopo 20 secondi circa (altimenti reset Arduino)

Apri un nuovo terminale per collegarti al container
Utilizza questo comando per vedere il nome del container avviato
$ docker ps

$ docker exec -it autonomus_differential_wheel_ros-run_run_258775d555c1 /bin/bash
$ source devel/setup.bash

Per testare la comunicazione con il lidar dentro il container esegui:
$ cat /dev/ttyUSB0
se il lidar si ferma la comunicazione avviene correttamente

Visualizza i topic di ROS
$ rostopic list

Avvia il nodo per gestire il lidar
$ roslaunch rplidar_ros rplidar.launch
Per mezzo secondo il lidar si ferma. 
Le letture del lidar sono pubblicate nel topic /scan

Avvia il tool per comandare il robot
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
Puoi visualizzare i comandi inviato aprendo un nuovo terminale nel container
$ rostopic echo cmd_vel


Su un desktop ubuntu:
Devi compilare hector-slam/
rimuovi il file CATKIN IGNORE?
$ catkin_make
$ source devel/setup.bash
$ export ROS_IP=ip-ubuntu
$ export ROS_MASTER_URI=http://ip-raspi:11311

Testa la comunicazione con il raspberry
$ rostopic list

Avvia RVIZ
$ roslaunch hector_slam_launch tutorial.launch







