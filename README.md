# ROS2 Code for EG2310, AY22/23 Sem2 by Group 11

This is Group 11's repository for EG2310. Here we attempt to modify a TurtleBot3 to navigate through a restaurant to deliver a can it receives from a dispenser to a certain table.

## What's in this repo
This repository contains all the code used for the mission in EG2310. The directory tree on the ubuntu laptop to control the TurtleBot and the RPi on the TurtleBot is also included below for reference. For this project, the RPi version used was RPi 3B+ and the microcontroller used for the dispenser was DOIT ESP32 Devkit V1 Board. The software, hardware, and steps required to replicate the working TurtleBot is explained in detail in the report. 

Follow Section 8 - Assembly of the final report before proceeding with [starting the mission](#start-the-mission). 

## Important Files
View section 8.4 - Algorithm Overview of the final report for a detailed breakdown for the program.

[r2table_nav](r2table_nav.py) is the master program that controls the TurtleBot during the delivery mission, completing the 4 Phases of Dispense, Delivery, Collection and Return.
[r2waypoints](r2waypoints.py) is the program that allows the users to set waypoints in case of an altered layout.
[waypoints.json](waypoints.json) is the json file that stores the waypoints for each table.
[map2base.py](map2base.py) is a publisher program that publishes a Pose message to locate the TurtleBot relative to the starting position.
[switchpub.py](switchpub.py) is a publisher program running on the RPi that publishes the state of the microswitch.
[ESP32/ESP32.ino](ESP32/ESP32.ino) is a ESP32 program that controls the Dispenser with functions that Display Input, Send input through MQTT, Rotate Servo Motor.

## Directory Tree on Ubuntu Laptop
```
colcon_ws
├── build
├── install
├── log
└── src
     └── auto_nav
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        └── auto_nav
            ├── __init__.py
            ├── map2base.py
            ├── waypoints.json
            ├── r2waypoints.py
            └── r2table_nav.py
```

## Directory Tree on TurtleBot
```
turtlebot_ws
├── build
├── install
├── log
└── src
    └── hardware-bringup
        ├── package.xml
        ├── setup.cfg
        ├── setup.py
        └── sensors
            ├── __init__.py
            └── swtichpub.py
```

## Start the mission 
ENSURE that the ip address in the ESP32 and TableNumber.py is what is displayed by the broker in terminal after running.
```
hostname -I.
```
Run the following commands in different terminals
### Turtlebot Bringup
```
  ssh ubuntu@(ip-address-of-pi)
  rosbu
```
### Run Micro Limit Switch Publisher
```
  ssh ubuntu@(ip-address-of-pi)
  switchpub
```
### Run Cartographer
```
  rslam
```
### Run Map2Base publisher
```
  map2base
```
### Control the turtlebot to store waypoints
```
  r2waypoints
```
### Run Master Script
```
  tablenav
```

## Troubleshooting
View Section 10.2 - Troubleshooting - Software of the final report to troubleshoot common problems faced when operating the system.
