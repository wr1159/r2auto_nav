# ROS2 Code for EG2310, AY22/23 Sem2 by Group 11

This is Group 11's repository for EG2310. Here we attempt to modify a TurtleBot3 to navigate through a restaurant to deliver a can it receives from a dispenser to a certain table.

## What's in this repo
This repository contains all the code used for the mission in EG2310. The directory tree on the ubuntu laptop to control the TurtleBot and the RPi on the TurtleBot is also included below for reference. For this project, the RPi version used was RPi 3B+ and the microcontroller used for the dispenser was DOIT ESP32 Devkit V1 Board. The software, hardware, and steps required to replicate the working TurtleBot is explained in detail in the report. 

Follow Section 8 - Assembly of the [final report](EG2310_Group11_FinalReport.pdf) before proceeding with [starting the mission](#start-the-mission). 

## Important Files
View section 8.4 - Algorithm Overview of the [final report](EG2310_Group11_FinalReport.pdf) for a detailed breakdown for the program.

* [r2table_nav.py](r2table_nav.py) is the master program that controls the TurtleBot during the delivery mission, completing the 4 Phases of Dispense, Delivery, Collection and Return.
* [r2waypoints.py](r2waypoints.py) is the program that allows the users to set waypoints in case of an altered layout.
* [waypoints.json](waypoints.json) is the json file that stores the waypoints for each table.
* [map2base.py](map2base.py) is a publisher program that publishes a Pose message to locate the TurtleBot relative to the starting position.
* [switchpub.py](switchpub.py) is a publisher program running on the RPi that publishes the state of the microswitch.
* [ESP32/ESP32.ino](ESP32/ESP32.ino) is a ESP32 program that controls the Dispenser with functions that Display Input, Send input through MQTT, Rotate Servo Motor.

## Dependencies

### Dispenser
* [Adafruit GFX](https://github.com/adafruit/Adafruit-GFX-Library) by Adafruit
* [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) by Adafruit
* [ESP32Servo](https://github.com/adafruit/Adafruit_SSD1306) by Kevin Harrington, John K. Bennett
* [Keypad](http://playground.arduino.cc/Code/Keypad) by Mark Stanley, Alexander Brevig
* [PubSubClient](https://pubsubclient.knolleary.net/) by Nick O’Leary

### Remote Laptop
* [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html) by ROS2 Team
* [Paho MQTT Client](https://pypi.org/project/paho-mqtt/) by Eclipse Foundation

### RPi
* [ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html) by ROS2 Team
* [RPi.GPIO](https://pypi.org/project/RPi.GPIO/) by Ben Croston
* [MQTT 2.0](https://mosquitto.org/download/) by Eclipse Foundation


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
### Run Master Script
```
  tablenav
```
### Run Waypoint script instead of Master Script in case of new layout.
```
  r2waypoints
```

## Troubleshooting
View Section 10.2 - Troubleshooting - Software of the [final report](EG2310_Group11_FinalReport.pdf) to troubleshoot common problems faced when operating the system.
