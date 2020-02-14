# Motor Lab

## Description
This repository contains our code for Motor Lab for Mechatronics Design 2020. For this lab, we integrated the three sensors from [Sensor Lab](https://github.com/Robo-Dutchmen/Sensor-Lab) with three Hebi Actuators. The GUI allows the user to see the values of all three sensors as well as toggle how the motors are controlled. The sensors can control the motor positions based on their values. The GUI also allows the user to manually control the motors' positions and velocity.

## Installation

### Network Setup
To ensure that it is possible to communicate with the Hebi Actuators, ensure that all the Hebi Actuators are powered with the 24V power and that all actuators are connected to an external router through Ethernet cables. Further, ensure that the working computer is connected to the router either through WiFi or Ethernet.

### GUI
Ensure that App Designer is installed in MATLAB by double clicking ```matlab_gui.mlapp```. This should result in a view similar to the following:

### Arduino
To wire up the Arduino, follow the wiring schematic from [Sensor Lab](https://github.com/Robo-Dutchmen/Sensor-Lab).

## Deployment
1. Connect the Arduino Uno to the computer.
2. Ensure that the Hebi network is properly setup by running ```HebiLookup``` and checking if the actuators are present.
3. Ensure that the Hebi actuators are powered.
3. Double click ```matlab_gui.mlapp```. 
4. Press Run on the top right.
