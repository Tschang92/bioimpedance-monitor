# Short Project Description

This repository contains the code for my final Thesis project at IMT University of Stuttgart.
The goal of the project was the development of a bioimpedance monitor for improved needle positioning during epidural anesthesia.

Epidural anesthesia is a local anesthetic procedure close to the spinal cord. The main challenge is the identification of the epidural space with the needle, which is conventionally performed based on the subjective sensation of the anesthesiologist.
To provide an objective criterion for determining the location of the needle, bioimpedance-based tissue differentiation is performed at the needle tip.

# Prototype Description
The Prototype consists of a AD5941 Analog Frontend by Analog Devices and a Adafruit Feather M0 Microcontroller. The AD5941, passive Electronics and connectors for the electrodes and I/O are placed on a custom PCB, which is designed to directly attach to the microcontroller (similar to an Arduino Shield). The electronics assembly is mounted into a 3D-printed Case.

# Software Components
The Software is based on the publicly available distribution code for the AD5941 by Analog Devices. 

## ad5940lib
The ad5940lib library contains the firmware for the AD5941. 

## Impedance.c
This file handles the impedance measurement via the AD5941.

## ArduinoPort_AdafruitFeather.cpp / main_AdafruitPort.cpp
The provided example code and firmware by Analog Devices are written for a different microcontroller. To make them work with the Adafruit Feather, these porting files are used.

## main.cpp
This file is the main file, that is running on the microcontroller. It handles the connection between microcontroller and AD5941, initialization of the hardware and measurement parameters, user input, tissue classification algorithm and display of results.  



### Commit Test
test

### branch test
branch test
undoing test
