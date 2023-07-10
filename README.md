# Short Project Description

This repository contains the code for my final Thesis project at IMT University of Stuttgart.
The goal of the project was the development of a bioimpedance monitor for improved needle positioning during epidural anesthesia.

Epidural anesthesia is a local anesthetic procedure close to the spinal cord. The main challenge is the identification of the epidural space with the needle, which is conventionally performed based on the subjective sensation of the anesthesiologist.
To provide an objective criterion for determining the location of the needle, bioimpedance-based tissue differentiation is performed at the needle tip.

# Prototype Description
The Prototype consists of a AD5941 Analog Frontend by Analog Devices and a Adafruit Feather M0 Microcontroller. The AD5941, passive Electronics and connectors for the electrodes and I/O are placed on a custom PCB, which is designed to directly attach to the microcontroller (similar to an Arduino Shield). The electronics assembly is mounted into a 3D-printed Case.

# Software Components
The Software is based on the publicly available distribution code for the AD5941 by Analog Devices. The ad5940lib library contains the firmware of the Chip. Additionally, there are Application examples for the AD5941 available for different use cases like measuring impedances or other voltammetric measurements. The example code is written for a different microcontroller, therefore the example code needs to be modified via a porting file. This porting file is based on the work of 



### Commit Test
test

### branch test
branch test
undoing test
