# Z-Wave Motion Sensor PIR Example #
![Type badge](https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_common.json&label=Type&query=type&color=green)
![Technology badge](https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_common.json&label=Technology&query=technology&color=green)
![License badge](https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_common.json&label=License&query=license&color=green)
![SDK badge](https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_common.json&label=SDK&query=sdk&color=green)
![Build badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_build_status.json)
![Flash badge](https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_common.json&label=Flash&query=flash&color=blue)
![RAM badge](https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/SiliconLabs/application_examples_ci/master/zwave_applications/z_wave_motion_sensor_pir_application_common.json&label=RAM&query=ram&color=blue)

## Summary ##

This project shows the implementation of PIR sensor with Z-Wave. The PIR sensor on the occupancy sensor EXP board enables the internal ADC of ZGM130S to take periodic measurements. CRYOTIMER is
set to signal the ADC using PRS. The Op-Amp is configured to use the external one on the board. A simple motion detection algorithm is implemented to post-process ADC samples. Once the motion 
is detected, it will send an event to the Z-Wave application layer to start transmitting RF frames.

The target application of the PIR sensor would be smart lighting or alarm systems in home automation. Whenever certain motion of the human body is detected, the system will either turn on the light
or the alarm. You can build a Z-Wave network and create an association such that the PIR sensor node will turn on the alert light upon motion detection. See figure below.

![](doc/z_wave_network.png)

Node 1 is the Z-Wave controller. Node 2 is the PIR sensor. Node 3 is the alert light(a binary switch device). 

## Gecko SDK version ##

v3.0.0

## Z-Wave SDK version ##

v7.14.1.0

## Hardware Required ##

- Z-Wave Controller
	- UZB7 Controller USB Dongle

- PIR Sensor Node
	- WSTK Mainboard (BRD4001A)
	- ZGM130S Radio board (BRD4202A)
	- Occupancy Sensor EXP board (BRD8030A)
	
- Light Node
	- WSTK Mainboard (BRd4001A)
	- ZGM130S Radio board (BRD4202A)
	- Buttons and LEDs EXP board (BRD8029A)

## Setup ##

To test the PIR sensor, you need to connect the occupancy sensor EXP board to the WSTK board through the expansion header. Then, you should program the ZGM130S with the z_wave_motion_sensor_pir_zgm130s.sls project.

To add the node to Z-Wave network, you need to plug the UZB7 Controller USB Dongle into the PC and run the Z-Wave PC controller software. You should also program the other ZGM130S with the z_wave_binary_switch_zgm130s.sls
project and add that to the Z-Wave network as the light node. That project is the same as the SwitchOnOff demo provided in Simplicity Studio. After including 2 nodes into the Z-Wave network, you can use the PC controller software to create the association between the PIR sensor node and the light node.

## How It Works ##

1. Push PB1 on WSTK to enter learn mode to add/remove the device to the network.
2. Push PB0 on WSTK to start/stop PIR motion sensor measurements.
3. RGB LED on the radio board will turn green whenever motion is detected.
4. If the light node is also added to the network and the proper association is created, LED0 on the expansion board of the light node will also be turned on
   whenever motion is detected and be turned off if no motion is detected for 10s.

## .sls Projects Used ##

- z_wave_motion_sensor_pir_zgm130s.sls
- z_wave_binary_switch_zgm130s.sls

## Special Notes ##

Hardware Modification: Since Buttons and LEDs EXP board (BRD8029A) is replaced with Occupancy Sensor EXP board (BRD8030A) at the PIR sensor node, both buttons and LEDs are limited in this application. It's required to tie pin P4 and P12 together at the Breakout Pads to allow EM4 wake-up using PB0.

Cross Reference: This example provides a reference design using the [PIR driver](https://github.com/SiliconLabs/platform_hardware_drivers/tree/master/pir_ira_s210st01) under platform harware drivers.