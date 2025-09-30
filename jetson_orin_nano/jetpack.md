# JetPack Installation Guide

This is a guide on how to set up and install JetPack on the Jetson Orin Nano Developer Kit.

JetPack is a package offered by NVIDIA that contains an operating system (OS) and some out-of-the-box modules for use on Jetson hardware. This guide shows how to set it up on an SSD.

This method is required for flashing SSDs and is also an option for SD cards. For SD cards, however, normal OS installation methods (such as using ISO files on removable media) may be applied depending on the JetPack version intended.

## Requirements

- Host machine running Ubuntu 22.04 (for JetPack 6)  
- USB-C cable capable of data transfer  
- SSD  

## Guide

First, install NVIDIA’s [SDK Manager](https://developer.nvidia.com/sdk-manager). It will handle most of the installation and flashing process for the SSD. Then, follow these steps:

- Connect the host machine to the Jetson using a USB-C cable.  
- Boot the Jetson into recovery mode by short-circuiting the *FC REC* and *GND* pins under the heatsink (I did this using a screwdriver, but it can also be done with a female-to-female jumper or a dedicated jumper sold online).  
- Power up the Jetson (after powering up, you can remove the short-circuit tool if needed).  

After that, SDK Manager will prompt you to select your Jetson model and the software version to flash. It is important to note that the JetPack version determines which Ubuntu version will run on the Jetson. For my use case, I chose JetPack 6 so that it runs Ubuntu 22.04.

The flashing process takes around one hour.  
