URSAII Driver [![Build Status](https://travis-ci.org/mars-uoit/URSAII-Driver.svg)](https://travis-ci.org/mars-uoit/URSAII-Driver)
=============
Version 0.1.1

This is a ROS Node and C++ driver for the URSAII multi-channel analyser (MCA).  The ROS node implements all the necessary commands to start taking readings from a radiation detector and publish them over ROS.  While the library implements all the available commands.

## What is this repository for? ##
### ROS Node###
This software will allow you to get radiation measurements in either gross counts (in MCS Mode) or using the URSA's 12 bit ADC to capture spectra.  This data then can be transported via custom messages to other ROS Nodes.

### C++ Library ###
I tried to make the driver portion of the repo as stand-alone as possible. I exposes functions to execute any of the commands that URSA will respond to.  Keep in mind though that some commands are meant to only be executed by factory personnel and setting parameters in a incorrect manner could damage the URSA or the detector head. Check out the doxygen documentation for the ursa::Interface class.

#### Note ####
If you are familiar with the standard software provided to operate the URSA there is a major difference between that software and this; This software will only provide you with the raw readings from URSA, if energy calibration or any other conditioning must be done you must implement it in your project.



## How do I get set up? ##

The ROS node can be compiled in a catkin workspace and its dependencies can be installed with rosdep. The Library depends on several boost libraries and William Woodall's serial library.  The Library can be compiled and installed using the Makefile (TODO).


### Contact ###
Repo Owner: Michael Hosmar mikehosmar@gmail.com
