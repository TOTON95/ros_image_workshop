[🇪🇸](/README-ES.md "Spanish")

# ros_image_workshop

ROS workshop for image acquisition, processing and distribution

## Description 

This workshop pretends to teach new users to extract the images from different sources such as:

* Integrated Web Cams
* USB Web Cams
* Color/Depth Cameras such as Kinect
* WiFi cameras (IP Cameras, [GoPro](https://github.com/TOTON95/ros-gopro-driver))

Also, this workshop is designed to apply basic image processing and overlay creation for feature and information for the user. 

Finally, the distribution of images is explored by providing different stages of processed images to the ROS environment.

This workshop has been designed for ROS Kinetic, but is expected to work in:

- Melodic
- Noetic

The goal of this repository is to make the code as simple, organized, self-documented as possible. 

Complementary PDF slides are included for better code explanation.

## Instructions

*This workshop assumes that you already have a ROS environment installed on your system, if not, please go to the [official documentation](http://wiki.ros.org/ROS/Installation) and follow the instructions. Then, come back to this repository.*



Before starting, we have to install a few things:

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-usb-cam
```

