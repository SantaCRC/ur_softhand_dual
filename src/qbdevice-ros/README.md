![qbrobotics logo](logo.png)

This repository contains the building blocks for ROS related applications based on _qbrobotics®_ devices. It is composed by all the device-independent structures to interface our devices with the ROS ecosystem, therefore is barely usable as is.

Please, refer to the following wikis respectively if you are dealing with a _qbhand_, a _qbmove_, or several _qbrobotics®_ devices connected together:
- [_qbhand_](http://wiki.ros.org/Robots/qbhand)
- [_qbmove_](http://wiki.ros.org/Robots/qbmove)

## Table of Contents
1. [Usage](#markdown-header-usage)
1. [ROS Packages Overview](#markdown-header-ros-packages-overview)
1. [Support, Bugs and Contribution](#markdown-header-support)
1. [Purchase](#markdown-header-purchase)

## Usage
>Even if these ROS packages can be installed properly as a standalone application, they are barely usable alone. Please, refer to [_qbhand_](http://wiki.ros.org/Robots/qbhand) and/or [_qbmove_](http://wiki.ros.org/Robots/qbmove) for detailed information of how to setup and use the specific ROS packages for your _qbrobotics®_ device.

To install this package via sources in your workspace follow this steps:

1. Clone the `qb_device` packages to your colcon Workspace, e.g. `~/colcon_ws`:
   ```
   cd `~/colcon_ws/src`
   git clone --recurse-submodules https://bitbucket.org/qbrobotics/qbdevice-ros.git
   cd qbdevice-ros
   git checkout production-humble
   git submodule update --init --recursive
   ```

1. Compile the packages using `colcon`:
   ```
   cd `~/colcon_ws`
   colcon build --packages-up-to qb_device
   ```
   **Note:** depending on your ROS installation, you may need some extra packages to properly compile the code. You can install ROS dependencies using `rosdep`:
   ```
   cd `~/colcon_ws`
   rosdep install --from-paths src --ignore-src -r -y

>Do not be worried if you have already followed the above installation instructions: simply proceed with your specific device installation and everything will be fine.

## ROS Packages Overview
| |Packages|
|---:|---|
|[qb_device](http://wiki.ros.org/qb_device): |[qb_device_bringup](http://wiki.ros.org/qb_device_bringup), [qb_device_control](http://wiki.ros.org/qb_device_control), [qb_device_description](http://wiki.ros.org/qb_device_description), [qb_device_driver](http://wiki.ros.org/qb_device_driver), [qb_device_hardware_interface](http://wiki.ros.org/qb_device_hardware_interface), [qb_device_msgs](http://wiki.ros.org/qb_device_msgs), [qb_device_srvs](http://wiki.ros.org/qb_device_srvs), [qb_device_utils](http://wiki.ros.org/qb_device_utils)|

## Support, Bugs and Contribution
Since we are not only focused on this project it might happen that you encounter some trouble once in a while. Maybe we have just forget to think about your specific use case or we have not seen a terrible bug inside our code. In such a case, we are really sorry for the inconvenience and we will provide any support you need.

To help you in the best way we can, we are asking you to do the most suitable of the following steps:

1. It is the first time you are holding a _qbrobotics®_ device, or the first time you are using ROS, or even both: it is always a pleasure for us to solve your problems, but please consider first to read again the instructions above and the ROS tutorials. If you have ROS related questions the right place to ask is [ROS Answers](http://answers.ros.org/questions/).
1. You are a beginner user stuck on something you completely don't know how to solve or you are experiencing unexpected behaviour: feel free to contact us at [support+ros at qbrobotics.com](support+ros@qbrobotics.com), you will receive the specific support you need as fast as we can handle it.
1. You are quite an expert user, everything has always worked fine, but now you have founded something strange and you don't know how to fix it: we will be glad if you open an Issue in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS).
1. You are definitely an expert user, you have found a bug in our code and you have also correct it: it will be amazing if you open a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS); we will merge it as soon as possible.
1. You are comfortable with _qbrobotics®_ products but you are wondering whether is possible to add some additional software features: feel free to open respectively an Issue or a Pull Request in the package of interest on [our Bitbucket](https://bitbucket.org/account/user/qbrobotics/projects/ROS), according to whether it is just an idea or you have already provided your solution.

In any case, thank you for using [_qbrobotics®_](https://www.qbrobotics.com) solutions.

## Purchase
If you have just found out our company and you are interested in our products, come to [visit us](https://www.qbrobotics.com) and feel free to ask for a quote.