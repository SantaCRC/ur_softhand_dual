^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qb_device_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.4 (2024-06-07)
------------------

4.1.3 (2024-05-09)
------------------
* Fixing my mess with releases...

4.1.2 (2024-05-09)
------------------
* Fix Versions
* 4.1.1
* 4.1.0

4.1.1 (2024-04-29)
------------------
* 4.1.0

4.1.0 (2024-03-22)
------------------
* Remove deprecated args from launch, added compatibility with qbmove

4.0.2 (2024-03-08)
------------------

4.0.1 (2024-03-08)
------------------

4.0.0 (2024-03-08)
------------------
* Feature: launcher with namespace
* Added controller list get method
* first compiling Humble version
* First commit (newbranch)

3.1.0 (2023-07-26)
------------------
* Updated Submodule
* FIX: submodules updated and current/deflection control mode check
* FIX: Conflict with ros-serial
* Tested code with modification and new API

3.0.6 (2023-04-26)
------------------
* FEAT: added option to specify the serial port to connect to. This can be useful when is desired to use the device with virtual ports (e.g. UR)
* FEAT: Added compatibility with new FW version of SHR
* MINOR FIX: Changed the name of the package maintainer

3.0.5 (2022-09-07)
------------------
* MINOR FIX: Changed cmake version required in all packages (current version 3.0.2).

3.0.4 (2022-07-18)
------------------
* package compiled with qbdevice-api v1.1.3

3.0.3 (2022-07-15)
------------------
* package compiled with qbdevice-api v1.1.2

3.0.2 (2022-07-07)
------------------
* Updated qbdevice api pkg to version 1.0.2.

3.0.1 (2022-07-06)
------------------
* Updated qbdevice api pkg to version 1.0.1.

3.0.0 (2022-07-05)
------------------
* Changed internal qbdevice API with public one.
* Created service for Homing SHR2
* Created SHR2 class in qbrobotics-api
* Minor changes (comments...)
* Modified serial sub-modules: Changes done in communication_handler node
* Modified waiting time. Works for a brief period but stop after reboot.
* modified scanPortsAndDevicesFunction. New API implementation does not with kinematic chain.
* Added mutex to avoid conflicts on the same serial port. Removed old and unusefull comments.
* changes in qbrobotics-api.
* New API migration. qb_device_driver class is no longer required.
* API 7.X.X integration. Incompatibility with the wrapper script

2.3.4 (2022-04-08)
------------------

2.3.3 (2022-04-08)
------------------

2.3.2 (2022-04-08)
------------------

2.3.1 (2022-04-08)
------------------

2.3.0 (2022-04-08)
------------------

2.2.1 (2021-08-27)
------------------

2.2.0 (2021-08-27)
------------------
* Replaced error stream with throttle
* Fixed crash when unplagged device is plugged at runtime.
* Created message published in a topic by communication_handler that contains the connection status of each device.
* Implemented rescanning when device is not found
* Deactivation bug fixed
* Fixed reconnection on killing nodes.
* Add methods to retrieve actual device references
* Refactor Gazebo plugin to prepare for SoftHand simulation
* Added the control mode switch service. Added a parameter to choose to use .yaml limits or firmware ones.
* Fix init when no device is found on the given port
* Update license
* Update arg description
* Added a parameter to use the device with or without other robots. Added some motor limits when the qbmoves are used in kinematic configs.

2.1.1 (2019-10-07)
------------------

2.1.0 (2019-05-28)
------------------
* Improve inheritance for other devices
* Update documentation
* Fix minor style issues

2.0.3 (2018-08-09)
------------------
* Update license agreement copyright

2.0.2 (2018-08-07)
------------------
* Exclude dummy boards from the connected device list

2.0.1 (2018-06-01)
------------------

2.0.0 (2018-05-30)
------------------
* Move sleep at low level (next to API)
* Add method to temporarily change PID parameters
* Fix doxygen documentation
* Fix communication errors with asynchronous reads
* Refactor node registration
* Add method to get currents and positions together
* Fix minors
* Fix repetitions reliablity check
* Add a blocking setCommands method
* Fix destructor calls on ROS shutdown
* Fix minors
* Fix unexpected fault with std::unordered_set
* Add parallelization with several USB connected
* Let the user decide whether to read/write or not
* Add an alert if maximum repetitions is set to zero
* Refactor node registration
* Add a real isConnected method
* Refactor device scan method with repetitions
* Retrieve control and input mode device settings
* Implement repetitions also for getMeasurements
* Add repetitions while reading from serial
* Move error checks in ROS service callbacks

1.2.2 (2017-11-30)
------------------
* Reduce communication errors

1.1.0 (2017-11-24)
------------------

1.0.8 (2017-06-27)
------------------
* Fix C++11 support for cmake version less than 3.1

1.0.7 (2017-06-26)
------------------
* Fix minor build problems

1.0.6 (2017-06-23)
------------------
* Update cmake version to match Kinetic standards

1.0.5 (2017-06-22)
------------------

1.0.4 (2017-06-21)
------------------

1.0.3 (2017-06-21)
------------------
* fix cmake settings to solve isolated builds

1.0.2 (2017-06-20)
------------------
* remove API git submodule and add API files manually (API commit: c61204b) because ROS buildfarm does not manage git submodules

1.0.1 (2017-06-19)
------------------
* first public release for Kinetic
