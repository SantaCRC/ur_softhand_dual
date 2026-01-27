^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qb_device_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* First SHR2M compatible version
* First SHR compatible version
* modified launch logic
* first compiling Humble version

3.1.0 (2023-07-26)
------------------

3.0.6 (2023-04-26)
------------------
* FIX: gazebo entity name setted as device_name (not device type). This caused errors when devices gazebo nodes start
* MINOR FIX: Changed the name of the package maintainer

3.0.5 (2022-09-07)
------------------
* MINOR FIX: Changed cmake version required in all packages (current version 3.0.2).
* Bug fixed when executing the waypoint movement and different controller with different number of controller joints are loaded.
* Changed device controller bringup to load properly the device controller.

3.0.4 (2022-07-18)
------------------

3.0.3 (2022-07-15)
------------------

3.0.2 (2022-07-07)
------------------

3.0.1 (2022-07-06)
------------------

3.0.0 (2022-07-05)
------------------
* Added loading motors control for SH2R

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
* BUG FIX when a generic qbrobotics device roslaunch is grouped in a namespace

2.2.1 (2021-08-27)
------------------

2.2.0 (2021-08-27)
------------------
* Fixed reconnection on killing nodes.
* Refactor control structures initialization
* Add Gazebo SoftHand and SoftHand 2 Motors plugin skeletons
* Refactor Gazebo plugin to prepare for SoftHand simulation
* Add hardware interface for SoftHand 2 Motors
* Set a better name for fake joint simulation variables
* Added the control mode switch service. Added a parameter to choose to use .yaml limits or firmware ones.
* Add Gazebo support for qbmove devices
* Fix hardcoded params in launch files since ros_comm#1354 has been backported to kinetic
* Fix 'pass_all_args' workaround since ros_comm#889 has been backported to kinetic
* Added compatibility to qb SoftClaw in .launch files
* Update arg description
* Added a parameter to use the device with or without other robots. Added some motor limits when the qbmoves are used in kinematic configs.

2.1.1 (2019-10-07)
------------------

2.1.0 (2019-05-28)
------------------
* Improve inheritance for other devices
* Update documentation
* Add a simulator mode to debug joint trajectories
* Add specific joint limits for the delta
* Add interactive markers startup activation
* Fix set/reset use_waypoints at each launch

2.0.3 (2018-08-09)
------------------

2.0.2 (2018-08-07)
------------------

2.0.1 (2018-06-01)
------------------

2.0.0 (2018-05-30)
------------------
* Refactor launch files
* Fix minors
* Add a blocking setCommands method
* Fix until ros_comm#889 is fixed
* Refactor launch files and fix controllers spawner

1.2.2 (2017-11-30)
------------------
* Fix launch file

1.1.0 (2017-11-24)
------------------
* Fix control nodes type

1.0.8 (2017-06-27)
------------------

1.0.7 (2017-06-26)
------------------

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

1.0.1 (2017-06-19)
------------------
* first public release for Kinetic
