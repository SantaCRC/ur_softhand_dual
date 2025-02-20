^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qbrobotics_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

7.2.8 (2023-09-06)
------------------
* FEAT: API SHIN are now compatible with FW 1.1.0. Libs on windows can be compiled using MinGW.

7.2.7 (2023-09-05)
------------------
* Add position-current control mode in setPositionAndStifness

7.2.6 (2023-08-30)
------------------
* FEAT: Added function to industry lib.

7.2.5 (2023-07-24)
------------------
* FIX: updated README.md file

7.2.4 (2023-07-24)
------------------
* MINOR FIX: updated licenses

7.2.3 (2023-07-24)
------------------
* FIX: Removed method to set serial number from qb SoftClaw class.
* FIX: cstring included (compiling error in newest linux distro)

7.2.2 (2023-07-20)
------------------
* Fix package version
* FIX: fixed constructor for Device class

7.2.1 (2023-07-20)
------------------
* FIX version 7.2.1
* MINOR FIX: SC serial number can be set with qbmove method.

7.2.0 (2023-07-20)
------------------
* FIX version 7.2.0
* FEAT Added specific subclass of qbmoveResearch for SoftClaw
* FIX: fixed changlog date.

7.1.3 (2023-07-20)
------------------
* FIX: updated pkg version(7.1.3)
* FEAT: Added NMMI modification requests
* FIX: changed type in some interna variables. Error on Windows occurs.
* FEAT Added functions to retrieve and set serial number param.
* FIX: less timeout in communication constructor.
* FIX: missed semicolon in a statement.
* FEAT: Implemented a method to retried SoftClaw device
* FEAT: Device classified from claw serial number
* FEAT: added a line with information about max current threshold in getStatisticsExtended method.
* FEAT: added a method to retrieve the maximum current threshold. Changed the function to check the minimum (ELMO) FW version.
* FEAT: Added functions to get additional parameters (e.g. S/N)
* FIX: changed type in get functions. Errors occur when getting some parameters (e.g. S/N)

7.1.2 (2023-07-20)
------------------
* Bug fixed with installation directory

7.1.1 (2023-07-20)
------------------
* Increased timeout to improve device scanning on Windows PCs
* Add warning in the description of setParamHandSide
* Removed space line in qbrobotics_research_api.cpp

7.1.0 (2023-07-20)
------------------
* Implemented set/get hand side parameter for SH2R

7.0.2 (2023-07-20)
------------------
* Updated pkg version
* Bug fixed in homing function.
* corrected description error in the file README.md. Updated package version
* corrected description error in the file README.md
* Modification of some function descriptions
* Added the multipliative synergies functions
* Modified CMakeLists.txt file after the serial folder name change
* Version updated and removed qbrobotics_driver.h file
* Removed unusefull code parts
* Modified homing function
* Modified SHR2 class in order to set parameters properly (6.X.X fw)
* Modified Timeout value (communication) to solve communication errors for SHR2
* Generated doxygen project for API documentation
* Added homing function for SHR2
* Added class to control qbSoftHand 2 and added some function descriptions.
* Added some function descriptions.
* Added some comments to methods. Implemented method setControlReferencesAndWait().
* Added a method in wrapper and modified cmakelists.txt file
* Bug fix on hand calibration
* Added compatibility to read cycle timer from PSoC.
* SHIN: Uncommented a code portion causing compilation error on VS2019
* Industry and research devices libs compiled separately.
* modified README file
* Added README.md file to compile research devices libs. Edited CMakeLists.txt file.
* Test enabling with flag in CMakeLists.txt. Added link to Serialinclude dir
* Comments in .pro for compilation.
* Added qmake project to compile libqbapi. Updated API
* Change setParamZeros() for new firmware versions
* Flush stream when getting unexpected data
* Add specific methods to get/set single parameters
* Refactor tests
* Fix store/restore ID which is a bit nasty
* Fix CMD_HAND_CALIBRATE package data
* Fix wrapper on Windows
* Split API wrapper and fix few bugs
* Fix serial timeouts namespace (required after Serial update)
* Sort methods alphabetically
* Update documentation
* Add interface wrapper for old API v6.2.x
* Add qb SoftHand and qbmove specific methods
* Fix get/set parameters and parse/send command methods
* Add methods to get single parameters
* Add methods to set parameters
* Add derived classes for specific devices
* Add routines to get Params from devices
* Add tests for basic routines and swap bytes
* Implement Device Class methods
* Add vector cast and swap bytes methods
* Implement Communication class and low level routines
* Refactor CMakeLists and link Serial library
* Add Google Testing Framework and a dummy test
* Refactor interface for the communication subclass
* Refactor research library with classes
* Merge NMMI centropiaggio branch fixes
* Fix old API formatting
