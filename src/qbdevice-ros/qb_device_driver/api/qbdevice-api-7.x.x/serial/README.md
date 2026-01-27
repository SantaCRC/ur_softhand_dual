# Serial library
## Disclaimer

This repository is a fork of [William Woodall's](https://github.com/wjwwood/serial) and [Alessandro Tondo's](https://github.com/alextoind/serial) serial, who we really thank (together with all the other contributors) for the great job done so far.

This project can be used as a support tool for the qbrobotics device API (qbdevice-driver-research).

## Installation

### Ubuntu Linux
In order to compile the project executes the following commands in the path containing this README file:
```
mkdir build
cd build
cmake ..
make
```
Now the **build** folder should contain
**serial** folder, with libSerial.a or libSerial.so library

### Windows 10
This project requires:

- Microsoft Visual Studio 2019
- Windows SDK 10.0.19041
- cmake tools 
- Developer Command Prompt fo VS 2019

Open the **Developer Command Prompt for VS 2019** and navigate to the folder containing this README file, then executes the following commands:

```
mkdir build
cd build
cmake ..
msbuild Serial.sln
```
The Serial.sln file can be opened and edited with Microsoft Visual Studio 2019.

---
**NOTE**

In order to compile **STATIC** or **DYNAMIC** libraries just move to CMakeLists.txt file and edit the **add_library** line in that file.

---
