# qbdevice API

C++ API to control qbrobotics devices (SoftHand Industry, SoftHand Research, SoftHand2 Research and qbmove)

## Get this project
In order to clone this project and all its submodules, execute the following command:
```
git clone --recurse-submodules https://bitbucket.org/qbrobotics/qbdevice-api-7.x.x.git
```

## INSTALLATION

### Research devices
In order to compile the libraries (static or dynamic) to be used in external projects, it is necessary to build both __Serial__ and __qbrobotics-driver__ projects.

Execute the following commands, in order to compile the project:
```
mkdir build
cd build
cmake ..
make
```
Now the **build** folder should contain:
- **qbrobotics-driver-internal** folder, with libqbrobotics_driver.a or libqbrobotics_driver.so library;
- **serial** folder, with libSerial.a or libSerial.so library.

Use them to exploit qbrobotics API in external projects.

---
**NOTE**

In order to compile **STATIC** or **DYNAMIC** libraries just move to each of CMakeLists.txt files contained inside **qbrobotics-driver-internal** and **serial** folders and edit the **add_library** line in that file.

---

## GEt THE LATEST UPDATES
Since the project is composed of git submodules, to get the latest updates run the command:

```
git submodule update --init 
```