/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2015-2023, qbrobotics®
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef QBROBOTICS_RESEARCH_API_WRAPPER_H
#define QBROBOTICS_RESEARCH_API_WRAPPER_H

// standard libraries
#include <iostream>
#include <cstring>
#ifdef _WIN32
#include <windows.h>
#endif
// project libraries
#include <qbrobotics_research_api/qbrobotics_research_api.h>
#include <qbrobotics_research_api/qbsofthand_research_api.h>
#include <qbrobotics_research_api/qbmove_research_api.h>

typedef struct comm_settings {
#ifndef _WIN32
  int file_handle = -1;
  #define INVALID_HANDLE_VALUE    -1
#else
  HANDLE file_handle = INVALID_HANDLE_VALUE;
#endif
  std::string serial_port_name;
} comm_settings;

/***
 *  Open the given serial port with common settings for qbrobotics devices.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param serial_port_name The serial port path.
 *  \param BAUD_RATE This old parameter is no longer used. The baud rate is always set to 2M.
 */
void openRS485(comm_settings *handle, const char *serial_port_name, int /*BAUD_RATE*/ = 2000000);

/***
 *  Close the given serial port.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 */
void closeRS485(comm_settings *handle);

/***
 *  \sa RS485GetInfo()
 */
void RS485GetInfo(comm_settings *handle, char *buffer);

/***
 *  List the device IDs connected to the given serial port.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param list_of_ids An array of bytes filled with the connected device IDs.
 *  \return The number of devices connected.
 */
int RS485ListDevices(comm_settings *handle, char list_of_ids[255]);

/***
 *  List the available serial ports (a maximum of 60 ports can be found).
 *  \param list_of_ports An array of C strings filled with the serial ports paths, e.g. "/dev/ttyUSB0".
 *  \return The number of serial ports found on success, or \p -1 otherwise.
 */
int RS485listPorts(char list_of_ports[60][255]);

/***
 *  Read a package from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param package A buffer of bytes where the package is stored.
 *  \return The package length on success, \p -1 otherwise.
 */
int RS485read(comm_settings *handle, int id, char *package);

/***
 *  Activates or deactivates the motor/s of the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param activate \p true to turn motors on, \p false to turn them off.
 */
void commActivate(comm_settings *handle, int id, char activate);

/***
 *  Send the given device board in bootloader mode in order to update its firmware.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commBootloader(comm_settings *handle, int id);

/***
 *  Calibrate the maximum stiffness value of the given device (the value is stored on the device memory).
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commCalibrate(comm_settings *handle, int id);

/***
 *  Get encoder acceleration measurements from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param accelerations Acceleration measurements relative to the device encoders.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetAccelerations(comm_settings *handle, int id, short int *accelerations);

/***
 *  Get the activation status of the motor/s of the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param activate \p true if motors are on, \p false otherwise.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetActivate(comm_settings *handle, int id, char *activate);

/***
 *  Get motor current measurements from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param currents Current measurements relative to the device motors.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetCurrents(comm_settings *handle, int id, short int *currents);

/***
 *  Get motor currents and encoder position measurements from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param values Motor current and encoder position measurements vector (currents are in first two places).
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetCurrAndMeas(comm_settings *handle, int id, short int *values);

/***
 *  Get measurements from ElectroMyoGraphic sensors connected to the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param emgs EMG sensors measurements.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetEmg(comm_settings *handle, int id, short int *emgs);

/***
 *  Get internal information about the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param info_type The type of information to be retrieved.
 *  \param info A buffer that stores the parameter values in a readable format (the minimum buffer size depends on the
 *  given device; a value of at least 2000 characters is advisable).
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetInfo(comm_settings *handle, int id, short int info_type, char *info);

/***
 *  Get the reference commands from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param inputs The reference command vector which is parsed w.r.t. the actual control mode.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetInputs(comm_settings *handle, int id, short int *inputs);

/***
 *  Get joystick measurements from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param joystick Joystick analog measurements.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetJoystick(comm_settings *handle, int id, short int *joystick);

/***
 *  Get encoder position measurements from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param positions Position measurements relative to the device encoders.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetMeasurements(comm_settings *handle, int id, short int *positions);

/***
 *  Get all the parameters that are stored in the given device memory (\p index must be \p 0 and \p buffer not \p null),
 *  or set one of them if specified (\p index must be positive and \p values not \p null).
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param index The parameter index w.r.t. the device memory map (unfortunately this is not always specified); use \p 0
 *  to retrieve all the parameters in \p buffer, and positive values to set a single parameter with the given \p values.
 *  \param values An array with the parameter values to be set.
 *  \param value_size The byte size of the parameter to be set, e.g. 2 for uint16_t, 4 for float, ...
 *  \param num_of_values The parameter values array size
 *  \param buffer A buffer of bytes where the parameters are stored. The new v7.x.x C++ API provides a helpful utility
 *  method to parse a given parameter from the buffer (\sa Device::Params::getParameter).
 */
int commGetParamList(comm_settings *handle, int id, unsigned short index, void *values, unsigned short value_size, unsigned short num_of_values, uint8_t *buffer);

/***
 *  Get encoder speed measurements from the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param velocities Velocity measurements relative to the device encoders.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commGetVelocities(comm_settings *handle, int id, short int *velocities);

/***
 *  Start a series of full opening and full closure cycles on the given device. The cycles can also be stopped by
 *  passing both the \p speed and the \p repetitions equal \p -1.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param speed The speed of openings and closures in range [0, 200].
 *  \param repetitions The number of cycles (max value is 32767).
 */
int commHandCalibrate(comm_settings *handle, int id, short int speed, short int repetitions);
/***
 *  Initialize the given device memory with the default factory parameters hardcoded in the firmware.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 */
int commInitMem(comm_settings *handle, int id);

/***
 *  Ping the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \return \p 0 on success, \p -1 otherwise.
 */
int commPing(comm_settings *handle, int id);

/***
 *  Restore the parameter values from the memory archive to the given device memory.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 */
int commRestoreParams(comm_settings *handle, int id);

/***
 *  Set the device communication baud rate.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param baudrate \p 0 to set 2Mbaud, \p 1 to set 460.8kbaud (actually only 2M is currently supported).
 */
void commSetBaudRate(comm_settings *handle, int id, short int baudrate);

/***
 *  Send reference inputs to the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param flag If nonzero activates the Cuff driving functionality of the board.
 */
void commSetCuffInputs(comm_settings *handle, int id, int flag);

/***
 *  Send reference commands to the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param inputs The reference command vector which is parsed w.r.t. the actual control mode (the references can either
 *  be positions or currents, or whatever is used to drive the device).
 */
void commSetInputs(comm_settings *handle, int id, short int *inputs);

int commSetInputsAck(comm_settings *handle, int id, short int *inputs);

/***
 *  Send shaft position and stiffness preset reference commands to the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param inputs The reference command vector where the first element is the shaft position and the second one is the
 *  stiffness preset. Both values are expressed in motor ticks.
 */
void commSetPosStiff(comm_settings *handle, int id, short int *inputs);

/***
 *  Set the watchdog timer of the given device.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param wdt The watchdog timer in range [0, 500]csec (\p 0 means disabled).
 */
void commSetWatchDog(comm_settings *handle, int id, short int wdt);

/***
 *  Set the actual positions as the zero encoder position values which remain stored the given device memory.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 *  \param value This old parameter is no longer used.
 *  \param num_of_values This old parameter is no longer used.
 */
int commSetZeros(comm_settings *handle, int id, void *values, unsigned short num_of_values);

/***
 *  Store all the actual parameters in the given device memory; these values will be loaded from memory at every startup.
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 */
int commStoreParams(comm_settings *handle, int id);

/***
 *  Store all the actual parameters in the given device memory archive; these values will not be loaded from memory at
 *  every startup, but the can be restored by calling \p commRestoreParams().
 *  \param handle A \p comm_settings structure containing info about the communication settings.
 *  \param id The device ID.
 */
int commStoreDefaultParams(comm_settings *handle, int id);

#endif  // QBROBOTICS_RESEARCH_API_WRAPPER_H
