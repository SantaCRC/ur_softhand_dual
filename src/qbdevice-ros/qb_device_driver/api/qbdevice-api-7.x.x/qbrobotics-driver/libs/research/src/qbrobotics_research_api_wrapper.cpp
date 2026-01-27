/***
 *  Software License Agreement: BSD 3-Clause License
 *  Copyright (c) 2015-2023, qbrobotics®
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 *  following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice, this list of conditions and the
 *    following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *  * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <qbrobotics_research_api/qbrobotics_research_api_wrapper.h>

std::shared_ptr<qbrobotics_research_api::Communication> handler_g = nullptr;

void setFakeValidHandle(comm_settings *handle) {
#ifndef _WIN32
  handle->file_handle = 1;
#else
  handle->file_handle = nullptr;
#endif
}

void setInvalidHandle(comm_settings *handle) {
#ifndef _WIN32
  handle->file_handle = -1;
#else
  handle->file_handle = INVALID_HANDLE_VALUE;
#endif
}

void openRS485(comm_settings *handle, const char *serial_port_name, int /*BAUD_RATE*/) {
  std::cerr << "\n\n\n"
               "  ------------------------------------------------------------------------------\n"
               "                   WARNING: This API wrapper is now deprecated!\n"
               "  ------------------------------------------------------------------------------\n"
               "  * All the methods of the qbrobotics C++ API v6.2.x are still usable but they\n"
               "    are now deprecated and will be no longer supported in following releases.\n"
               "  * Please update your code to the new C++ API v7.x.x which adds also many\n"
               "    improvements in efficiency and reliability.\n"
               "  * See the documentation at https://bitbucket.org/qbrobotics/qbrobotics-driver\n"
               "    for more details about the new API, and contact support@qbrobotics.com for\n"
               "    any questions or concerns.\n"
               "  ------------------------------------------------------------------------------\n\n" << std::endl;

  char serial_ports[60][255] = {};
  bool serial_port_found = false;
  int number_of_ports = RS485listPorts(serial_ports);
  for (int i=0; i<number_of_ports; i++) {
    if (std::string(serial_ports[i]) == std::string(serial_port_name)) {
      serial_port_found = true;
      if (!handler_g->openSerialPort(std::string(serial_port_name))) {
        handle->serial_port_name = std::string(serial_port_name);
        setFakeValidHandle(handle);  // fake positive handle
        return;
      }
    }
  }

  if (!serial_port_found) {  // try to open it anyway
    if (!handler_g->createSerialPort(std::string(serial_port_name))) {
      handle->serial_port_name = std::string(serial_port_name);
      setFakeValidHandle(handle);  // fake positive handle
      return;
    }
  }

  handle->serial_port_name.clear();
  setInvalidHandle(handle);
}

void closeRS485(comm_settings *handle) {
  if (!handle->serial_port_name.empty()) {
    handler_g->closeSerialPort(handle->serial_port_name);
    handle->serial_port_name.clear();
    setInvalidHandle(handle);
  }
}

void RS485GetInfo(comm_settings *handle, char *buffer) {
  std::cerr << "\n\n\n"
               "  ------------------------------------------------------------------------------\n"
               "                      WARNING: RS485GetInfo() is not safe!\n"
               "  ------------------------------------------------------------------------------\n"
               "  * This method is exactly the same as sending commGetInfo() in broadcast mode,\n"
               "    i.e. by setting the ID equals 0.\n"
               "  * If there is only one device connected on the given serial port, this method\n"
               "    is safe as it is the commGetInfo(). However, when there are several devices\n"
               "    in chain, it is not possible to distinguish the received packages from one\n"
               "    to the other, and the behavior is therefore unpredictable.\n"
               "  * Please use the similar commGetInfo() method instead and if needed list the\n"
               "    connected device IDs with RS485ListDevices() before the call.\n"
               "  ------------------------------------------------------------------------------\n\n" << std::endl;
}

int RS485ListDevices(comm_settings *handle, char list_of_ids[255]) {
  if (!handle->serial_port_name.empty()) {
    std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids;
    if (handler_g->listConnectedDevices(handle->serial_port_name, device_ids) >= 0) {
      for (size_t i=0; i<device_ids.size(); i++) {
        list_of_ids[i] = device_ids.at(i).id;
      }
      return device_ids.size();
    }
  }
  return -1;
}

int RS485listPorts(char list_of_ports[60][255]) {
  std::vector<serial::PortInfo> serial_ports;
  handler_g = std::make_shared<qbrobotics_research_api::Communication>();
  if (handler_g->listSerialPorts(serial_ports) < 0 || serial_ports.size() > 60) {
    return -1;
  }
  for (size_t i=0; i<serial_ports.size() && serial_ports.at(i).serial_port.size() < 256; i++) {
    std::strcpy(list_of_ports[i], serial_ports.at(i).serial_port.c_str());
  }
  return serial_ports.size();
}

int RS485read(comm_settings *handle, int id, char *package) {
  if (!handle->serial_port_name.empty()) {
    uint8_t device_id = 0;
    uint8_t command = 0;
    std::vector<int8_t> buffer;
    if (handler_g->readPackage(handle->serial_port_name, device_id, command, buffer) > 0 && device_id == id) {
      std::memcpy(package, buffer.data(), buffer.size());
      return buffer.size();
    }
  }
  return -1;
}

void commActivate(comm_settings *handle, int id, char activate) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    dev.setMotorStates(activate);
  }
}

int commBootloader(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.setBootloaderMode();
  }
  return -1;
}

int commCalibrate(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::qbmoveResearch qbmove(handler_g, "dev", handle->serial_port_name, id, false);
    return qbmove.computeAndStoreMaximumStiffness();
  }
  return -1;
}

int commGetAccelerations(comm_settings *handle, int id, short int *accelerations) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> accelerations_in;
    if (dev.getAccelerations(accelerations_in)) {
      return -1;
    }
    accelerations[0] = accelerations_in.at(0);
    accelerations[1] = accelerations_in.at(1);
    accelerations[2] = accelerations_in.at(2);
    return 0;
  }
  return -1;
}

int commGetActivate(comm_settings *handle, int id, char *activate) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    bool motor_state_in = false;
    if (dev.getMotorStates(motor_state_in)) {
      return -1;
    }
    *activate = motor_state_in ? 0x03 : 0x00;
    return 0;
  }
  return -1;
}

int commGetCurrents(comm_settings *handle, int id, short int *currents) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> currents_in;
    if (dev.getCurrents(currents_in)) {
      return -1;
    }
    currents[0] = currents_in.at(0);
    currents[1] = currents_in.at(1);
    return 0;
  }
  return -1;
}

int commGetCurrAndMeas(comm_settings *handle, int id, short int *values) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> currents_in;
    std::vector<int16_t> positions_in;
    if (dev.getCurrentsAndPositions(currents_in, positions_in)) {
      return -1;
    }
    values[0] = currents_in.at(0);
    values[1] = currents_in.at(1);
    values[2] = positions_in.at(0);
    values[3] = positions_in.at(1);
    values[4] = positions_in.at(2);
    return 0;
  }
  return -1;
}

int commGetEmg(comm_settings *handle, int id, short int *emgs) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::qbSoftHandResearch qbsofthand(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> emg_values_in;
    if (qbsofthand.getEMGs(emg_values_in)) {
      return -1;
    }
    emgs[0] = emg_values_in.at(0);
    emgs[1] = emg_values_in.at(1);
    return 0;
  }
  return -1;
}

int commGetInfo(comm_settings *handle, int id, short int info_type, char *info) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::string info_in;
    if (dev.getInfo(info_type, info_in)) {
      return -1;
    }
    std::strcpy(info, info_in.c_str());
    return 0;
  }
  return -1;
}

int commGetInputs(comm_settings *handle, int id, short int *inputs) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> references_in;
    if (dev.getControlReferences(references_in)) {
      return -1;
    }
    inputs[0] = references_in.at(0);
    inputs[1] = references_in.at(1);
    return 0;
  }
  return -1;
}

int commGetJoystick(comm_settings *handle, int id, short int *joystick) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::qbSoftHandResearch qbsofthand(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> joystick_values_in;
    if (qbsofthand.getJoystick(joystick_values_in)) {
      return -1;
    }
    joystick[0] = joystick_values_in.at(0);
    joystick[1] = joystick_values_in.at(1);
    return 0;
  }
  return -1;
}

int commGetMeasurements(comm_settings *handle, int id, short int *positions) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> positions_in;
    if (dev.getPositions(positions_in)) {
      return -1;
    }
    positions[0] = positions_in.at(0);
    positions[1] = positions_in.at(1);
    positions[2] = positions_in.at(2);
    return 0;
  }
  return -1;
}

int commGetParamList(comm_settings *handle, int id, unsigned short index, void *values, unsigned short value_size, unsigned short num_of_values, uint8_t *buffer) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    if(!index) {
      std::vector<int8_t> buffer_in;
      dev.getParameters(buffer_in);
      std::memcpy(buffer, buffer_in.data(), buffer_in.size());
      return 0;
    } else {
      std::vector<int8_t> data_out;
      switch (value_size) {
        case 1: {
          std::vector<uint8_t> vector({reinterpret_cast<uint8_t*>(values), reinterpret_cast<uint8_t*>(values)+num_of_values});
          data_out = qbrobotics_research_api::Communication::vectorSwapAndCast<int8_t, uint8_t>(vector);
        } break;
        case 2: {
          std::vector<uint16_t> vector({reinterpret_cast<uint16_t*>(values), reinterpret_cast<uint16_t*>(values)+num_of_values});
          data_out = qbrobotics_research_api::Communication::vectorSwapAndCast<int8_t, uint16_t>(vector);
        } break;
        case 4: {
          std::vector<uint32_t> vector({reinterpret_cast<uint32_t*>(values), reinterpret_cast<uint32_t*>(values)+num_of_values});
          data_out = qbrobotics_research_api::Communication::vectorSwapAndCast<int8_t, uint32_t>(vector);
        } break;
        default:
          return -1;
      }
      return dev.setParameter(index, data_out);
    }
  }
  return -1;
}

int commGetVelocities(comm_settings *handle, int id, short int *velocities) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    std::vector<int16_t> velocities_in;
    if (dev.getVelocities(velocities_in)) {
      return -1;
    }
    velocities[0] = velocities_in.at(0);
    velocities[1] = velocities_in.at(1);
    velocities[2] = velocities_in.at(2);
    return 0;
  }
  return -1;
}

int commHandCalibrate(comm_settings *handle, int id, short int speed, short int repetitions) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::qbSoftHandResearch qbsofthand(handler_g, "dev", handle->serial_port_name, id, false);
    return qbsofthand.openCloseCycles(speed, repetitions);
  }
  return -1;
}

int commInitMem(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.restoreFactoryDataMemory();
  }
  return -1;
}

int commPing(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.ping();
  }
  return -1;
}

int commRestoreParams(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.restoreUserDataMemory();
  }
  return -1;
}

void commSetBaudRate(comm_settings *handle, int id, short int baudrate) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    dev.setParamBaudrate(!baudrate ? 3 : 13);
  }
}

void commSetCuffInputs(comm_settings *handle, int id, int flag) {
  //TODO: this method should be ported in the new API under Cuff device class
  if (!handle->serial_port_name.empty()) {
    auto const data_out = std::vector<int8_t>(flag ? 1 : 0);
    handler_g->sendCommand(handle->serial_port_name, id, CMD_SET_CUFF_INPUTS, data_out);
  }
}

void commSetInputs(comm_settings *handle, int id, short int *inputs) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    dev.setControlReferences({inputs[0], inputs[1]});
  }
}

int commSetInputsAck(comm_settings *handle, int id, short int *inputs) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    dev.setControlReferences({inputs[0], inputs[1]});
    return 0;
  } else {
    return -1;
  }
}

void commSetPosStiff(comm_settings *handle, int id, short int *inputs) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::qbmoveResearch qbmove(handler_g, "dev", handle->serial_port_name, id, false);
    qbmove.setPositionAndStiffnessReferences(inputs[0], inputs[1]);
  }
}

void commSetWatchDog(comm_settings *handle, int id, short int wdt) {
  std::cerr << "\n\n\n"
               "  ------------------------------------------------------------------------------\n"
               "                WARNING: commSetWatchDog() is already deprecated!\n"
               "  ------------------------------------------------------------------------------\n"
               "  * This method has not been ported to the new API and therefore it will be no\n"
               "    longer supported at all in future releases.\n"
               "  * Please contact support@qbrobotics.com for any questions, concerns or feature\n"
               "    requests.\n"
               "  ------------------------------------------------------------------------------\n\n" << std::endl;

  if (!handle->serial_port_name.empty()) {
    auto const data_out = std::vector<int8_t>({static_cast<int8_t>(wdt/2)});
    handler_g->sendCommand(handle->serial_port_name, id, CMD_SET_WATCHDOG, data_out);
  }
}

int commSetZeros(comm_settings *handle, int id, void *values, unsigned short num_of_values) {
  std::cerr << "\n\n\n"
               "  ------------------------------------------------------------------------------\n"
               "                   WARNING: commSetZeros() differs from v6.2.x!\n"
               "  ------------------------------------------------------------------------------\n"
               "  * This method uses now an internal computation and does no longer use `values`"
               "    and `num_of_values`.\n"
               "  * Please contact support@qbrobotics.com for any questions, concerns or feature\n"
               "    requests.\n"
               "  ------------------------------------------------------------------------------\n\n" << std::endl;

  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.setParamZeros();
  }
  return -1;
}

int commStoreParams(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.storeUserDataMemory();
  }
  return -1;
}

int commStoreDefaultParams(comm_settings *handle, int id) {
  if (!handle->serial_port_name.empty()) {
    qbrobotics_research_api::Device dev(handler_g, "dev", handle->serial_port_name, id, false);
    return dev.storeFactoryDataMemory();
  }
  return -1;
}
