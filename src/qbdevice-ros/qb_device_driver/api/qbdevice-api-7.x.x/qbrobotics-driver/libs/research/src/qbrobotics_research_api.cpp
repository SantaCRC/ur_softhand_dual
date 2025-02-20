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

// standard libraries
#include <algorithm>
#include <sstream>  // for macOS
// project libraries
#include <qbrobotics_research_api/qbrobotics_research_api.h>

using namespace qbrobotics_research_api;

Communication::Communication()
    : Communication(2000000) {}

Communication::Communication(uint32_t baud_rate)
    : Communication(baud_rate, serial::Serial::Timeout(200)) {}  // could be serial::Serial::Timeout(20) for new devices but legacy require a higher timeout(even 300 for SHR2)

Communication::Communication(const serial::Serial::Timeout &timeout)
    : Communication(2000000, timeout) {}

Communication::Communication(uint32_t baud_rate, const serial::Serial::Timeout &timeout)
    : serial_ports_baud_rate_(baud_rate),
      serial_ports_timeout_(timeout) {}

Communication::Communication(const Communication &communication)
    : Communication(communication, communication.serial_ports_timeout_) {}

Communication::Communication(const Communication &communication, const serial::Serial::Timeout &timeout)
    : connected_devices_(communication.connected_devices_),
      serial_ports_info_(communication.serial_ports_info_),
      serial_ports_(communication.serial_ports_),
      serial_ports_baud_rate_(communication.serial_ports_baud_rate_),
      serial_ports_timeout_(timeout) {}

int Communication::listSerialPorts(std::vector<serial::PortInfo> &serial_ports_info) {
  serial_ports_info.clear();

  std::vector<serial::PortInfo> connected_serial_ports;
  if (serial::getPortsInfo(connected_serial_ports) < 0) {
    return -1;
  }

  // add new entries to private members
  for (auto const &serial_port : connected_serial_ports) {
    if (serial_port.manufacturer == "QB Robotics" || serial_port.id_vendor == 0x403) {
      serial_ports_info.push_back(serial_port);
      if (!isInSerialPortsInfo(serial_port.serial_port)) {
        if (isInSerialPorts(serial_port.serial_port)) {
          return -1;  // this should never happen
        }
        serial_ports_info_[serial_port.serial_port] = serial_port;
        serial_ports_[serial_port.serial_port] = std::make_shared<serial::Serial>();  // this does not open the serial port
      }
    }
  }

  // remove no longer existing entries in private members
  for (auto serial_port_it = serial_ports_info_.begin(); serial_port_it != serial_ports_info_.end();) {
    if (std::find_if(connected_serial_ports.begin(), connected_serial_ports.end(),
        [&](auto item){ return serial_port_it->first == item.serial_port; }) == connected_serial_ports.end()) {
      serial_ports_.erase(serial_port_it->first);
      serial_port_it = serial_ports_info_.erase(serial_port_it);
    } else {
      serial_port_it++;  // only when items are not erased
    }
  }

  return serial_ports_info.size();
}

int Communication::listConnectedDevices() {
  connected_devices_.clear();
  int connected_devices = 0;
  for (auto const &serial_port : serial_ports_) {
    std::vector<ConnectedDeviceInfo> device_ids;
    if (listConnectedDevices(serial_port.first, device_ids) > 0) {
      connected_devices_[serial_port.first] = device_ids;
      connected_devices += device_ids.size();
    }
  }
  return connected_devices;
}

int Communication::listConnectedDevices(const std::string &serial_port_name, std::vector<ConnectedDeviceInfo> &device_ids) {
  device_ids.clear();

  serial::Serial::Timeout short_timeout(serial::Serial::Timeout(5));
  if (!openSerialPort(serial_port_name, serial_ports_baud_rate_, short_timeout)) {
    serial_ports_.at(serial_port_name)->setTimeout(short_timeout);  // if already connected it might have a different timeout
    std::vector<int8_t> data_in;

    std::vector<uint8_t> max_id({255, 50, 10, 10, 10});  // max ID checked during i-th scan
    // Smart device scan: more retries for lower IDs (more likely)
    //   1-10:  very likely   -> 5x10 scan
    //  11-50:  unlikely      -> 2x40 scan
    // 51-255:  very unlikely -> 1x205 scan
    // = 335 pings
    for (size_t scan = 0; scan < max_id.size(); scan++) {
      for (uint16_t device_id = 1; device_id <= max_id.at(scan); device_id++) {  // should be uint8_t, but uint8_t(256) == 0 and it never ends
        // skip device_id if already found
        if (std::find_if(device_ids.begin(), device_ids.end(), [&device_id](const auto& info){ return info.id == device_id; }) != device_ids.end()) {
          continue;
        }
        // sendCommandAndParse with 0 repetitions. Actual repetitions are decided by the for loops
        if (sendCommandAndParse(serial_port_name, device_id, CMD_PING, 0, data_in) >= 0) {
          device_ids.push_back({static_cast<uint8_t>(device_id)});
          if (data_in.size() > 0) {  // the 4-byte S/N is stored in the ping payload (only for v7.1+ firmware)
            std::stringstream str;
            str << std::setfill('0') << std::setw(10) << Communication::vectorCastAndSwap<uint32_t>(data_in).front();
            std::string serial_number(str.str());
            device_ids.back().serial_number = serial_number;
            device_ids.back().type = std::string(serial_number.begin(), serial_number.begin()+3);
            device_ids.back().sub_type = std::string(serial_number.begin()+3, serial_number.begin()+6);
          }
        }
      }
    }
    std::sort(device_ids.begin(), device_ids.end(), [](const auto& a, const auto& b){ return a.id > b.id; });
  }
  serial_ports_.at(serial_port_name)->setTimeout(serial_ports_timeout_);

  return device_ids.size();
}

int Communication::closeSerialPort(const std::string &serial_port_name) {
  if (!isInSerialPorts(serial_port_name)) {
    return -1;
  }

  try {
    serial_ports_.at(serial_port_name)->close();
  } catch (...) {
    return -1;
  }

  return 0;
}

int Communication::createSerialPort(const std::string &serial_port_name) {
  return createSerialPort(serial_port_name, serial_ports_baud_rate_);
}

int Communication::createSerialPort(const std::string &serial_port_name, uint32_t baud_rate) {
  return createSerialPort(serial_port_name, baud_rate, serial_ports_timeout_);
}

int Communication::createSerialPort(const std::string &serial_port_name, const serial::Serial::Timeout &timeout) {
  return createSerialPort(serial_port_name, serial_ports_baud_rate_, timeout);
}

int Communication::createSerialPort(const std::string &serial_port_name, uint32_t baud_rate, const serial::Serial::Timeout &timeout) {
  if (isInSerialPorts(serial_port_name)) {
    return -1;
  }

  try {
    serial_ports_[serial_port_name] = std::make_shared<serial::Serial>(serial_port_name, baud_rate, timeout);
  } catch (...) {
    return -1;
  }
  return 0;
}

int Communication::openSerialPort(const std::string &serial_port_name) {
  return openSerialPort(serial_port_name, serial_ports_baud_rate_);
}

int Communication::openSerialPort(const std::string &serial_port_name, uint32_t baud_rate) {
  return openSerialPort(serial_port_name, baud_rate, serial_ports_timeout_);
}

int Communication::openSerialPort(const std::string &serial_port_name, serial::Serial::Timeout &timeout) {
  return openSerialPort(serial_port_name, serial_ports_baud_rate_, timeout);
}

int Communication::openSerialPort(const std::string &serial_port_name, uint32_t baud_rate, serial::Serial::Timeout &timeout) {
  if (!isInSerialPorts(serial_port_name)) {
    return -1;
  }
  if (serial_ports_.at(serial_port_name)->isOpen()) {
    return 0;
  }

  try {
    if (serial_ports_.at(serial_port_name)->getPort().empty()) {
      serial_ports_.at(serial_port_name)->setPort(serial_port_name);
    }
    serial_ports_.at(serial_port_name)->setBaudrate(baud_rate);
    serial_ports_.at(serial_port_name)->setTimeout(timeout);
    serial_ports_.at(serial_port_name)->open();
  } catch (...) {
    return -1;
  }
  return 0;
}

int Communication::parsePackage(const std::string &serial_port_name, uint8_t device_id, uint8_t command) {
  std::vector<int8_t> data_in;
  return parsePackage(serial_port_name, device_id, command, data_in);
}

int Communication::parsePackage(const std::string &serial_port_name, uint8_t device_id, uint8_t command, std::vector<int8_t> &data_in) {
  data_in.clear();
  uint8_t device_id_in = 0;
  uint8_t command_in = 0;
  std::vector<int8_t> data_in_buffer;

  // works only for non-broadcast commands
  if (!device_id) {
    return -1;
  }

  while (readPackage(serial_port_name, device_id_in, command_in, data_in_buffer) > 0) {
    //WARN: this is ugly because it can lead to unwanted command parsing on CMD_INIT_MEM, CMD_RESTORE_PARAMS or CMD_GET_PARAM_LIST (especially this that is used for many things!)
    if (device_id != device_id_in && (command != CMD_INIT_MEM && command != CMD_RESTORE_PARAMS && (command != CMD_GET_PARAM_LIST || data_in_buffer.size() != 1))) {  // id and id_in can differ on CMD_INIT_MEM or CMD_RESTORE_PARAMS (and also with CMD_GET_PARAM_LIST when changing the device ID)
      continue;
    }
    // ACK is checked from the caller that knows whether the command has ACK or not
    if (command_in == command) {
      data_in = data_in_buffer;
      if (device_id != device_id_in) {
        data_in.push_back(device_id_in);  // ugly fix to pass the new id to the caller (data_in_buffer is only the ACK response in these cases)
      }
      return data_in.size();
    }
  }
  return -1;
}

int Communication::sendCommand(const std::string &serial_port_name, uint8_t device_id, uint8_t command) {
  std::vector<int8_t> data_out;
  return sendCommand(serial_port_name, device_id, command, data_out);
}

int Communication::sendCommand(const std::string &serial_port_name, uint8_t device_id, uint8_t command, const std::vector<int8_t> &data_out) {
  if (openSerialPort(serial_port_name)) {
    return -1;
  }
  if (writePackage(serial_port_name, device_id, command, data_out) < 0) {
    return -1;
  }
  return 0;
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command) {
  return sendCommandAndParse(serial_port_name, device_id, command, 2);  // 3 attempts by default (1 + 2 optional retrials)
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, uint8_t max_repeats) {
  std::vector<int8_t> data_in, data_out;
  return sendCommandAndParse(serial_port_name, device_id, command, max_repeats, data_out, data_in);
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, std::vector<int8_t> &data_in) {
  return sendCommandAndParse(serial_port_name, device_id, command, 2, data_in);  // 3 attempts by default (1 + 2 optional retrials)
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, uint8_t max_repeats, std::vector<int8_t> &data_in) {
  std::vector<int8_t> data_out;
  return sendCommandAndParse(serial_port_name, device_id, command, max_repeats, data_out, data_in);
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, const std::vector<int8_t> &data_out) {
  return sendCommandAndParse(serial_port_name, device_id, command, 2, data_out);  // 3 attempts by default (1 + 2 optional retrials)
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, uint8_t max_repeats, const std::vector<int8_t> &data_out) {
  std::vector<int8_t> data_in;
  return sendCommandAndParse(serial_port_name, device_id, command, max_repeats, data_out, data_in);
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, const std::vector<int8_t> &data_out, std::vector<int8_t> &data_in) {
  return sendCommandAndParse(serial_port_name, device_id, command, 2, data_out, data_in);  // 3 attempts by default (1 + 2 optional retrials)
}

int Communication::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, uint8_t max_repeats, const std::vector<int8_t> &data_out, std::vector<int8_t> &data_in) {
  uint8_t repeats = 0;
  data_in.clear();

  while (repeats <= max_repeats) {
    if (sendCommand(serial_port_name, device_id, command, data_out) < 0) {
      repeats++;
      continue;
    }

    if (command == CMD_STORE_PARAMS || command == CMD_STORE_DEFAULT_PARAMS) {  // writing to EEPROM takes a while
      std::this_thread::sleep_for(std::chrono::milliseconds(600));
    }

    std::vector<int8_t> packages_in;
    if (parsePackage(serial_port_name, device_id, command, packages_in) >= 0) {
      data_in = packages_in;
      break;
    }

    repeats++;
  }

  return repeats <= max_repeats ? data_in.size() : -1;
}

int Communication::sendCommandBroadcast(const std::string &serial_port_name, uint8_t command) {
  std::vector<int8_t> data_out;
  return sendCommand(serial_port_name, 0, command, data_out);
  //WARN: broadcast should be used only for non-returning methods (it is unreliable for returning methods)
}

int Communication::sendCommandBroadcast(const std::string &serial_port_name, uint8_t command, const std::vector<int8_t> &data_out) {
  return sendCommand(serial_port_name, 0, command, data_out);
  //WARN: broadcast should be used only for non-returning methods (it is unreliable for returning methods)
}

int Communication::deserializePackage(const std::vector<uint8_t> &package_in, uint8_t &device_id, uint8_t &command) {
  std::vector<int8_t> data;
  return deserializePackage(package_in, device_id, command, data);
}

int Communication::deserializePackage(const std::vector<uint8_t> &package_in, uint8_t &device_id, uint8_t &command, std::vector<int8_t> &data) {
  if (package_in.size() < 6) {
    return -1;
  }
  if (package_in.at(0) != ':' || package_in.at(1) != ':') {
    return -1;
  }
  if (package_in.at(2) == 0) {
    return -1;
  }
  if (package_in.at(4) == CMD_GET_INFO || package_in.at(4) == CMD_GET_PARAM_LIST) {  // special case for very long packets that exceed 8bit length
    if (package_in.at(3) != package_in.size()-4 && (static_cast<uint32_t>(package_in.at(3)) << 5) != package_in.size()-4) {
      return -1;
    }
  } else {  // all common commands
    if (package_in.at(3) != package_in.size()-4) {
      return -1;
    }
  }
  if (package_in.back() != checksum(package_in, package_in.size()-1)) {
    return -1;
  }
  device_id = package_in.at(2);
  command = package_in.at(4);
  data.clear();
  data.insert(std::end(data), std::begin(package_in)+5, std::end(package_in)-1);

  return package_in.size();
}

int Communication::readPackage(const std::string &serial_port_name, uint8_t &device_id, uint8_t &command) {
  std::vector<uint8_t> package_in;
  if (readPackage(serial_port_name, package_in) < 0) {
    return -1;
  }
  return deserializePackage(package_in, device_id, command);
}

int Communication::readPackage(const std::string &serial_port_name, uint8_t &device_id, uint8_t &command, std::vector<int8_t> &data) {
  std::vector<uint8_t> package_in;
  if (readPackage(serial_port_name, package_in) < 0) {
    return -1;
  }
  return deserializePackage(package_in, device_id, command, data);
}

int Communication::readPackage(const std::string &serial_port_name, std::vector<uint8_t> &package_in) {
  package_in.clear();
  try {
    serial_ports_.at(serial_port_name)->read(package_in, 4);
    if (package_in.size() != 4) {
      serial_ports_.at(serial_port_name)->flush();
      return -1;
    }
    if (package_in.at(0) != ':' || package_in.at(1) != ':') {
      serial_ports_.at(serial_port_name)->flush();
      return -1;
    }
    serial_ports_.at(serial_port_name)->read(package_in, package_in.at(3));

    // special case for very long packets that exceed 8bit length
    if (package_in.size() > 3 && (package_in.at(4) == CMD_GET_INFO || (package_in.at(4) == CMD_GET_PARAM_LIST && package_in.size() > 7))) {
      int special_packet_length = package_in.at(3) << 5;  // multiple of 32
      serial_ports_.at(serial_port_name)->read(package_in, special_packet_length - package_in.at(3));  // already read package_in.at(3) bytes
    }
  } catch (...) {
    return -1;
  }
  return package_in.size();
}

int Communication::serializePackage(uint8_t device_id, uint8_t command, std::vector<uint8_t> &package_out) {
  std::vector<int8_t> data;
  return serializePackage(device_id, command, data, package_out);
}

int Communication::serializePackage(uint8_t device_id, uint8_t command, const std::vector<int8_t> &data, std::vector<uint8_t> &package_out) {
  package_out.clear();
  package_out.push_back(':');
  package_out.push_back(':');
  package_out.push_back(device_id);
  package_out.push_back(2 + data.size());
  package_out.push_back(command);
  package_out.insert(std::end(package_out), std::begin(data), std::end(data));
  package_out.push_back(checksum(package_out, package_out.size()));
  return package_out.size();
}

int Communication::writePackage(const std::string &serial_port_name, uint8_t device_id, uint8_t command) {
  std::vector<uint8_t> package_out;
  serializePackage(device_id, command, package_out);
  return writePackage(serial_port_name, package_out);
}

int Communication::writePackage(const std::string &serial_port_name, uint8_t device_id, uint8_t command, const std::vector<int8_t> &data) {
  std::vector<uint8_t> package_out;
  serializePackage(device_id, command, data, package_out);
  return writePackage(serial_port_name, package_out);
}

int Communication::writePackage(const std::string &serial_port_name, const std::vector<uint8_t> &package_out) {
  try {
    if (serial_ports_.at(serial_port_name)->write(package_out) != package_out.size()) {
      return -1;
    }
  } catch (...) {
    return -1;
  }
  return package_out.size();
}

uint8_t Communication::checksum(const std::vector<uint8_t> &data, uint32_t size) {
  uint8_t checksum = 0x00;
  if (data.size() >= size) {
    for (uint32_t i=4; i<size; i++) {  // exclude first 4 bytes for backward compatibility
      checksum = checksum ^ data.at(i);
    }
  }
  return checksum;
}

bool Communication::isInSerialPorts(const std::string &serial_port_name) {
  return serial_ports_.count(serial_port_name);
}

bool Communication::isInSerialPortsInfo(const std::string &serial_port_name) {
  return serial_ports_info_.count(serial_port_name);
}

CommunicationLegacy::CommunicationLegacy(const Communication &communication)
    : Communication(communication) {}

CommunicationLegacy::CommunicationLegacy(const Communication &communication, const serial::Serial::Timeout &timeout)
    : Communication(communication, timeout) {}

int CommunicationLegacy::parsePackage(const std::string &serial_port_name, uint8_t device_id, uint8_t command, std::vector<int8_t> &data_in) {
  data_in.clear();
  uint8_t device_id_in = 0;
  uint8_t command_in = 0;
  std::vector<int8_t> data_in_buffer;

  // works only for non-broadcast commands
  if (!device_id) {
    return -1;
  }

  while (readPackage(serial_port_name, device_id_in, command_in, data_in_buffer) > 0) {
    //WARN: this is ugly because it can lead to unwanted command parsing on CMD_INIT_MEM, CMD_RESTORE_PARAMS or CMD_GET_PARAM_LIST (especially this that is used for many things!)
    if (device_id != device_id_in && (command != CMD_INIT_MEM && command != CMD_RESTORE_PARAMS && (command != CMD_GET_PARAM_LIST || data_in_buffer.size() != 1))) {  // id and id_in can differ on CMD_INIT_MEM or CMD_RESTORE_PARAMS (and also with CMD_GET_PARAM_LIST when changing the device ID)
      continue;
    }
    //WARN: ACK messages do not carry the command info on legacy devices
    if (command_in == command || (command_in == ACK_OK && (command == CMD_INIT_MEM || command == CMD_STORE_PARAMS || command == CMD_STORE_DEFAULT_PARAMS || command == CMD_RESTORE_PARAMS || command == CMD_BOOTLOADER || command == CMD_CALIBRATE || command == CMD_HAND_CALIBRATE))) {
      data_in = data_in_buffer;
      if (device_id != device_id_in) {
        data_in.push_back(device_id_in);  // ugly fix to pass the new id to the caller (data_in_buffer is only the ACK response in these cases)
      }
      return data_in.size();
    }
  }
  return -1;
}

int CommunicationLegacy::sendCommandAndParse(const std::string &serial_port_name, uint8_t device_id, uint8_t command, uint8_t max_repeats, const std::vector<int8_t> &data_out, std::vector<int8_t> &data_in) {
  uint8_t repeats = 0;
  data_in.clear();

  while (repeats <= max_repeats) {
    if (sendCommand(serial_port_name, device_id, command, data_out) < 0) {
      repeats++;
      continue;
    }

    if (command == CMD_STORE_PARAMS || command == CMD_STORE_DEFAULT_PARAMS) {  // writing to EEPROM takes a while
      std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 100ms should be enough for legacy firmware which write less bytes in EEPROM
    }

    // CMD_GET_PARAM_LIST and CMD_GET_INFO packages size exceeds uint8 max value and require a special parsing method
    if ((command == CMD_GET_PARAM_LIST && data_out.size() == 2) || command == CMD_GET_INFO) {
      std::vector<uint8_t> package_in;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (readLongPackage(serial_port_name, package_in) > 0) {
        if (command == CMD_GET_PARAM_LIST) {  // CMD_GET_INFO is pure ASCII package without packet info (there is no need to trim it)
          package_in.erase(package_in.begin(), package_in.begin()+5);  // 5 bytes that need to be filtered out (:|:|<ID>|<LEN>|<CMD>)
          package_in.erase(package_in.end()-1);  // one checksum byte that need to be filtered out
        }
        data_in = std::vector<int8_t>(std::begin(package_in), std::end(package_in));
        break;
      }
    } else if (command == CMD_GET_PARAM_LIST) {  // set param did non have ACK on legacy devices...
      data_in.push_back(1);  // fake ACK because legacy devices do not provide ACK on set param
      break;
    } else {  // all other commands
      std::vector<int8_t> packages_in;
      if (parsePackage(serial_port_name, device_id, command, packages_in) >= 0) {
        data_in = packages_in;
        break;
      }
    }

    repeats++;
  }

  return repeats <= max_repeats ? data_in.size() : -1;
}

int CommunicationLegacy::deserializePackage(const std::vector<uint8_t> &package_in, uint8_t &device_id, uint8_t &command, std::vector<int8_t> &data) {
  if (package_in.size() < 6) {
    return -1;
  }
  if (package_in.at(0) != ':' || package_in.at(1) != ':') {
    return -1;
  }
  if (package_in.at(2) == 0) {
    return -1;
  }
  if (package_in.at(4) != CMD_GET_INFO && (package_in.at(4) != CMD_GET_PARAM_LIST || package_in.size() <= 7)) {  // cannot check size for very long packets that exceed 8bit length
    if (package_in.at(3) != package_in.size()-4) {
      return -1;
    }
  }
  if (package_in.back() != checksum(package_in, package_in.size()-1)) {
    return -1;
  }
  device_id = package_in.at(2);
  command = package_in.at(4);
  data.clear();
  data.insert(std::end(data), std::begin(package_in)+5, std::end(package_in)-1);

  return package_in.size();
}

int CommunicationLegacy::readLongPackage(const std::string &serial_port_name, std::vector<uint8_t> &package_in) {
  package_in.clear();
  try {
    serial_ports_.at(serial_port_name)->read(package_in, 10000);
  } catch (...) {
    return -1;
  }
  return package_in.size();
}

int CommunicationLegacy::readPackage(const std::string &serial_port_name, uint8_t &device_id, uint8_t &command, std::vector<int8_t> &data) {
  std::vector<uint8_t> package_in;
  if (readPackage(serial_port_name, package_in) < 0) {
    return -1;
  }
  return deserializePackage(package_in, device_id, command, data);
}

int CommunicationLegacy::readPackage(const std::string &serial_port_name, std::vector<uint8_t> &package_in) {
  package_in.clear();
  try {
    serial_ports_.at(serial_port_name)->read(package_in, 4);
    if (package_in.size() != 4) {
      serial_ports_.at(serial_port_name)->flush();
      return -1;
    }
    if (package_in.at(0) != ':' || package_in.at(1) != ':') {
      serial_ports_.at(serial_port_name)->flush();
      return -1;
    }
    serial_ports_.at(serial_port_name)->read(package_in, package_in.at(3));
  } catch (...) {
    return -1;
  }
  return package_in.size();
}

Device::Device(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, true, std::make_unique<Params>()) {}

Device::Device(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, true, std::make_unique<Params>()) {}

Device::Device(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Params> params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, true, std::move(params)) {}

Device::Device(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, bool check_param, std::unique_ptr<Params> params)
    : name_(std::move(name)),
      serial_port_(std::move(serial_port)),
      params_(std::move(params)),
      communication_(std::move(communication)) {
  params_->id = id;
  if (init_params) {
    std::vector<int8_t> param_buffer;
    bool getParamSuccess = !getParameters(id, param_buffer);
    if (!getParamSuccess && check_param) {
      throw std::runtime_error("failure during getParameters()");
    }
    if (getParamSuccess){
      params_->initParams(param_buffer);
      if (params_->id != id) {
        throw std::runtime_error("failure during initParams()");
      }
    }
  }
}

int Device::ping() {
  return communication_->sendCommandAndParse(serial_port_, params_->id, CMD_PING);
}

int Device::getControlReferences(std::vector<int16_t> &control_references) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_INPUTS, data_in) < 0) {
    return -1;
  }
  control_references = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int Device::getCurrents(std::vector<int16_t> &currents) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_CURRENTS, data_in) < 0) {
    return -1;
  }
  currents = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int Device::getCurrentsAndPositions(std::vector<int16_t> &currents,
                                                           std::vector<int16_t> &positions) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_CURR_AND_MEAS, data_in) < 0) {
    return -1;
  }
  auto currents_and_positions = Communication::vectorCastAndSwap<int16_t>(data_in);
  currents = std::vector<int16_t>(currents_and_positions.begin(), currents_and_positions.begin()+2);
  positions = std::vector<int16_t>(currents_and_positions.begin()+2, currents_and_positions.end());
  return 0;
}

int Device::getPositions(std::vector<int16_t> &positions) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_MEASUREMENTS, data_in) < 0) {
    return -1;
  }
  positions = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int Device::getVelocities(std::vector<int16_t> &velocities) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_VELOCITIES, data_in) < 0) {
    return -1;
  }
  velocities = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int Device::getAccelerations(std::vector<int16_t> &accelerations) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_ACCEL, data_in) < 0) {
    return -1;
  }
  accelerations = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int Device::getMotorStates(bool &motor_state) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_ACTIVATE, data_in) < 0) {
    return -1;
  }
  if (data_in.size() != 1 || (data_in.front() != 0x03 && data_in.front() != 0x00)) {
    return -1;
  }
  motor_state = data_in.front() == 0x03;
  return 0;
}

int Device::getCycleTime(int16_t &cycle_time) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_CYCLE_TIME, data_in) < 0) {
    return -1;
  }
  cycle_time = Communication::vectorCastAndSwap<int16_t>(data_in).at(1);
  return 0;
}

int Device::setControlReferences(const std::vector<int16_t> &control_references) {
  auto const data_out = Communication::vectorSwapAndCast<int8_t>(control_references);
  if (communication_->sendCommand(serial_port_, params_->id, CMD_SET_INPUTS, data_out) < 0) {
    return -1;
  }
  return 0;
}

int Device::setControlReferencesAndWait(const std::vector<int16_t> &control_references) {
  auto const data_out = Communication::vectorSwapAndCast<int8_t>(control_references);
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_SET_INPUTS_ACK, data_out, data_in) < 0) {
    return -1;
  }
  if (data_in.size() < 0) {
    return data_in.size();
  }
  return 0;
}

int Device::setMotorStates(bool motor_state) {
  uint8_t data_out = motor_state ? 0x03 : 0x00;
  if (communication_->sendCommand(serial_port_, params_->id, CMD_ACTIVATE, std::vector<int8_t>(1, data_out)) < 0) {
    return -1;
  }
  return 0;
}

int Device::getInfo(std::string &info) {
  return getInfo(0, info);
}

int Device::getInfo(uint16_t info_type, std::string &info) {
  std::vector<int8_t> data_in;
  const std::vector<int8_t> data_out = Communication::vectorSwapAndCast<int8_t, uint16_t>({info_type});
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_INFO, data_out, data_in) < 0) {
    return -1;
  }
  info.assign(data_in.begin(), data_in.end());
  return 0;
}

int Device::getParameters(std::vector<int8_t> &param_buffer) {
  return getParameters(params_->id, param_buffer);
}

int Device::getParameters(uint8_t id, std::vector<int8_t> &param_buffer) {
  param_buffer.clear();
  const std::vector<int8_t> data_out = Communication::vectorSwapAndCast<int8_t, uint16_t>({0x0000});
  if (communication_->sendCommandAndParse(serial_port_, id, CMD_GET_PARAM_LIST, data_out, param_buffer) < 0) {
    return -1;
  }
  return 0;
}

int Device::getParamId() {
  return getParamId(params_->id);
}

int Device::getParamId(uint8_t &id) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(1, param_buffer, id);
  return 0;
}

int Device::getParamPositionPID() {
  return getParamPositionPID(params_->position_pid);
}

int Device::getParamPositionPID(std::vector<float> &position_pid) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<float>(2, param_buffer, position_pid);
  return 0;
}

int Device::getParamCurrentPID() {
  return getParamCurrentPID(params_->current_pid);
}

int Device::getParamCurrentPID(std::vector<float> &current_pid) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<float>(3, param_buffer, current_pid);
  return 0;
}

int Device::getParamStartupActivation() {
  return getParamStartupActivation(params_->startup_activation);
}

int Device::getParamStartupActivation(uint8_t &startup_activation) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(4, param_buffer, startup_activation);
  return 0;
}

int Device::getParamInputMode() {
  return getParamInputMode(params_->input_mode);
}

int Device::getParamInputMode(uint8_t &input_mode) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(5, param_buffer, input_mode);
  return 0;
}

int Device::getParamControlMode() {
  return getParamControlMode(params_->control_mode);
}

int Device::getParamControlMode(uint8_t &control_mode) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(6, param_buffer, control_mode);
  return 0;
}

int Device::getParamEncoderResolutions() {
  return getParamEncoderResolutions(params_->encoder_resolutions);
}

int Device::getParamEncoderResolutions(std::vector<uint8_t> &encoder_resolutions) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(7, param_buffer, encoder_resolutions);
  return 0;
}

int Device::getParamEncoderOffsets() {
  return getParamEncoderOffsets(params_->encoder_offsets);
}

int Device::getParamEncoderOffsets(std::vector<int16_t> &encoder_offsets) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<int16_t>(8, param_buffer, encoder_offsets);
  return 0;
}

int Device::getParamEncoderMultipliers() {
  return getParamEncoderMultipliers(params_->encoder_multipliers);
}

int Device::getParamEncoderMultipliers(std::vector<float> &encoder_multipliers) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<float>(9, param_buffer, encoder_multipliers);
  return 0;
}

int Device::getParamUsePositionLimits() {
  return getParamUsePositionLimits(params_->use_position_limits);
}

int Device::getParamUsePositionLimits(uint8_t &use_position_limits) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(10, param_buffer, use_position_limits);
  return 0;
}

int Device::getParamPositionLimits() {
  return getParamPositionLimits(params_->position_limits);
}

int Device::getParamPositionLimits(std::vector<int32_t> &position_limits) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<int32_t>(11, param_buffer, position_limits);
  return 0;
}

int Device::getParamPositionMaxSteps() {
  return getParamPositionMaxSteps(params_->position_max_steps);
}

int Device::getParamPositionMaxSteps(std::vector<int32_t> &position_max_steps) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<int32_t>(12, param_buffer, position_max_steps);
  return 0;
}

int Device::getParamCurrentLimit() {
  return getParamCurrentLimit(params_->current_limit);
}

int Device::getParamCurrentLimit(int16_t &current_limit) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<int16_t>(13, param_buffer, current_limit);
  return 0;
}

int Device::setParameter(uint16_t param_type, const std::vector<int8_t> &param_data) {
  bool motor_state = true;
  if (getMotorStates(motor_state) || motor_state) {
    return -1;  // motor must be stopped while changing parameters
  }

  std::vector<int8_t> data_in;
  std::vector<int8_t> param_payload = Communication::vectorSwapAndCast<int8_t, uint16_t>({param_type});
  param_payload.insert(param_payload.end(), param_data.begin(), param_data.end());
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_PARAM_LIST, param_payload, data_in) < 0) {
    return -1;
  }
  if (data_in.size() > 0 && data_in.front() != 1) {  // check ACK
    return -2;
  }
  return 0;
}

int Device::setParamId(uint8_t id) {
  int set_fail = setParameter(1, Communication::vectorSwapAndCast<int8_t, uint8_t>({id}));
  if (!set_fail) {
    params_->id = id;
  }
  return set_fail;
}

int Device::setParamPositionPID(const std::vector<float> &position_pid) {
  int set_fail = setParameter(2, Communication::vectorSwapAndCast<int8_t, float>(position_pid));
  if (!set_fail) {
    params_->position_pid = position_pid;
  }
  return set_fail;
}

int Device::setParamCurrentPID(const std::vector<float> &current_pid) {
  int set_fail = setParameter(3, Communication::vectorSwapAndCast<int8_t, float>(current_pid));
  if (!set_fail) {
    params_->current_pid = current_pid;
  }
  return set_fail;
}

int Device::setParamStartupActivation(bool startup_activation) {
  int set_fail = setParameter(4, Communication::vectorSwapAndCast<int8_t, uint8_t>({startup_activation}));
  if (!set_fail) {
    params_->startup_activation = startup_activation;
  }
  return set_fail;
}

int Device::setParamInputMode(uint8_t input_mode) {
  int set_fail = setParameter(5, Communication::vectorSwapAndCast<int8_t, uint8_t>({input_mode}));
  if (!set_fail) {
    params_->input_mode = input_mode;
  }
  return set_fail;
}

int Device::setParamControlMode(uint8_t control_mode) {
  int set_fail = setParameter(6, Communication::vectorSwapAndCast<int8_t, uint8_t>({control_mode}));
  if (!set_fail) {
    params_->control_mode = control_mode;
  }
  return set_fail;
}

int Device::setParamEncoderResolutions(const std::vector<uint8_t> &encoder_resolutions) {
  int set_fail = setParameter(7, Communication::vectorSwapAndCast<int8_t, uint8_t>(encoder_resolutions));
  if (!set_fail) {
    params_->encoder_resolutions = encoder_resolutions;
  }
  return set_fail;
}

int Device::setParamEncoderOffsets(const std::vector<int16_t> &encoder_offsets) {
  int set_fail = setParameter(8, Communication::vectorSwapAndCast<int8_t, int16_t>(encoder_offsets));
  if (!set_fail) {
    params_->encoder_offsets = encoder_offsets;
  }
  return set_fail;
}

int Device::setParamEncoderMultipliers(const std::vector<float> &encoder_multipliers) {
  int set_fail = setParameter(9, Communication::vectorSwapAndCast<int8_t, float>(encoder_multipliers));
  if (!set_fail) {
    params_->encoder_multipliers = encoder_multipliers;
  }
  return set_fail;
}

int Device::setParamUsePositionLimits(bool use_position_limits) {
  int set_fail = setParameter(10, Communication::vectorSwapAndCast<int8_t, uint8_t>({use_position_limits}));
  if (!set_fail) {
    params_->use_position_limits = use_position_limits;
  }
  return set_fail;
}

int Device::setParamPositionLimits(const std::vector<int32_t> &position_limits) {
  int set_fail = setParameter(11, Communication::vectorSwapAndCast<int8_t, int32_t>(position_limits));
  if (!set_fail) {
    params_->position_limits = position_limits;
  }
  return set_fail;
}

int Device::setParamPositionMaxSteps(const std::vector<int32_t> &position_max_steps) {
  int set_fail = setParameter(12, Communication::vectorSwapAndCast<int8_t, int32_t>(position_max_steps));
  if (!set_fail) {
    params_->position_max_steps = position_max_steps;
  }
  return set_fail;
}

int Device::setParamCurrentLimit(int16_t current_limit) {
  int set_fail = setParameter(13, Communication::vectorSwapAndCast<int8_t, int16_t>({current_limit}));
  if (!set_fail) {
    params_->current_limit = current_limit;
  }
  return set_fail;
}

int Device::setParamZeros() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_SET_ZEROS, data_in) < 0) {
    return -1;
  }
  return getParamEncoderOffsets();  // update intenal params_->encoder_offsets values
}

int Device::setParamSerialNumber(const uint32_t &serial_number) {
  return -1;
}

int Device::setParamBaudrate(uint8_t prescaler_divider) {
  const std::vector<int8_t> data_out = Communication::vectorSwapAndCast<int8_t, uint8_t>({prescaler_divider});
  if (communication_->sendCommand(serial_port_, params_->id, CMD_SET_BAUDRATE, data_out) < 0) {
    return -1;
  }
  return 0;
}

void Device::Params::initParams(const std::vector<int8_t> &param_buffer) {
  //FIXME: hardcoded input mode parameter id
  getParameter<uint8_t>(1, param_buffer, id);
  getParameter<float>(2, param_buffer, position_pid);
  getParameter<float>(3, param_buffer, current_pid);
  getParameter<uint8_t>(4, param_buffer, startup_activation);
  getParameter<uint8_t>(5, param_buffer, input_mode);
  getParameter<uint8_t>(6, param_buffer, control_mode);
  getParameter<uint8_t>(7, param_buffer, encoder_resolutions);
  getParameter<int16_t>(8, param_buffer, encoder_offsets);
  getParameter<float>(9, param_buffer, encoder_multipliers);
  getParameter<uint8_t>(10, param_buffer, use_position_limits);
  getParameter<int32_t>(11, param_buffer, position_limits);
  getParameter<int32_t>(12, param_buffer, position_max_steps);
  getParameter<int16_t>(13, param_buffer, current_limit);
}

int Device::restoreFactoryDataMemory() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_INIT_MEM, data_in) < 0) {
    return -1;
  }
  if (data_in.size() > 0 && data_in.front() != 1) {  // check ACK
    return -2;
  }
  if (data_in.size() > 0) {  // if data_in is empty the ID is the same
    params_->id = data_in.back();  // the package contains the ACK + the new device id
  }
  return 0;
}

int Device::restoreUserDataMemory() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_RESTORE_PARAMS, data_in) < 0) {
    return -1;
  }
  if (data_in.size() > 0 && data_in.front() != 1) {  // check ACK
    return -2;
  }
  if (data_in.size() > 0) {  // if data_in is empty the ID is the same
    params_->id = data_in.back();  // the package contains the ACK + the new device id
  }
  return 0;
}

int Device::storeUserDataMemory() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_STORE_PARAMS, data_in) < 0) {
    return -1;
  }
  if (data_in.size() > 0 && data_in.front() != 1) {  // check ACK
    return -2;
  }
  return 0;
}

int Device::storeFactoryDataMemory() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_STORE_DEFAULT_PARAMS, data_in) < 0) {
    return -1;
  }
  if (data_in.size() > 0 && data_in.front() != 1) {  // check ACK
    return -2;
  }
  return 0;
}

int Device::setBootloaderMode() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_BOOTLOADER, data_in) < 0) {
    return -1;
  }
  if (data_in.size() > 0 && data_in.front() != 1) {  // check ACK
    return -2;
  }
  return 0;
}
