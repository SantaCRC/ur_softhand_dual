/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2016-2024, qbrobotics®
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

#include <qb_device_driver/qb_device_communication_handler.h>

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

using namespace qb_device_communication_handler;

qbDeviceCommunicationHandler::qbDeviceCommunicationHandler(rclcpp::Node::SharedPtr& node): qbDeviceCommunicationHandler(true, node){}

qbDeviceCommunicationHandler::qbDeviceCommunicationHandler(bool scan, rclcpp::Node::SharedPtr& node)
    : node_handle_(node),
      activate_motors_(node_handle_->create_service<qb_device_msgs::srv::Trigger>("/communication_handler/activate_motors", BIND_CLS_CB(&qbDeviceCommunicationHandler::activateCallback))),
      deactivate_motors_(node_handle_->create_service<qb_device_msgs::srv::Trigger>("/communication_handler/deactivate_motors", BIND_CLS_CB(&qbDeviceCommunicationHandler::deactivateCallback))),
      get_info_(node_handle_->create_service<qb_device_msgs::srv::Trigger>("/communication_handler/get_info", BIND_CLS_CB(&qbDeviceCommunicationHandler::getInfoCallback))),
      get_measurements_(node_handle_->create_service<qb_device_msgs::srv::GetMeasurements>("/communication_handler/get_measurements", BIND_CLS_CB(&qbDeviceCommunicationHandler::getMeasurementsCallback))),
      initialize_device_(node_handle_->create_service<qb_device_msgs::srv::InitializeDevice>("/communication_handler/initialize_device", BIND_CLS_CB(&qbDeviceCommunicationHandler::initializeCallback))),
      set_commands_(node_handle_->create_service<qb_device_msgs::srv::SetCommands>("/communication_handler/set_commands", BIND_CLS_CB(&qbDeviceCommunicationHandler::setCommandsCallback))),
      set_pid_(node_handle_->create_service<qb_device_msgs::srv::SetPid>("/communication_handler/set_pid", BIND_CLS_CB(&qbDeviceCommunicationHandler::setPIDCallback))),
      set_control_mode_(node_handle_->create_service<qb_device_msgs::srv::SetControlMode>("/communication_handler/set_control_mode", BIND_CLS_CB(&qbDeviceCommunicationHandler::setControlModeCallback))),
      go_to_home_(node_handle_->create_service<qb_device_msgs::srv::Trigger>("/communication_handler/go_to_home", BIND_CLS_CB(&qbDeviceCommunicationHandler::goToHomeCallback))),
      connection_state_publisher_(node_handle_->create_publisher<qb_device_msgs::msg::ConnectionState>("/communication_handler/connection_state",1)),
      communication_handler_(std::make_shared<qbrobotics_research_api::Communication>()),
      communication_handler_legacy_(std::make_shared<qbrobotics_research_api::CommunicationLegacy>(*communication_handler_)){ // make shared pointer that handles the communication {
  if (scan){  
    while (!getSerialPortsAndDevices(3)) {
      RCLCPP_WARN_STREAM(node_handle_->get_logger(), "[CommunicationHandler] is waiting for devices...");
      rclcpp::sleep_for((std::chrono::seconds)(1)); //1s
    }
    checkActivation();
  }
}

qbDeviceCommunicationHandler::~qbDeviceCommunicationHandler() {
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] Attempt to terminate the process properly...");
  for (auto const port:serial_ports_) {
    close(port.serial_port);
  }
}

int qbDeviceCommunicationHandler::activate(const int &id, const bool &command, const int &max_repeats) {
  std::string command_prefix = command ? "" : "de";
  bool status = false;
  int failures = 0;

  failures = isActive(id, max_repeats, status);
  if (status != command) {
    devices_.at(id)->setMotorStates(command);
    rclcpp::sleep_for((std::chrono::nanoseconds)(500000000)); // 0.5s wait for motors to be active
    failures = std::max(failures, isActive(id, max_repeats, status));
    if (status != command) {
      rclcpp::Clock& clock = *node_handle_->get_clock();
      RCLCPP_ERROR_STREAM_THROTTLE(node_handle_->get_logger(),clock,60, "[CommunicationHandler] device [" << id << "] fails on " << command_prefix << "activation.");
      return -1;
    }
    RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] device [" << id << "] motors have been " << command_prefix << "activated!");
    return failures;
  }
  RCLCPP_DEBUG_STREAM(node_handle_->get_logger(), "[CommunicationHandler] device [" << id << "] motors were already " << command_prefix << "activated!");
  return failures;
}

int qbDeviceCommunicationHandler::activate(const int &id, const int &max_repeats) {
  return activate(id, true, max_repeats);
}

bool qbDeviceCommunicationHandler::activateCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->message = "Device [" + std::to_string(request->id) + "] is not connected.";
    response->success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));

  response->message = "Device [" + std::to_string(request->id) + "] activation.";
  response->failures = activate(request->id, request->max_repeats);
  response->success = isReliable(response->failures, request->max_repeats);
  return true;
}

void qbDeviceCommunicationHandler::checkActivation() {
  check_connection_status_timer_ = node_handle_->create_wall_timer(std::chrono::seconds(1),std::bind(&qbDeviceCommunicationHandler::checkConnectionAndPublish, this));
}

int qbDeviceCommunicationHandler::close(const std::string &serial_port) {
  for (auto const &device : connected_devices_) {
    if (device.second == serial_port) {
      deactivate(device.first, 3);
    }
  }
  connected_devices_.clear();
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] # of connected devices " << connected_devices_.size());
  communication_handler_->closeSerialPort(serial_port);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] does not handle [" << serial_port << "] anymore.");
  return 0;
}

int qbDeviceCommunicationHandler::deactivate(const int &id, const int &max_repeats) {
  return activate(id, false, max_repeats);
}

bool qbDeviceCommunicationHandler::deactivateCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->message = "Device [" + std::to_string(request->id) + "] is not connected.";
    response->success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));

  response->message = "Device [" + std::to_string(request->id) + "] deactivation.";
  response->failures = deactivate(request->id, request->max_repeats);
  response->success = isReliable(response->failures, request->max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::getCurrents(const int &id, const int &max_repeats, std::vector<short int> &currents) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  currents.resize(2);  // required by 'getCurrents()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->getCurrents(currents) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::getSerialPortsAndDevices(const int &max_repeats) {
  // clear devices and serial vectors/maps
  serial_ports_.clear();
  device_ids_.clear();
  devices_.clear();
  std::string serial_port_name;
  if(node_handle_->get_parameter("/qb_device_communication_handler/serial_port_name", serial_port_name)){ // If 'serial_port_name' param is specified, then try to connect to that port
    if (!communication_handler_->createSerialPort(serial_port_name)) {
      serial::PortInfo port;
      port.serial_port = serial_port_name;
      serial_ports_.push_back(port);
    } else {
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "[CommunicationHandler] Cannot open " << serial_port_name);
      return -1;
    }
  } else {
    if (communication_handler_->listSerialPorts(serial_ports_) <= 0) {
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "[CommunicationHandler] No serial ports found!");
      return -1;
    }
  }
  int devices_retrieved;
  rclcpp::sleep_for((std::chrono::seconds)(1)); //1s
  for(auto &serial_port:serial_ports_){ // scan and open all the serial port
    try {
      devices_retrieved = communication_handler_->listConnectedDevices(serial_port.serial_port, device_ids_);
    } catch(serial::SerialIOException &/*exc_name*/) {
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "[CommunicationHandler] No qbrobotics device(s) connected!");
      return -1;
    }
    serial_protectors_.insert(std::make_pair(serial_port.serial_port, std::make_unique<std::mutex>()));  // never override
    if (devices_retrieved > 0) { // retrieved at least a qbrobotics device
      RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] Found "<< devices_retrieved << " qbrobotics devices on [" << serial_port.serial_port << "]");
      for(auto &device_id:device_ids_) {
        if (device_id.id == 120 || device_id.id == 0) {
          continue;  // ID 120 is reserved for dummy board which should not be considered as a connected device (ID 0 is for sure an error)
        }
        int failures = 0;
        while (failures <= max_repeats) {
          try { // TODO: differentiate the devices
            if (device_id.type == "001" || (device_id.type == "006" && device_id.sub_type == "100")) {  // device S/N can be retireved only from new device firmware. They can use communication_handler_
              devices_.insert(std::make_pair(static_cast<int>(device_id.id), std::make_shared<qbrobotics_research_api::Device>(communication_handler_, "dev", serial_port.serial_port, device_id.id)));
              connected_devices_.insert(std::make_pair(static_cast<int>(device_id.id), serial_port.serial_port));
              RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] Connected to device with id: "<< (int)device_id.id);
              break;
            } else {
              communication_handler_legacy_ = std::make_shared<qbrobotics_research_api::CommunicationLegacy>(*communication_handler_);
              devices_.insert(std::make_pair(static_cast<int>(device_id.id), std::make_shared<qbrobotics_research_api::Device>(communication_handler_legacy_, "dev", serial_port.serial_port, device_id.id)));
              connected_devices_.insert(std::make_pair(static_cast<int>(device_id.id), serial_port.serial_port));
              RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] The device with id " << (int)device_id.id << " is connected.");
              break;
            }
          } catch(...) {
            rclcpp::sleep_for((std::chrono::nanoseconds)(100000000)); //0.1s
            failures++;
          }
        }
        if(failures > max_repeats){
          RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] cannot connect to qbrobotics device(s) after "<< max_repeats << " attempt(s)");
          close(serial_port.serial_port);
          return -1;
        }
      }
    } else {
      RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] no qbrobotics devices found on [" << serial_port.serial_port << "]");
      close(serial_port.serial_port);
      return -1;
    }
  }
  return devices_retrieved;
}

int qbDeviceCommunicationHandler::getInfo(const int &id, const int &max_repeats, std::string &info) {
  int failures = 0;
  while (failures <= max_repeats) {
    if (devices_.at(id)->getInfo(INFO_ALL, info) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::getInfoCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->message = "Device [" + std::to_string(request->id) + "] is not connected.";
    response->success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));

  response->failures = getInfo(request->id, request->max_repeats, response->message);  // blocks while reading
  response->success = isReliable(response->failures, request->max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::getCommands(const int &id, const int &max_repeats, std::vector<short int> &commands) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  commands.resize(2);  // required by 'getCurrents()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->getControlReferences(commands) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::getMeasurements(const int &id, const int &max_repeats, std::vector<short int> &currents, std::vector<short int> &positions) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  currents.resize(2);
  positions.resize(3);
  std::vector<short int> measurements(5, 0);  // required by 'getMeasurements()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->getCurrentsAndPositions(currents, positions) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::getMeasurementsCallback(const std::shared_ptr<qb_device_msgs::srv::GetMeasurements::Request> request,  std::shared_ptr<qb_device_msgs::srv::GetMeasurements::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));
  const clock_t begin_time = clock();
  response->failures = 0;  // need to return true even if both 'get_currents' and 'get_positions' are set to false
  if (request->get_currents && request->get_positions) {
    response->failures += getCurrents(request->id, request->max_repeats, response->currents)
                        + getPositions(request->id, request->max_repeats, response->positions);  // both block while reading 
  }
  else if (request->get_currents) {
    response->failures += getCurrents(request->id, request->max_repeats, response->currents);  // blocks while reading
  }
  else if (request->get_positions) {
    response->failures += getPositions(request->id, request->max_repeats, response->positions);  // blocks while reading
  }

  if (request->get_commands) {
    response->failures += getCommands(request->id, request->max_repeats, response->commands);  // blocks while reading
  }
  rclcpp::Clock& clock = *node_handle_->get_clock();
  response->stamp = clock.now();
  response->success = isReliable(response->failures, request->max_repeats);
  rclcpp::sleep_for((std::chrono::nanoseconds)(100000)); //0.0001s
  //std::cout << "time getMeasurementsCallback of " << request->id << ": "<< float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  return true;
}

int qbDeviceCommunicationHandler::getParameters(const int &id, std::vector<int32_t> &limits, std::vector<uint8_t> &resolutions) {
  uint8_t control_mode;
  std::vector<int8_t> param_buffer;
  devices_.at(id)->getParameters(param_buffer);
  rclcpp::sleep_for((std::chrono::nanoseconds)(1000000)); //0.001s
  devices_.at(id)->getParams()->getParameter<uint8_t>(6, param_buffer, control_mode);
  devices_.at(id)->getParams()->getParameter<int32_t>(11, param_buffer, limits);
  devices_.at(id)->getParams()->getParameter<uint8_t>(7, param_buffer, resolutions);
  if (control_mode == 0 // posizione
  || control_mode == 4  // deflessione (qbmove)
  || control_mode == 3 // posizione/corrente
  || control_mode == 5) {  // deflessione/corrente
    return 0;
  }
  return -1;
}

int qbDeviceCommunicationHandler::getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions) {
  int failures = 0;
  positions.resize(3);  // required by 'getPositions()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->getPositions(positions) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

int qbDeviceCommunicationHandler::isActive(const int &id, const int &max_repeats, bool &status) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  status = false;
  while (failures <= max_repeats) {
    if (devices_.at(id)->getMotorStates(status) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}


int qbDeviceCommunicationHandler::isConnected(const int &id, const int &max_repeats) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    bool status;
    if (devices_.at(id)->getMotorStates(status) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::isInConnectedSet(const int &id) {
  return connected_devices_.count(id);
}

bool qbDeviceCommunicationHandler::initializeCallback(const std::shared_ptr<qb_device_msgs::srv::InitializeDevice::Request> request,  std::shared_ptr<qb_device_msgs::srv::InitializeDevice::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  std::vector<std::unique_lock<std::mutex>> serial_locks;  // need to lock on all the serial resources to scan for new ports/devices
  for (auto const &mutex : serial_protectors_) {
    serial_locks.push_back(std::unique_lock<std::mutex>(*mutex.second));
  }

  if (request->rescan || !isInConnectedSet(request->id)) {
    // update connected devices
    RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] Rescanning...");
    check_connection_status_timer_->cancel();
    getSerialPortsAndDevices(request->max_repeats);
    check_connection_status_timer_->reset();
  }
  if (!isInConnectedSet(request->id) || !isReliable(isConnected(request->id, request->max_repeats), request->max_repeats)) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "[CommunicationHandler] fails while initializing device [" << request->id << "] because it is not connected.");
    response->message = "Device [" + std::to_string(request->id) + "] initialization fails because it is not connected.";
    response->success = false;
    return true;
  }

  if (getParameters(request->id, response->info.position_limits, response->info.encoder_resolutions)) {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "[CommunicationHandler] fails while initializing device [" << request->id << "] because it requires 'USB' input mode and 'Position' control mode.");
    response->message = "Device [" + std::to_string(request->id) + "] initialization fails because it requires 'USB' input mode and 'Position' control mode.";
    response->success = false;
    return true;
  }
  if (request->activate) {
    response->failures = activate(request->id, request->max_repeats);
    response->success = isReliable(response->failures, request->max_repeats);
    if (!response->success) {
      RCLCPP_ERROR_STREAM(node_handle_->get_logger(), "[CommunicationHandler] has not initialized device [" << request->id << "] because it cannot activate its motors (please, check the motor positions).");
      response->message = "Device [" + std::to_string(request->id) + "] initialization fails because it cannot activate the device (please, check the motor positions).";
      return true;
    }
  }
  response->info.id = request->id;
  response->info.serial_port = connected_devices_.at(request->id);
  RCLCPP_INFO_STREAM(node_handle_->get_logger(), "[CommunicationHandler] has initialized device [" << request->id << "].");
  response->message = "Device [" + std::to_string(request->id) + "] initialization succeeds.";
  response->success = true;
  return true;
}

void qbDeviceCommunicationHandler::checkConnectionAndPublish() {
  qb_device_msgs::msg::DeviceConnectionInfo device_info_;
  bool status = false;
  int failures = 0;
  const int max_repeats = 3;
  connection_state_msg_.devices.clear();

  for(auto devices : connected_devices_) {
    std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(devices.first)));
    failures = isActive(devices.first, max_repeats, status);
    if (failures >= max_repeats) { // too much consecutive failues
      device_info_.is_active = false;
    } else {
      device_info_.is_active = status;
    }
    device_info_.id = devices.first;
    device_info_.port = devices.second;
    connection_state_msg_.devices.push_back(device_info_);
  }

  connection_state_publisher_->publish(connection_state_msg_);
}

int qbDeviceCommunicationHandler::setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  const clock_t begin_time = clock();
  commands.resize(2);  // required by 'setCommandsAndWait()'
  while (failures <= max_repeats) {
    if (devices_.at(id)->setControlReferencesAndWait(commands) < 0) {
      failures++;
      continue;
    }
    break;
  }
  //std::cout << "time setCommandsAndWait of" << id << ": " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  return failures;
}

int qbDeviceCommunicationHandler::setCommandsAsync(const int &id, std::vector<short int> &commands) {
  // qbhand sets only inputs.at(0), but setCommandsAsync expects two-element vector (ok for both qbhand and qbmove)
  commands.resize(2);  // required by 'setCommandsAsync()'
  const clock_t begin_time = clock();
  devices_.at(id)->setControlReferences(commands);
  rclcpp::sleep_for((std::chrono::nanoseconds)(100000)); //0.0001s
  //std::cout << "time setCommandsAsync of" << id << ": " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;
  return 0;  // note that this is a non reliable method
}

bool qbDeviceCommunicationHandler::setCommandsCallback(const std::shared_ptr<qb_device_msgs::srv::SetCommands::Request> request,  std::shared_ptr<qb_device_msgs::srv::SetCommands::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));

  if (request->set_commands) {
    if (!request->set_commands_async) {
      response->failures = setCommandsAndWait(request->id, request->max_repeats, request->commands);  // blocking function
    }
    else {
      response->failures = setCommandsAsync(request->id, request->commands);  // non-blocking (unreliable) function
    }
  }
  response->success = isReliable(response->failures, request->max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::setPID(const int &id, const int &max_repeats, std::vector<float> &pid) {
  // the API methods are called at most (i.e. very unlikely) 'max_repeats' times to guarantee the correct identification of a real fault in the communication
  int failures = 0;
  while (failures <= max_repeats) {
    if (devices_.at(id)->setParamPositionPID(pid) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}

bool qbDeviceCommunicationHandler::setPIDCallback(const std::shared_ptr<qb_device_msgs::srv::SetPid::Request> request,  std::shared_ptr<qb_device_msgs::srv::SetPid::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->success = false;
    return true;
  }
  if (request->p < 0 || request->p > 1 || request->i < 0 || request->i > 0.1 || request->d < 0 || request->d > 0.1) {  //TODO: set hardcoded properly
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),"PID parameters are not in their acceptable ranges");
    response->success = false;
    return true;
  }
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));

  std::vector<float> pid({request->p, request->i, request->d});
  response->failures = setPID(request->id, request->max_repeats, pid);  // blocking function
  response->success = isReliable(response->failures, request->max_repeats);
  return true;
}

int qbDeviceCommunicationHandler::setControlMode(const int &id, const int &max_repeats, uint8_t &control_id) {
  int failures = 0;
  while (failures <= max_repeats) {
    if (devices_.at(id)->setParamControlMode(control_id) < 0) {
      failures++;
      continue;
    }
    break;
  }
  return failures;
}


bool qbDeviceCommunicationHandler::setControlModeCallback(const std::shared_ptr<qb_device_msgs::srv::SetControlMode::Request> request,  std::shared_ptr<qb_device_msgs::srv::SetControlMode::Response> response) {
  RCLCPP_ERROR_STREAM_EXPRESSION(node_handle_->get_logger(),request->max_repeats < 0, "Device [" << request->id << "] has request service with non-valid 'max_request' [" << request->max_repeats << "].");
  if (!isInConnectedSet(request->id)) {
    response->success = false;
    return true;
  }
  uint8_t control_id;
  if (request->control == "position") {
    control_id = 3;
  } else if (request->control == "deflection") {
    control_id = 5;
  } else {
    RCLCPP_ERROR_STREAM(node_handle_->get_logger(),"Not valid control mode request->");
    response->success = false;
    return true;
  }

  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));
  response->failures = setControlMode(request->id, request->max_repeats, control_id);
  response->success = isReliable(response->failures, request->max_repeats);
  return true; 
}

bool qbDeviceCommunicationHandler::goToHomeCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response){ //TODO: implement for all devices
  int failures = 0;
  std::lock_guard<std::mutex> serial_lock(*serial_protectors_.at(connected_devices_.at(request->id)));
  qbrobotics_research_api::qbSoftHand2MotorsResearch SHR2(communication_handler_legacy_, "dev", connected_devices_.at(request->id), request->id);
  while (failures < request->max_repeats) {
    if (SHR2.setHomePosition() == -1) {
      failures++;
      continue;
    }
    break;
  }
  response->failures = failures;
  if(failures < request->max_repeats){
    response->success = true;
    response->message = "Device sent to HOME position";
  } else {
    response->success = false;
    response->message = "Communication error";
  }
  return true;
}
