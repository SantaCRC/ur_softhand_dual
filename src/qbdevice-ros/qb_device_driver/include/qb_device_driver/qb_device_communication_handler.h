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

#ifndef QB_DEVICE_COMMUNICATION_HANDLER_H
#define QB_DEVICE_COMMUNICATION_HANDLER_H

// Standard libraries
#include <mutex>
#include <regex>

// ROS libraries
#include "rclcpp/rclcpp.hpp"

// internal libraries
#include <serial.h>
#include <qbrobotics_research_api/qbsofthand2_research_api.h>
#include <qb_device_msgs/msg/connection_state.hpp>
#include <qb_device_msgs/msg/device_connection_info.hpp>
#include <qb_device_msgs/srv/get_measurements.hpp>
#include <qb_device_msgs/srv/initialize_device.hpp>
#include <qb_device_msgs/srv/set_commands.hpp>
#include <qb_device_msgs/srv/set_pid.hpp>
#include <qb_device_msgs/srv/trigger.hpp>
#include <qb_device_msgs/srv/set_control_mode.hpp>

namespace qb_device_communication_handler {
/**
 * The Communication Handler class is aimed to instantiate a ROS node which provides several ROS services to
 * communicate with one - or many - qbrobotics devices connected to the ROS ecosystem.
 *
 * Each hardware interface which manage a single qbrobotics device must first request the initialization of its ID to
 * the Communication Handler and then interact with it through specific ROS services - blocking by nature. The
 * Communication Handler essentially manage the shared serial port resources and lets all the hardware interfaces to
 * get access to them. This could seem a bottleneck in the architecture, but the real bottleneck is the shared resource
 * itself; moreover this structure is also necessary since the multi-process nature of ROS which does not easily allow
 * to share common resources among distinct processes.
 * Actually the implementation of qbrobotics classes could be reshaped to exploit the multi-threading approach, e.g.
 * using [nodelet](http://wiki.ros.org/nodelet) or similar plugins each device node could handle the communication
 * process by itself, without the intermediary Communication Handler. The advantage of such a restyle in term of
 * communication speed does not worth the effort though.
 */
class qbDeviceCommunicationHandler {
 public:
  /**
   * Wait until at least one device is connected and then initialize the Communication Handler.
   * \sa getSerialPortsAndDevices()
   */
  qbDeviceCommunicationHandler(rclcpp::Node::SharedPtr& node);

  /**
   * @brief Construct a new qb Device Communication Handler object
   * @param scan \p true if you want to scan serial ports and check the connection 
   * otherwise with \p false you will not either start the spinner
   */
  qbDeviceCommunicationHandler(bool scan, rclcpp::Node::SharedPtr& node);

  /**
   * Close all the still open serial ports.
   * \sa close()
   */
  virtual ~qbDeviceCommunicationHandler();

 protected:
  rclcpp::Node::SharedPtr node_handle_;
  std::map<std::string, std::unique_ptr<std::mutex>> serial_protectors_;  // only callbacks must lock the serial resources
  std::map<int, std::string> connected_devices_;

  // handlers to manage the communication with qbdevices
  std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_;
  std::shared_ptr<qbrobotics_research_api::Communication> communication_handler_legacy_;
  // Communication ports
  std::vector<serial::PortInfo> serial_ports_;
  std::map<int, std::shared_ptr<qbrobotics_research_api::Device>> devices_;
  // IDs of connected devices 
  std::vector<qbrobotics_research_api::Communication::ConnectedDeviceInfo> device_ids_;

  /**
   * Activate the motors of the given device. Do nothing if the device is not connected in the Communication Handler.
   * \param id The ID of the device to be activated, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \sa activateCallback(), activate(const int &, const bool &, const int &), isActive()
   */
  virtual int activate(const int &id, const int &max_repeats);

  /**
   * Activate the motors of the device relative to the node requesting the service.
   * \param request The request of the given service (see qb_device_msgs::Trigger for details).
   * \param response The response of the given service (see qb_device_msgs::Trigger for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa activate(const int &, const int &)
   */
  bool activateCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response);

  /**
   * Close the communication with all the devices connected to the given serial port.
   * \param serial_port The serial port which has to be closed, e.g. \p /dev/ttyUSB0.
   * \sa open(), qbrobotics_research_api::Communication::closeSerialPort
   */
  virtual int close(const std::string &serial_port);
  
  /**
   * Starts walltimer then check comunication
   */
  void checkActivation();

  /**
   * Called by check_connection_status_timer_. It verify if a qbdevice is connected or not
   */
  void checkConnectionAndPublish();

  /**
   * Deactivate the motors of the given device. Do nothing if the device is not connected in the Communication Handler.
   * \param id The ID of the device to be deactivated, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \sa deactivateCallback(), activate(const int &, const bool &, const int &), isActive()
   */
  virtual int deactivate(const int &id, const int &max_repeats);

  /**
   * Deactivate the motors of the device relative to the node requesting the service.
   * \param request The request of the given service (see qb_device_msgs::Trigger for details).
   * \param response The response of the given service (see qb_device_msgs::Trigger for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa deactivate()
   */
  bool deactivateCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response);

  /**
   * Retrieve the motor currents of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
   * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
   * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qb_device_driver::qbDeviceAPI::getCurrents(), getMeasurementsCallback(), getMeasurements()
   */
  virtual int getCurrents(const int &id, const int &max_repeats, std::vector<short int> &currents);

  /**
   * Retrieve the printable configuration setup of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \param info The configuration setup formatted as a plain text string (empty string on communication error).
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qb_device_driver::qbDeviceAPI::getInfo(), getInfoCallback(), getParameters()
   */
  virtual int getInfo(const int &id, const int &max_repeats, std::string &info);

  /**
   * Retrieve the printable configuration setup of the device relative to the node requesting the service.
   * \param request The request of the given service (see qb_device_msgs::Trigger for details).
   * \param response The response of the given service (see qb_device_msgs::Trigger for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa getInfo()
   */
  bool getInfoCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response);

  /**
   * Retrieve the reference command to the motors of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only
   * the first element is filled while the other remains always \p 0; in the case of a \em qbmove both the elements
   * contain relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of commands retrieved, i.e. the number of motors equipped on the given device.
   */
  virtual int getCommands(const int &id, const int &max_repeats, std::vector<short int> &commands);

  /**
   * Retrieve the motor currents of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
   * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
   * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
   * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
   * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
   * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
   * not directly actuated).
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qb_device_driver::qbDeviceAPI::getMeasurements(), getMeasurementsCallback(), getCurrents(), getPositions()
   */
  virtual int getMeasurements(const int &id, const int &max_repeats, std::vector<short int> &currents, std::vector<short int> &positions);

  /**
   * Retrieve the motor positions and currents of the device relative to the node requesting the service.
   * \param request The request of the given service (see qb_device_msgs::GetMeasurements for details).
   * \param response The response of the given service (see qb_device_msgs::GetMeasurements for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa getMeasurements(), getCurrents(), getPositions()
   */
  bool getMeasurementsCallback(const std::shared_ptr<qb_device_msgs::srv::GetMeasurements::Request> request,  std::shared_ptr<qb_device_msgs::srv::GetMeasurements::Response> response);

  /**
   * Retrieve some of the parameters from the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] limits The vector of motor position limits expressed in \em ticks: two values for each motor,
   * respectively [\p lower_limit, \p upper_limit].
   * \param[out] resolutions The vector of encoder resolutions, each in range [\p 0, \p 8]: one value for each encoder
   * (\b note: the \em qbmove has also the shaft encoder even if it is not actuated). The word "resolution" could be
   * misunderstood: taken the resolution \p r, \f$2^r\f$ is the number of turns of the wire inside the device mechanics.
   * It is used essentially to convert the measured position of the motors in \em ticks to \em radians or \em degrees.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::getParameters(), getInfo(), initializeCallback()
   */
  virtual int getParameters(const int &id, std::vector<int32_t> &limits, std::vector<uint8_t> &resolutions);

  /**
   * Retrieve the motor positions of the given device.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
   * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
   * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
   * not directly actuated).
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qb_device_driver::qbDeviceAPI::getPositions(), getMeasurementsCallback(), getMeasurements()
   */
  virtual int getPositions(const int &id, const int &max_repeats, std::vector<short int> &positions);

  /**
   * Scan for all the serial ports of type \p /dev/ttyUSB* detected in the system and retrieve all
   * the qbrobotics devices connected to them. For each device, store its ID in the private map \p connected_devices_,
   * i.e. insert a pair [\p device_id, \p serial_port]. The map \p connected_devices_ is constructed from scratch at
   * each call.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return the number of connected devices.
   * \sa open(), qbrobotics_research_api::Communication::listSerialPorts, qbrobotics_research_api::Communication::listConnectedDevices, close()
   */
  virtual int getSerialPortsAndDevices(const int &max_repeats);

  /**
   * Check whether the motors of the device specified by the given ID are active.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \param status \p true if the device motors are on.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qbrobotics_research_api::getMotorStates()
   */
  virtual int isActive(const int &id, const int &max_repeats, bool &status);

  /**
   * Check whether the the device specified by the given ID is connected through the serial port.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return The number of failure reads between \p 0 and \p max_repeats.
   * \sa qb_device_driver::qbDeviceAPI::getStatus()
   */
  virtual int isConnected(const int &id, const int &max_repeats);

  /**
   * Check whether the physical device specified by the given ID is connected to the Communication Handler.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \return \p true if the given device belongs to the connected device vector, i.e. \p connected_devices_.
   * \sa getSerialPortsAndDevices()
   */
  virtual bool isInConnectedSet(const int &id);

  /**
   * Check whether the given serial port is managed by the Communication Handler, i.e. is open.
   * \param serial_port The name of the serial port of interest, e.g. \p /dev/ttyUSB0.
   * \return \p true if the given serial port belongs to the open file descriptor map, i.e. \p file_descriptors_.
   * \sa open(), close()
   */
  //virtual bool isInOpenMap(const std::string &serial_port);

  /**
   * Initialize the device node requesting the service to the Communication Handler if the relative physical device is
   * connected through any serial port to the system (can re-scan the serial resources if specified in the request).
   * If the device is found, retrieve some of its parameter and activate its motors, if requested.
   * \param request The request of the given service (see qb_device_msgs::InitializeDevice for details).
   * \param response The response of the given service (see qb_device_msgs::InitializeDevice for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa getSerialPortsAndDevices()
   */
  virtual bool initializeCallback(const std::shared_ptr<qb_device_msgs::srv::InitializeDevice::Request> request,  std::shared_ptr<qb_device_msgs::srv::InitializeDevice::Response> response);

  /**
   * Send the reference command to the motors of the given device and wait for acknowledge.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
   * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
   * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
   * \return \p 0 on success.
   * \sa qb_device_driver::qbDeviceAPI::setCommandsAndWait(), setCommandsCallback()
   */
  virtual int setCommandsAndWait(const int &id, const int &max_repeats, std::vector<short int> &commands);

  /**
   * Send the reference command to the motors of the given device in a non-blocking fashion.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
   * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
   * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
   * \return Always \p 0 (note that this is a non reliable method).
   * \sa qb_device_driver::qbDeviceAPI::setCommandsAsync(), setCommandsCallback()
   */
  virtual int setCommandsAsync(const int &id, std::vector<short int> &commands);

  /**
   * Send the reference command to the motors of the device relative to the node requesting the service.
   * \param request The request of the given service (see qb_device_msgs::SetCommands for details).
   * \param response The response of the given service (see qb_device_msgs::SetCommands for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa setCommandsAndWait(), setCommandsAsync()
   */
  bool setCommandsCallback(const std::shared_ptr<qb_device_msgs::srv::SetCommands::Request> request,  std::shared_ptr<qb_device_msgs::srv::SetCommands::Response> response);

  /**
   * Set the position control PID parameters of the given device, temporarily (until power off).
   * \param file_descriptor The file descriptor of the serial port on which the device is connected.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param pid The three-element [\p P, \p I, \p D] vector of parameters to be set.
   * \return Always \p 0 (note that this is a non reliable method).
   * \sa qb_device_driver::qbDeviceAPI::setPID(), setPIDCallback()
   */
  int setPID(const int &id, const int &max_repeats, std::vector<float> &pid);

  /**
   * Set (temporarily, i.e. until power off) the position control PID parameters of the device relative to the node
   * requesting the service.
   * \param request The request of the given service (see qb_device_msgs::SetPid for details).
   * \param response The response of the given service (see qb_device_msgs::SetPid for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa setPID()
   */
  bool setPIDCallback(const std::shared_ptr<qb_device_msgs::srv::SetPid::Request> request,  std::shared_ptr<qb_device_msgs::srv::SetPid::Response> response);

  /**
   * Set control mode for qbmove.
   * \param id The ID of the device of interest, in range [\p 1, \p 128].
   * \param control_id the control id for qbmove: 0 -> POSITION, 4 -> DEFLECTION.
   * \return Always \p 0 (note that this is a non reliable method).
   * \sa qb_device_driver::qbDeviceAPI::setPID(), setPIDCallback()
   */
  int setControlMode(const int &id, const int &max_repeats, uint8_t &control_id);

  /**
   * Set (temporarily, i.e. until power off) the control mode parameters of the device relative to the node
   * requesting the service.
   * \param request The request of the given service (see qb_device_msgs::setControlMode for details).
   * \param response The response of the given service (see qb_device_msgs::setControlMode for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   * \sa setControlMode()
   */
  bool setControlModeCallback(const std::shared_ptr<qb_device_msgs::srv::SetControlMode::Request> request,  std::shared_ptr<qb_device_msgs::srv::SetControlMode::Response> response);

  /**
   * Send the device to HOME position.
   * \warning if this service is used while a trajectory for the device is defined, the device will move to home and then will return 
   * to the position preceding the call of this service. In such a case, use the service in qbdevice_control. 
   * \param request The request of the given service (see qb_device_msgs::setControlMode for details).
   * \param response The response of the given service (see qb_device_msgs::setControlMode for details).
   * \return \p true if the call succeed (actually \p response.success may be false).
   */
  bool goToHomeCallback(const std::shared_ptr<qb_device_msgs::srv::Trigger::Request> request,  std::shared_ptr<qb_device_msgs::srv::Trigger::Response> response);

 private:
  rclcpp::Service<qb_device_msgs::srv::Trigger>::SharedPtr activate_motors_;
  rclcpp::Service<qb_device_msgs::srv::Trigger>::SharedPtr deactivate_motors_;
  rclcpp::Service<qb_device_msgs::srv::Trigger>::SharedPtr get_info_;
  rclcpp::Service<qb_device_msgs::srv::GetMeasurements>::SharedPtr get_measurements_;
  rclcpp::Service<qb_device_msgs::srv::InitializeDevice>::SharedPtr initialize_device_;
  rclcpp::Service<qb_device_msgs::srv::SetCommands>::SharedPtr set_commands_;
  rclcpp::Service<qb_device_msgs::srv::SetPid>::SharedPtr set_pid_;
  rclcpp::Service<qb_device_msgs::srv::SetControlMode>::SharedPtr set_control_mode_;
  rclcpp::Service<qb_device_msgs::srv::Trigger>::SharedPtr go_to_home_;
  rclcpp::TimerBase::SharedPtr check_connection_status_timer_;
  rclcpp::Publisher<qb_device_msgs::msg::ConnectionState>::SharedPtr connection_state_publisher_;
  qb_device_msgs::msg::ConnectionState connection_state_msg_;

  /**
   * Activate (or deactivate, according to the given command) the motors of the given device. Do nothing if the device
   * is not connected in the Communication Handler.
   * \param id The ID of the device to be activated (or deactivated), in range [\p 1, \p 128].
   * \param command \p true to turn motors on, \p false to turn them off.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \sa qbrobotics_research_api::Device::setMotorStates(), isActive()
   */
  virtual int activate(const int &id, const bool &command, const int &max_repeats);

  /**
   * Check whether the reading failures are in the given range.
   * \param failures The current number of communication failures per serial resource reading.
   * \param max_repeats The maximum number of consecutive repetitions to mark retrieved data as corrupted.
   * \return \p true if the failures are less than the given threshold.
   */
  inline bool isReliable(int const &failures, int const &max_repeats) { return failures >= 0 && failures <= max_repeats; }
};
}  // namespace qb_device_communication_handler

#endif // QB_DEVICE_COMMUNICATION_HANDLER_H