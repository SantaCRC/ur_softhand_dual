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

#include "qb_device_ros2_control/qb_device_ros2_control.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission_loader.hpp"
#include "transmission_interface/transmission.hpp"
#include "transmission_interface/transmission_interface_exception.hpp"

namespace qb_device_ros2_control
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

hardware_interface::CallbackReturn DeviceHW::on_init(
  const hardware_interface::HardwareInfo & info)
{
  logger_ = std::make_unique<rclcpp::Logger>(
    rclcpp::get_logger("DeviceHW"));
  
  joint_position_.assign(5, 0); //max joints number for qb device
  joint_position_.resize(info_.joints.size());
  
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_ = rclcpp::Node::make_shared("qbdevice_hw_interface", node_options);

  clock_ = std::make_unique<rclcpp::Clock>();

  RCLCPP_INFO(*logger_, "Initializing...");

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // actuator_slowdown_ = std::stod(info_.hardware_parameters["actuator_slowdown"]);

  const auto num_joints = std::accumulate(
    info_.transmissions.begin(), info_.transmissions.end(), 0ul,
    [](const auto & acc, const auto & trans_info) { return acc + trans_info.joints.size(); });

  const auto num_actuators = std::accumulate(
    info_.transmissions.begin(), info_.transmissions.end(), 0ul,
    [](const auto & acc, const auto & trans_info) { return acc + trans_info.actuators.size(); });

  // reserve the space needed for joint and actuator data structures
  joint_interfaces_.reserve(num_joints);
  actuator_interfaces_.reserve(num_actuators);

  // create transmissions, joint and actuator handles
  auto transmission_loader = transmission_interface::SimpleTransmissionLoader();

  for (const auto & transmission_info : info_.transmissions)
  {
    // only simple transmissions are supported in this package
    if (transmission_info.type != "transmission_interface/SimpleTransmission")
    {
      RCLCPP_FATAL(
        *logger_, "Transmission '%s' of type '%s' not supported in this package",
        transmission_info.name.c_str(), transmission_info.type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::shared_ptr<transmission_interface::Transmission> transmission;
    try
    {
      transmission = transmission_loader.load(transmission_info);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
      RCLCPP_FATAL(
        *logger_, "Error while loading %s: %s", transmission_info.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<transmission_interface::JointHandle> joint_handles;
    for (const auto & joint_info : transmission_info.joints)
    {
      // // this package supports only one interface per joint
      // if (!(joint_info.state_interfaces.size() == 1 &&
      //       joint_info.state_interfaces[0] == hardware_interface::HW_IF_POSITION &&
      //       joint_info.command_interfaces.size() == 1 &&
      //       joint_info.command_interfaces[0] == hardware_interface::HW_IF_POSITION))
      // {
      //   RCLCPP_FATAL(
      //     *logger_, "Invalid transmission joint '%s' configuration",
      //     joint_info.name.c_str());
      //   return hardware_interface::CallbackReturn::ERROR;
      // }

      const auto joint_interface =
        joint_interfaces_.insert(joint_interfaces_.end(), InterfaceData(joint_info.name));

      transmission_interface::JointHandle joint_handle(
        joint_info.name, hardware_interface::HW_IF_POSITION,
        &joint_interface->transmission_passthrough_);
      joint_handles.push_back(joint_handle);
    }

    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto & actuator_info : transmission_info.actuators)
    {
      // no check for actuators types

      const auto actuator_interface =
        actuator_interfaces_.insert(actuator_interfaces_.end(), InterfaceData(actuator_info.name));
      transmission_interface::ActuatorHandle actuator_handle(
        actuator_info.name, hardware_interface::HW_IF_POSITION,
        &actuator_interface->transmission_passthrough_);
      actuator_handles.push_back(actuator_handle);
    }

    /// @note no need to store the joint and actuator handles, the transmission
    /// will keep whatever info it needs after is done with them

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException & exc)
    {
      RCLCPP_FATAL(
        *logger_, "Error while configuring %s: %s", transmission_info.name.c_str(), exc.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    transmissions_.push_back(transmission);
  }

  this->initializeServicesAndWait();
  this->waitForInitialization();
  RCLCPP_INFO(*logger_, "Initialization successful");

  RCLCPP_INFO_STREAM(*logger_, "" << getInfo());
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DeviceHW::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Configuring...");

  auto reset_interfaces = [](std::vector<InterfaceData> & interfaces)
  {
    for (auto & interface_data : interfaces)
    {
      interface_data.command_ = 0.0;
      interface_data.state_ = 0.0;
      interface_data.transmission_passthrough_ = kNaN;
    }
  };

  reset_interfaces(joint_interfaces_);
  reset_interfaces(actuator_interfaces_);

  RCLCPP_INFO(*logger_, "Configuration successful");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DeviceHW::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // we have only one interface
  int ind = 0;
  for (const auto & joint : info_.joints)
  {
    state_interfaces.emplace_back(joint.name, "position", &joint_position_[ind++]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DeviceHW::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & joint : info_.joints)
  {
    /// @pre all joint interfaces exist, checked in on_init()
    auto joint_interface = std::find_if(
      joint_interfaces_.begin(), joint_interfaces_.end(),
      [&](const InterfaceData & interface) { return interface.name_ == joint.name; });

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint.name, hardware_interface::HW_IF_POSITION, &joint_interface->command_));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn DeviceHW::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Activating...");
  
  if(activateMotors()==0) {  
    RCLCPP_INFO(*logger_, "Activation successful");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  RCLCPP_WARN_STREAM(*logger_, "Activation failed!");
  return hardware_interface::CallbackReturn::FAILURE; 
}

hardware_interface::CallbackReturn DeviceHW::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(*logger_, "Deactivating...");
  
  if(deactivateMotors()==0) {  
    RCLCPP_INFO(*logger_, "Deactivation successful");
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  
  RCLCPP_WARN_STREAM(*logger_, "Deactivation failed!");
  return hardware_interface::CallbackReturn::FAILURE; 
}

hardware_interface::return_type DeviceHW::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // actuator: state -> transmission
  rclcpp::Time stamp;
  std::vector<double> actual_pos, actual_curr;
  actual_pos.assign(4, 0); //max encoders number for qb device
  actual_pos.resize(actuator_interfaces_.size());
  actual_curr.assign(4, 0); //max encoders number for qb device
  actual_curr.resize(actuator_interfaces_.size());

  this->getMeasurements(actual_pos, actual_curr, stamp);
  
  this->readingsToActuatorsPosition(actual_pos);

  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface)
    { actuator_interface.transmission_passthrough_ = actuator_interface.state_; });

  // transmission: actuator -> joint
  std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->actuator_to_joint(); });

  // joint: transmission -> state
  std::for_each(
    joint_interfaces_.begin(), joint_interfaces_.end(),
    [](auto & joint_interface)
    { joint_interface.state_ = joint_interface.transmission_passthrough_; });
  
  int ind = 0;
  for (const auto & joint_interface : joint_interfaces_)
  {
    joint_position_[ind++]= joint_interface.state_;
  }

  // // log state data
  // std::stringstream ss;
  // ss << "State data:";
  // for (const auto & transmission_info : info_.transmissions)
  // {
  //   // this only for simple transmissions, now it will log only one joint
  //   const auto joint_interface = std::find_if(
  //     joint_interfaces_.cbegin(), joint_interfaces_.cend(),
  //     [&](const auto & joint_interface)
  //     { return joint_interface.name_ == transmission_info.joints[0].name; });

  //   const auto actuator_interface = std::find_if(
  //     actuator_interfaces_.cbegin(), actuator_interfaces_.cend(),
  //     [&](const auto & actuator_interface)
  //     { return actuator_interface.name_ == transmission_info.actuators[0].name; });

  //   const auto & reduction = transmission_info.joints[0].mechanical_reduction;

  //   ss << std::endl
  //      << "\t" << joint_interface->name_ << ": " << joint_interface->state_ << " <-- "
  //      << transmission_info.name << "(R=" << reduction << ") <-- " << actuator_interface->name_
  //      << ": " << actuator_interface->state_;
  // }
  // RCLCPP_INFO_THROTTLE(*logger_, *clock_, 1000, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DeviceHW::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // joint: command -> transmission
  std::for_each(
    joint_interfaces_.begin(), joint_interfaces_.end(),
    [](auto & joint_interface)
    { joint_interface.transmission_passthrough_ = joint_interface.command_; });

  // transmission: joint -> actuator
  std::for_each(
    transmissions_.begin(), transmissions_.end(),
    [](auto & transmission) { transmission->joint_to_actuator(); });

  // actuator: transmission -> command
  std::for_each(
    actuator_interfaces_.begin(), actuator_interfaces_.end(),
    [](auto & actuator_interface)
    { actuator_interface.command_ = actuator_interface.transmission_passthrough_; });

  std::vector<double> commands;
  commands.assign(2, 0); //max motors for qb device

  this->actuatorsCommandsToWrite(commands);

  this->setCommands(commands);

  // // log command data
  // std::stringstream ss;
  // ss << "Command data:";
  // for (const auto & transmission_info : info_.transmissions)
  // {
  //   // this only for simple transmissions, now it will log only one joint
  //   const auto joint_interface = std::find_if(
  //     joint_interfaces_.cbegin(), joint_interfaces_.cend(),
  //     [&](const auto & joint_interface)
  //     { return joint_interface.name_ == transmission_info.joints[0].name; });

  //   const auto actuator_interface = std::find_if(
  //     actuator_interfaces_.cbegin(), actuator_interfaces_.cend(),
  //     [&](const auto & actuator_interface)
  //     { return actuator_interface.name_ == transmission_info.actuators[0].name; });

  //   const auto & reduction = transmission_info.joints[0].mechanical_reduction;

  //   ss << std::endl
  //      << "\t" << joint_interface->name_ << ": " << joint_interface->command_ << " --> "
  //      << transmission_info.name << "(R=" << reduction << ") --> " << actuator_interface->name_
  //      << ": " << actuator_interface->command_;
  // }
  // RCLCPP_INFO_THROTTLE(*logger_, *clock_, 1000, "%s", ss.str().c_str());

  return hardware_interface::return_type::OK;
}

void DeviceHW::readingsToActuatorsPosition(std::vector<double> &actual_positions){
  int ind = 0;
  for (const auto & pose : actual_positions)
  {
    actuator_interfaces_.at(ind++).state_ = pose;
  }
}

void DeviceHW::actuatorsCommandsToWrite(std::vector<double> &commands_to_write){
  for (int i=0; i<actuator_interfaces_.size() &&  i<commands_to_write.size(); i++)
  {
    commands_to_write[i] = actuator_interfaces_.at(i).command_;
  }
}

DeviceHW::InterfaceData::InterfaceData(const std::string & name)
: name_(name), command_(kNaN), state_(kNaN), transmission_passthrough_(kNaN)
{
}

void DeviceHW::initializeServicesAndWait(){

  get_measurements_client_ = node_->create_client<qb_device_msgs::srv::GetMeasurements>("/communication_handler/get_measurements");
  set_commands_client_ = node_->create_client<qb_device_msgs::srv::SetCommands>("/communication_handler/set_commands");
  activate_client_ = node_->create_client<qb_device_msgs::srv::Trigger>("/communication_handler/activate_motors");
  deactivate_client_ = node_->create_client<qb_device_msgs::srv::Trigger>("/communication_handler/deactivate_motors");
  get_info_client_ = node_->create_client<qb_device_msgs::srv::Trigger>("/communication_handler/get_info");
  initialize_device_client_ = node_->create_client<qb_device_msgs::srv::InitializeDevice>("/communication_handler/initialize_device");
  client_list_controller_ = node_->create_client<controller_manager_msgs::srv::ListControllers>("controller_manager/list_controllers");
  switch_control_mode_client_ = node_->create_client<qb_device_msgs::srv::SetControlMode>("/communication_handler/set_control_mode");
  this->waitForSrvs();
}

void DeviceHW::waitForSrvs(){
  activate_client_ ->wait_for_service();
  deactivate_client_ ->wait_for_service();
  get_measurements_client_->wait_for_service();
  set_commands_client_->wait_for_service();
  get_info_client_->wait_for_service();
  initialize_device_client_->wait_for_service();
  switch_control_mode_client_->wait_for_service();
  // client_list_controller_->wait_for_service();
  RCLCPP_INFO_STREAM(*logger_, "All necessary servers exist in [communication_handler]");
}

std::string DeviceHW::getInfo() {
  if (get_info_client_->service_is_ready()) {
    auto req = std::make_shared<qb_device_msgs::srv::Trigger::Request>();
    req->id = device_.id;
    req->max_repeats = device_.max_repeats;

    auto result = get_info_client_->async_send_request(req);
    if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {  
      auto res = result.get();
      return res->message;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot get info from device [" << device_.id << "].");
    return "";
  }
  RCLCPP_WARN_STREAM(*logger_, "[DeviceHW] service [get_info] seems no longer advertised.");
  resetServicesAndWait();
  return "";
}

int DeviceHW::getMeasurements(std::vector<double> &positions, std::vector<double> &currents, rclcpp::Time &stamp) {
  std::vector<double> commands;  // not used
  return getMeasurements(positions, currents, commands, stamp);
}

int DeviceHW::getMeasurements(std::vector<double> &positions, std::vector<double> &currents, std::vector<double> &commands, rclcpp::Time &stamp) {
  if (get_measurements_client_->service_is_ready()) {
    auto req = std::make_shared<qb_device_msgs::srv::GetMeasurements::Request>();
    req->id = device_.id;
    req->max_repeats = device_.max_repeats;
    req->get_currents = device_.get_currents;
    req->get_positions = device_.get_positions;
    req->get_commands = device_.get_commands;  // usually false, it is used for async requests only
    
    auto result = get_measurements_client_->async_send_request(req);

    if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
      
      auto res = result.get();
      // positions and currents cannot be resized because actuators state handle in the caller stores their addresses
      for (int i=0; i<(int)(positions.size()) && i<(int)(res->positions.size()); i++) {
        positions.at(i) = res->positions.at(i) * device_.motor_axis_direction;
      }
      for (int i=0; i<(int)(currents.size()) && i<(int)(res->currents.size()); i++) {
        currents.at(i) = res->currents.at(i);
      }
      for (int i=0; i<(int)(commands.size()) && i<(int)(res->commands.size()); i++) {
        commands.at(i) = res->commands.at(i);
      }
      stamp = res->stamp;
 
      return res->failures;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot get measurements from device [" << device_.id << "].");
    return -1;
  }
  RCLCPP_WARN_STREAM(*logger_, "Service [get_measurements] seems no longer advertised.");
  resetServicesAndWait();
  return -1; 
}

int DeviceHW::setCommands(const std::vector<double> &commands){
  if(set_commands_client_->service_is_ready()){
    auto req = std::make_shared<qb_device_msgs::srv::SetCommands::Request>();
    req->id = device_.id; 
    req->max_repeats = device_.max_repeats;
    req->set_commands = device_.set_commands;
    req->set_commands_async = device_.set_commands_async;

    for (auto const &command : commands) {
      req->commands.push_back(static_cast<short int>(command) * device_.motor_axis_direction);
    }

    auto result = set_commands_client_->async_send_request(req);
    
    if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {  
      return 0;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot send commands to device [" << device_.id << "].");
    return -1;
  }
  RCLCPP_WARN_STREAM(*logger_, "Service [set_commands] seems no longer advertised.");
  resetServicesAndWait();
  return -1; 
}

void DeviceHW::resetServicesAndWait(const bool &reinitialize_device) {
  // reset all the service clients in case they were not yet advertised during initialization
  initializeServicesAndWait();
  if (reinitialize_device) {
    waitForInitialization();
  }
}

void DeviceHW::waitForInitialization() {
  while(initializeDevice()) {
    rclcpp::sleep_for((std::chrono::seconds)(1)); //1s
  }
}

int DeviceHW::initializeDevice() {
  
  device_.id = node_->get_parameter("device_id").as_int();
  
  if(initialize_device_client_->service_is_ready()){
    auto req = std::make_shared<qb_device_msgs::srv::InitializeDevice::Request>();
    req->id = device_.id;
    req->activate = false;
    req->rescan = node_->get_parameter("rescan_on_initialization").as_bool();
    int max_repeats = node_->get_parameter("max_repeats").as_int();
    req->max_repeats = max_repeats;
    
    auto result = initialize_device_client_->async_send_request(req);

    if (!(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)) {
      RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot initialize device [" << device_.id << "].");
      return -1;
    }

    device_.max_repeats = max_repeats;
    device_.get_currents = node_->get_parameter("get_currents").as_bool();
    device_.get_positions = node_->get_parameter("get_positions").as_bool();
    device_.set_commands = node_->get_parameter("set_commands").as_bool();
    device_.set_commands_async = node_->get_parameter("set_commands_async").as_bool();

    auto res = result.get();
    
    device_.serial_port = res->info.serial_port;
    device_.position_limits = res->info.position_limits;
    device_.encoder_resolutions = res->info.encoder_resolutions;

    device_info_.id = device_.id;
    device_info_.serial_port = device_.serial_port;
    device_info_.max_repeats = device_.max_repeats;
    device_info_.get_currents = device_.get_currents;
    device_info_.get_positions = device_.get_positions;
    device_info_.get_distinct_packages = device_.get_distinct_packages;
    device_info_.set_commands = device_.set_commands;
    device_info_.set_commands_async = device_.set_commands_async;
    device_info_.position_limits = device_.position_limits;
    device_info_.encoder_resolutions = device_.encoder_resolutions;

    RCLCPP_INFO_STREAM(*logger_, "[DeviceHW] device [" << device_.id << "] is initialized.");
    return 0;
  }
  RCLCPP_WARN_STREAM(*logger_, "[DeviceHW] service [initialize_device] is no longer advertised.");
  resetServicesAndWait(false);
  return -1;
}

int DeviceHW::activateMotors() {
  if (activate_client_->service_is_ready()) {
    auto req = std::make_shared<qb_device_msgs::srv::Trigger::Request>();
    req->id = device_.id;
    req->max_repeats = device_.max_repeats;
    auto result = activate_client_->async_send_request(req);
    
    if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {  
      RCLCPP_INFO_STREAM(*logger_, "[DeviceHW] device [" << device_.id << "] motors are active!");
      return 0;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot activate device [" << device_.id << "].");
    return -1;
  }
  RCLCPP_WARN_STREAM(*logger_, "Service [activate_motors] seems no longer advertised.");
  resetServicesAndWait();
  return -1; 
}

int DeviceHW::deactivateMotors() {
  if (deactivate_client_->service_is_ready()) {
    auto req = std::make_shared<qb_device_msgs::srv::Trigger::Request>();
    req->id = device_.id;
    req->max_repeats = device_.max_repeats;
    auto result = deactivate_client_->async_send_request(req);
    
    if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {  
      RCLCPP_INFO_STREAM(*logger_, "[DeviceHW] device [" << device_.id << "] motors are stopped!");
      return 0;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot deactivate device [" << device_.id << "].");
    return -1;
  }
  RCLCPP_WARN_STREAM(*logger_, "Service [deactivate_motors] seems no longer advertised.");
  resetServicesAndWait();
  return -1; 
}

int DeviceHW::setControlMode(const std::string &control_mode) {
  if (switch_control_mode_client_->service_is_ready()) {
    auto req = std::make_shared<qb_device_msgs::srv::SetControlMode::Request>();
    req->id = device_.id;
    req->max_repeats = device_.max_repeats;
    req->control = control_mode;
    auto result = switch_control_mode_client_->async_send_request(req);
    
    if(rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {  
      RCLCPP_INFO_STREAM(*logger_, "[DeviceHW] device [" << device_.id << "] control mode set to " << control_mode);
      return 0;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "[DeviceHW] cannot change control mode of device [" << device_.id << "].");
    return -1;
  }
  RCLCPP_WARN_STREAM(*logger_, "Service [switch_control_mode] seems no longer advertised.");
  resetServicesAndWait();
  return -1; 
}

int DeviceHW::getActiveControllers(std::vector<std::string> &list_controllers){
  if(client_list_controller_->service_is_ready()){
    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto result = client_list_controller_->async_send_request(req);

    if(rclcpp::spin_until_future_complete(node_, result, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS) {  
      auto res = result.get();
      for (auto controller : res->controller){
        if(controller.state == "active"){
          list_controllers.push_back(controller.name);
        }
      }
      return 0;
    }

    RCLCPP_ERROR_STREAM_THROTTLE( *logger_, *clock_, 60, "Cannot get list of controllers.");
    return -1;
  }
  RCLCPP_WARN_STREAM(*logger_, "Service [list_controllers] seems no longer advertised.");

  return -1; 
}

}  // namespace qb_device_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  qb_device_ros2_control::DeviceHW,
  hardware_interface::SystemInterface)
