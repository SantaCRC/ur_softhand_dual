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

#ifndef QB_DEVICE_ROS2_CONTROL_HPP_
#define QB_DEVICE_ROS2_CONTROL_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <regex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/state.hpp"
#include "transmission_interface/transmission.hpp"

#include "controller_manager_msgs/srv/list_controllers.hpp"

#include <qb_device_msgs/msg/resource_data.hpp>
#include <qb_device_msgs/msg/state.hpp>
#include <qb_device_msgs/msg/state_stamped.hpp>
#include <qb_device_msgs/msg/info.hpp>
#include <qb_device_msgs/srv/get_measurements.hpp>
#include <qb_device_msgs/srv/set_commands.hpp>
#include <qb_device_msgs/srv/trigger.hpp>
#include <qb_device_msgs/srv/initialize_device.hpp>
#include <qb_device_msgs/srv/set_control_mode.hpp>

namespace qb_device_ros2_control
{

  /**
   * The qbrobotics Device Resources contains just few device information. The most important is the device ID.
   */
  class qbDeviceResources {
    public:
      /**
       * Construct the Resource with the default ID, \p 1.
       * \sa qbDeviceResources(const int &)
       */
      qbDeviceResources()
          : qbDeviceResources(1) {}

      /**
       * Construct the Resource from the "signed" device ID.
       * \param signed_id The "signed" device ID is a value in range [\p -128, \p 128]: the \em sign specify the motor axis
       * direction, while the absolute value represent the real device ID. \b Note: actually \p 0 is reserved for broadcast
       * communication.
       */
      qbDeviceResources(const int &signed_id)
          : motor_axis_direction((0 < signed_id) - (signed_id < 0)),
            id(std::abs(signed_id)),
            max_repeats(3),
            get_currents(true),
            get_positions(true),
            get_distinct_packages(false),
            get_commands(false),
            set_commands(true),
            set_commands_async(false) {
        // qbhand uses only a subset of the followings
        position_limits.resize(4);  // qbmove has two motors
        encoder_resolutions.resize(3);  // qbmove has one encoder per motor and one for the shaft
      }

    /**
     * Do nothing.
     */
    virtual ~qbDeviceResources() {}

    // device info
    int id;
    int motor_axis_direction;
    std::string name;  // device name is chosen to properly set ROS namespaces to avoid name clashes
    std::string serial_port;  // serial port to which the device is connected
    std::vector<int32_t> position_limits;  // lower and upper limits for each motor [ticks]
    std::vector<uint8_t> encoder_resolutions;  // used to convert from [ticks] to [radians/degrees] and vice versa

    // read/write settings for the current device
    int max_repeats;  // max number of consecutive repetitions to mark retrieved data as corrupted
    bool get_currents;  // specify if motor currents are retrieved at each control loop
    bool get_positions;  // specify if motor positions are retrieved at each control loop
    bool get_distinct_packages;  // old devices cannot retrieve current and position measurements in a single package
    bool get_commands;  // specify if motor references are retrieved at each control loop
    bool set_commands;  // specify if command references are sent at each control loop
    bool set_commands_async;  // specify if command references are sent in a non-blocking fashion
    bool use_joint_limits;  // specify if the device has to use the firmware limits or the yaml one
  };

  class DeviceHW : public hardware_interface::SystemInterface
  {
    public:

      hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;


      hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;


      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;


      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


      hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;


      hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;


      hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;


      hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:

      std::unique_ptr<rclcpp::Logger> logger_;
      std::unique_ptr<rclcpp::Clock> clock_;

      std::shared_ptr<rclcpp::Node> node_;

      qb_device_msgs::msg::Info device_info_;
      qbDeviceResources device_;

      int max_repeats_ = 0;
      std::string device_name_;

      // transmissions
      std::vector<std::shared_ptr<transmission_interface::Transmission>> transmissions_;

      struct InterfaceData
      {
        explicit InterfaceData(const std::string & name);

        std::string name_;
        double command_;
        double state_;

        // this is the "sink" that will be part of the transmission Joint/Actuator handles
        double transmission_passthrough_;
      };
      std::vector<InterfaceData> joint_interfaces_;
      std::vector<InterfaceData> actuator_interfaces_;
      std::vector<double> joint_position_;

      rclcpp::Client<qb_device_msgs::srv::GetMeasurements>::SharedPtr get_measurements_client_;
      rclcpp::Client<qb_device_msgs::srv::SetCommands>::SharedPtr set_commands_client_;
      rclcpp::Client<qb_device_msgs::srv::Trigger>::SharedPtr activate_client_;
      rclcpp::Client<qb_device_msgs::srv::Trigger>::SharedPtr deactivate_client_;
      rclcpp::Client<qb_device_msgs::srv::Trigger>::SharedPtr get_info_client_;
      rclcpp::Client<qb_device_msgs::srv::InitializeDevice>::SharedPtr initialize_device_client_; 
      rclcpp::Client<qb_device_msgs::srv::SetControlMode>::SharedPtr switch_control_mode_client_; 
      rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr client_list_controller_;

      /**
       * Call the service to activate the device motors and wait for the response.
       * \return \p 0 on success.
       * \sa deactivateMotors()
       */
      virtual int activateMotors();

      /**
       * Call the service to deactivate the device motors and wait for the response.
       * \return \p 0 on success.
       * \sa activateMotors()
       */
      virtual int deactivateMotors();

      /**
       * Call the service to retrieve the printable configuration setup of the device and wait for the response.
       * \return The configuration setup formatted as a plain text string (empty string on error).
       */
      virtual std::string getInfo();

      /**
       * Call the service to retrieve device measurements (both positions and currents) and wait for the response. When
       * data is received, correct the positions direction with the \p motor_axis_direction.
       * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
       * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
       * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
       * not directly actuated).
       * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
       * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
       * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
       * \param[out] stamp The time stamp of the retrieved measurements (it is set by the process which does the read).
       * \return \p 0 on success.
       * \sa setCommands()
       */
      virtual int getMeasurements(std::vector<double> &positions, std::vector<double> &currents, rclcpp::Time &stamp);

      /**
       * Call the service to retrieve device measurements (positions, currents and commands) and wait for the response.
       * When data is received, correct the positions direction with the \p motor_axis_direction.
       * \param[out] positions The device position vector, expressed in \em ticks: if the device is a \em qbhand only
       * the first element is filled while the others remain always \p 0; in the case of a \em qbmove all the elements
       * contain relevant data, i.e. the positions respectively of \p motor_1, \p motor_2 and \p motor_shaft (which is
       * not directly actuated).
       * \param[out] currents The two-element device motor current vector, expressed in \em mA: if the device is a \em
       * qbhand only the first element is filled while the other remains always \p 0; in the case of a \em qbmove both
       * the elements contain relevant data, i.e. the currents respectively of \p motor_1 and \p motor_2.
       * \param[out] commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only
       * the first element is filled while the other remains always \p 0; in the case of a \em qbmove both the elements
       * contain relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
       * \param[out] stamp The time stamp of the retrieved measurements (it is set by the process which does the read).
       * \return \p 0 on success.
       * \sa setCommands()
       */
      virtual int getMeasurements(std::vector<double> &positions, std::vector<double> &currents, std::vector<double> &commands, rclcpp::Time &stamp);

      /**
       * Call the service to initialize the device with parameters from the Communication Handler and wait for the response.
       * If the initializaation succeed, store the device parameters received, e.g. \p position_limits. It can additionally
       * activate motors during the initialization process, if specified in the Parameter Server.
       * \return \p 0 on success.
       * \sa waitForInitialization()
       */
      virtual int initializeDevice();

      /**
       * Updates \p actuator_interfaces_ state with the measurements of the encoders read from the device
       */
      virtual void readingsToActuatorsPosition(std::vector<double> &actual_positions);

      /**
       * Fill \param[out] commands with the \p actuator_interfaces_ commands correctly composed for the device
       */
      virtual void actuatorsCommandsToWrite(std::vector<double> &commands_to_write);

      //Initialize the services needed for getting/setting cmds from/to the device
      void initializeServicesAndWait();

      //Wait untill all the necessary servers exists
      void waitForSrvs();

      /**
       * Call the service to send reference commands to the device and wait for the response. Before sending the references,
       * correct their direction with the \p motor_axis_direction.
       * \param commands The reference command vector, expressed in \em ticks: if the device is a \em qbhand only the first
       * element is meaningful while the other remains always \p 0; in the case of a \em qbmove both the elements contain
       * relevant data, i.e. the commands respectively of \p motor_1 and \p motor_2.
       * \return \p 0 on success.
       * \sa getMeasurements()
       */
      virtual int setCommands(const std::vector<double> &commands);
      
      /**
       * Get the Measurements from the device, by advertising the server 
       * return \p true if the response of the server is true, \p false otherwise
       */
      bool getMeasurements(float &position, float &velocity, float &current, rclcpp::Time & stamp);
  
      /**
       * Re-subscribe to all the services advertised by the Communication Handler and wait until all the services are
       * properly advertised. Then re-initialize the device parameters (it is assumed that the this method can be called
       * only if the device was previously initialized), unless otherwise specified.
       * \param reinitialize_device If \p true, i.e. by default, reinitialize the device.
       * \sa initializeServicesAndWait(), waitForInitialization(), waitForServices()
       */
      void resetServicesAndWait(const bool &reinitialize_device = true);

      /**
       * Wait until the device is initialized.
       * \sa initializeDevice()
       */
      void waitForInitialization();

      /**
       * Get the list of active controllers, by advertising the server 
       * return \p 0 if the response of the server is true, \p -1 otherwise
       */
      int getActiveControllers(std::vector<std::string> &list_controllers);
      
      /**
       * Calls the service to send \param control_mode to \em qbmove and \em qbclaw devices and wait for the response.
       * \return \p 0 on success.
       */
      int setControlMode(const std::string &control_mode); 

  };

}  // namespace qb_device_ros2_control

#endif  // QB_DEVICE_ROS2_CONTROL_HPP_
