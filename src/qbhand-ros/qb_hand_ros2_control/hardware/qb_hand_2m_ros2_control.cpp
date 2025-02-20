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

#include "qb_hand_ros2_control/qb_hand_2m_ros2_control.hpp"

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

namespace qb_hand_ros2_control
{

void qbHand2MotorsHW::readingsToActuatorsPosition(std::vector<double> &actual_positions){
  if(actual_positions.size()>=3 && actuator_interfaces_.size()>=4){
    actuator_interfaces_.at(0).state_ =  (actual_positions.at(0) + actual_positions.at(2))/ 2; // 1st Synergy Joint
    actuator_interfaces_.at(1).state_ = (actual_positions.at(0) - actual_positions.at(2))* info_.transmissions.at(1).joints[0].mechanical_reduction/(actual_positions.at(0) +
     actual_positions.at(2) + 2 * info_.transmissions.at(1).joints[0].mechanical_reduction); //  2nd Synergy Joint
    actuator_interfaces_.at(2).state_ = actual_positions.at(0); // Motor 1 Joint
    actuator_interfaces_.at(3).state_ = actual_positions.at(2); // Motor 2 Joint
    return;
  }
  RCLCPP_ERROR_STREAM(*logger_, "[qbHand2MotorsHW] Error: interfaces size mismatch!");
}

void qbHand2MotorsHW::actuatorsCommandsToWrite(std::vector<double> &commands_to_write){
  
  if(commands_to_write.size()>=2 && actuator_interfaces_.size()>=2){
    std::vector<std::string> list_controllers;
    this->getActiveControllers(list_controllers);
    bool flag = false; 
    
    for(auto name : list_controllers){
      flag = flag || (name.find("synergies") != std::string::npos); // Check if we are commanding Synergies
    }

    if(flag){
      commands_to_write.at(0) =  actuator_interfaces_.at(0).command_ + actuator_interfaces_.at(1).command_*(actuator_interfaces_.at(0).command_/info_.transmissions.at(1).joints[0].mechanical_reduction+1); // 1st Synergy Joint
      commands_to_write.at(1) = actuator_interfaces_.at(0).command_ - actuator_interfaces_.at(1).command_*(actuator_interfaces_.at(0).command_/info_.transmissions.at(1).joints[0].mechanical_reduction+1); //  2nd Synergy Joint
    } else {
      commands_to_write.at(0) =  actuator_interfaces_.at(2).command_; //Motor1 Joint
      commands_to_write.at(1) = actuator_interfaces_.at(3).command_; //Motor2 Joint
    }
    return;
  }
  RCLCPP_ERROR_STREAM(*logger_, "[qbHand2MotorsHW] Error: interfaces size mismatch!");
}

}  // namespace qb_hand_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  qb_hand_ros2_control::qbHand2MotorsHW,
  hardware_interface::SystemInterface)