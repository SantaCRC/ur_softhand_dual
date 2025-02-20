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

#ifndef QB_HAND_2M_ROS2_CONTROL_HPP_
#define QB_HAND_2M_ROS2_CONTROL_HPP_

#include "qb_device_ros2_control/qb_device_ros2_control.hpp"

namespace qb_hand_ros2_control
{
/**
 * The qbrobotics \em qbhand HardWare interface implements the specific structures to manage the communication with the
 * \em qbhand device. It exploits the features provided by the base device-independent hardware interface and the
 * specific transmission interface.
 * \sa qb_device_ros2_control::DeviceHW
 */
class qbHand2MotorsHW : public qb_device_ros2_control::DeviceHW {
  
  /**
   * Here the function to override, specific for this device.
   */
 private:  
   /**
    * Updates \p actuator_interfaces_ state with the measurements of the encoders read from the device
    */
   void readingsToActuatorsPosition(std::vector<double> &actual_positions) override;

   /**
    * Fill \param[out] commands with the \p actuator_interfaces_ commands correctly composed for the device
    */
   void actuatorsCommandsToWrite(std::vector<double> &commands_to_write) override;
};
typedef std::shared_ptr<qbHand2MotorsHW> qbHand2MotorsHWPtr;

}  // namespace qb_hand_ros2_control

#endif  // QB_HAND_2m_ROS2_CONTROL_HPP_
