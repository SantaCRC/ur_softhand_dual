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

#ifndef QBROBOTICS_DRIVER_QBSOFTCLAW_API_H
#define QBROBOTICS_DRIVER_QBSOFTCLAW_API_H

#include <qbrobotics_research_api/qbmove_research_api.h>

namespace qbrobotics_research_api {

const float max_allowed_mm_ = 110;
const float min_allowed_mm_ = 0;
const float max_allowed_tick_ = 4500;
const float min_allowed_tick_ = -2200;


class qbSoftClaw : public qbmoveResearch {
 public:
  
  explicit qbSoftClaw(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id);
  explicit qbSoftClaw(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params);
  explicit qbSoftClaw(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Device::Params> params);

  ~qbSoftClaw() override = default;


  enum controlModes {

    position_stiffness,
    position_current,
    deflection,
    deflection_current

  };

  /**
   * @brief Conversion [mm] to device unit [tick]
   * 
   * @param mm 
   * @return int16_t tick 
   */
  int16_t convertMillimiters2Tick(const int16_t &mm);

  /**
   * @brief Conversion device unit [tick] to [mm]
   * 
   * @param tick 
   * @return int16_t millimiters
   */
  int16_t convertTick2Millimiters(const int16_t &tick);

  /**
   * @brief Set the Control Mode as one of the following: 
   *  \p qbrobotics_research_api::qbSoftClaw::controlModes::position_stiffness ;
   *  \p qbrobotics_research_api::qbSoftClaw::controlModes::deflection or
   *  \p qbrobotics_research_api::qbSoftClaw::controlModes::deflection_current
   * 
   * @param control_mode 
   * @return 0 on success, < 0 on error
   */
  int setControlMode(const controlModes &control_mode);
  
  /**
   * @brief Set the Deflection Reference value in percentage ([0,100] for closure; [-1,-100] for opening). 
   * Use this while the deflection control is active
   * 
   * @param deflection in percentage
   * @return 0 on success, < 0 on error
   */
  int setDeflectionReference(int16_t deflection);
    
  /**
   * @brief Set the Position And Stiffness References values for the claw while the position-stiffness control is active
   * 
   * @param position in tick (device unit) 
   * @param stiffness in percentage
   * @return 0 on success, < 0 on error
   */
  int setPositionAndStiffnessReferences(int16_t position, int16_t stiffness) override;

};

}  // namespace qbrobotics_research_api

#endif  // QBROBOTICS_DRIVER_QBSOFTCLAW_API_H
