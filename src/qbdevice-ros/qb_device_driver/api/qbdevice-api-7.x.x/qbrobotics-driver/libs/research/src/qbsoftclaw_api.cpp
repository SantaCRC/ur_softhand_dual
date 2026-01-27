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
#include <qbrobotics_research_api/qbsoftclaw_api.h>

using namespace qbrobotics_research_api;

qbSoftClaw::qbSoftClaw(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : qbmoveResearch(std::move(communication), std::move(name), std::move(serial_port), id){}

qbSoftClaw::qbSoftClaw(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : qbmoveResearch(std::move(communication), std::move(name), std::move(serial_port), id, init_params) {}

qbSoftClaw::qbSoftClaw(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Device::Params> params)
    : qbmoveResearch(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::move(params)) {}

int16_t qbSoftClaw::convertMillimiters2Tick(const int16_t &mm){
  return (int16_t)(min_allowed_tick_ + (max_allowed_tick_ - min_allowed_tick_)*(mm - max_allowed_mm_)/(min_allowed_mm_ - max_allowed_mm_));
}

int16_t qbSoftClaw::convertTick2Millimiters(const int16_t &tick){
  return (int16_t)(max_allowed_mm_ - (max_allowed_mm_/(max_allowed_tick_ - min_allowed_tick_)*(tick - min_allowed_tick_)));
}

int qbSoftClaw::setControlMode(const controlModes &control_mode){
  switch(control_mode){
    case position_stiffness:
      return setParamControlMode(0);

    case position_current:
      return setParamControlMode(3);

    case deflection:
      return setParamControlMode(4);

    case deflection_current:
      return setParamControlMode(5);

    default:
      return -1;
  }
};
  
int qbSoftClaw::setDeflectionReference(int16_t deflection){

  int16_t deflection_raw;

  //Saturation on the input
  if (deflection > 100) {
    deflection = 100;
  } else if(deflection < -100) {
    deflection = -100;
  }

  if (getParams()->control_mode == 4) { //check if control mode is "deflection"
    deflection_raw = deflection*42; //range is [-2100, 4200]

  } else if (getParams()->control_mode == 5) { //check if control mode is "deflection-current"
    deflection_raw = deflection*90; //range is [-2100, 9000]

  } else {
    return -1;
  }

  if(deflection < 0) {
    deflection_raw = deflection*21; //minimum is the same for both control modes
  }

  return qbmoveResearch::setPositionAndStiffnessReferences(deflection_raw, 0);
};
    
int qbSoftClaw::setPositionAndStiffnessReferences(int16_t position, int16_t stiffness){
  
  if (stiffness > 100) {
    stiffness = 100;
  } else if(stiffness < 0) {
    stiffness = 0;
  }
  stiffness *= 320;//from percentage to tick

  if ((getParams()->control_mode == 0) || (getParams()->control_mode == 3)) { //check if control mode is "position-stiffness" or "position-current"
    return qbmoveResearch::setPositionAndStiffnessReferences(position, stiffness);
  }
  return -1;
};
