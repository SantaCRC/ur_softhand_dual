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

#include <qbrobotics_research_api/qbsofthand2_research_api.h>

using namespace qbrobotics_research_api;

void qbSoftHand2MotorsResearch::Params::initParams(const std::vector<int8_t> &param_buffer) {
  Device::Params::initParams(param_buffer);
  // Legacy firmware needs inverted signs for second motor encoders (encoder 3 and encoder 4)
  std::for_each(encoder_offsets.begin()+2, encoder_offsets.end(), [](int16_t &x){ x *= -1; } );

  getParameter<uint8_t>(25, param_buffer, hand_side);
}

qbSoftHand2MotorsResearch::qbSoftHand2MotorsResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, true, std::make_unique<qbSoftHand2MotorsResearch::Params>()) {}

qbSoftHand2MotorsResearch::qbSoftHand2MotorsResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::make_unique<qbSoftHand2MotorsResearch::Params>()) {}

/*------ Methods for 6.X.X firmware ------*/
int qbSoftHand2MotorsResearch::setMotorStates(bool motor_state) {
  int set_fail = Device::setMotorStates(motor_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return set_fail;
}

int qbSoftHand2MotorsResearch::setParameter(uint16_t param_type, const std::vector<int8_t> &param_data) {
  int set_fail = Device::setParameter(param_type, param_data);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!set_fail && param_type != 1) {  // WARN: the storeUserDataMemory for ID change is called by the specific method
    set_fail = Device::storeUserDataMemory();  // WARN: a store user memory is required on legacy devices!
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return set_fail;
}

int qbSoftHand2MotorsResearch::setParamHandSide(uint8_t hand_side) {
  int set_fail = setParameter(25, Communication::vectorSwapAndCast<int8_t, uint8_t>({hand_side}));
  return set_fail;
}

int qbSoftHand2MotorsResearch::setParamId(uint8_t id) {  // old method only for legacy devices
  uint8_t previous_id = params_->id;
  int set_fail = Device::setParamId(id);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (!set_fail) {
    params_->id = previous_id;  // WARN: need to send the command to the previous ID on legacy devices!
    set_fail = Device::storeUserDataMemory();  // WARN: a store user memory is required on legacy devices!
    params_->id = id;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return set_fail;
}

int qbSoftHand2MotorsResearch::setParamZeros() {  // old method only for legacy devices
  std::vector<int16_t> positions;
  if (getPositions(positions)) {
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::vector<int16_t> offsets;
  if (getParamEncoderOffsets(offsets)) {
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  std::transform(offsets.begin(), offsets.end(), positions.begin(), offsets.begin(), std::minus<int16_t>());
  return setParamEncoderOffsets(offsets);
}

/*------ Methods for SoftHand 2 ------*/

int qbSoftHand2MotorsResearch::getParamEncoderOffsets() {
  return getParamEncoderOffsets(params_->encoder_offsets);
}

int qbSoftHand2MotorsResearch::getParamEncoderOffsets(std::vector<int16_t> &encoder_offsets) {
  int set_fail = Device::getParamEncoderOffsets(encoder_offsets);
  // Legacy firmware needs inverted signs for second motor encoders (encoder 3 and encoder 4)
  std::for_each( encoder_offsets.begin()+2, encoder_offsets.end(), [](int16_t &x){x *= -1;} );
  return set_fail;
}

int qbSoftHand2MotorsResearch::getParamHandSide() {
    return getParamHandSide(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHand2MotorsResearch::Params>(params_)->hand_side);
}

int qbSoftHand2MotorsResearch::getParamHandSide(uint8_t &hand_side) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(25, param_buffer, hand_side);
  return 0;
}

int qbSoftHand2MotorsResearch::getSynergies(std::vector<int16_t> &synergies) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_SYNERGIES, data_in) < 0) {
    return -1;
  }
  synergies = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int qbSoftHand2MotorsResearch::setParamEncoderOffsets(const std::vector<int16_t> &encoder_offsets) {
  std::vector<int16_t> legacy_offsets = encoder_offsets;
  // Legacy firmware needs inverted signs for second motor encoders (encoder 3 and encoder 4)
  std::for_each( legacy_offsets.begin()+2, legacy_offsets.end(), [](int16_t &x){ x *= -1; } );
  return Device::setParamEncoderOffsets(legacy_offsets);
}

int qbSoftHand2MotorsResearch::setAdditiveSynergiesReferences(int16_t synergy_1, int16_t synergy_2) {
  auto const data_out = Communication::vectorSwapAndCast<int8_t, int16_t>({synergy_1, synergy_2});
  if (communication_->sendCommand(serial_port_, params_->id, CMD_SET_SYNERGIES, data_out) < 0) {
    return -1;
  }
  return 0;
}

int qbSoftHand2MotorsResearch::setMultiplicativeSynergiesReferences(float synergy_1, float synergy_2) {
  if(synergy_1 < 0 || synergy_1 > 1 || synergy_2 < -1 || synergy_2 > 1) {
      return -2;
  }
  int32_t l = getParams()->position_limits[0];
  int32_t L = getParams()->position_limits[1];
  int32_t Lr = (L + l);
  float Sf = (2/(float)Lr);
  float mf = (1/(float)l);
  // multiplicative synergies formulas
  int16_t q1 = (1 + synergy_2)*synergy_1/Sf - synergy_2/mf;
  int16_t q2 = (1 - synergy_2)*synergy_1/Sf + synergy_2/mf;

  std::vector<int16_t> refs{q1, q2};

  return setControlReferences(refs);
}

int qbSoftHand2MotorsResearch::setHomePosition() {
  // Get the params to return to home position
  std::vector<int16_t> synergies;
  int result = getSynergies(synergies);
  std::vector<int32_t> limits;
  getParamPositionLimits(limits);

  if(synergies.size() <= 0 || result < 0 || limits.size() < 4){ // something went wrong
    return result;
  }
  std::vector<std::vector<int>> synergies_references{{limits.at(0), 0}, {synergies.at(0), 0}}; //|synergies references to cancel synergy 1 | synergies references to cancel synergy 2|
  std::vector<int> synergies_to_reach {-1000, 200};

  int synergy_to_cancel = 1;
  while(!synergies_references.empty()) {
      if(setAdditiveSynergiesReferences(synergies_references.back().at(0), synergies_references.back().at(1)) != 0){ // cancel the contribution of synergies
        return -1;
      }
      auto t_start = std::chrono::high_resolution_clock::now();
      auto t_end = t_start;
      double elapsed_time_ms;
      bool condition = true;
      while(condition){
        getSynergies(synergies);
        if(synergy_to_cancel == 1) {
            condition = std::abs(synergies.at(synergy_to_cancel)) > synergies_to_reach.back();
        } else {
            condition = synergies.at(synergy_to_cancel) > synergies_to_reach.back();
        }
        //condition = synergy_to_cancel == 1 ? std::abs(synergies.at(synergy_to_cancel)) > synergies_to_reach.back() : synergies.at(synergy_to_cancel) > synergies_to_reach.back();
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //sleep 1 ms in order to not clog the communication with the device
        t_end = std::chrono::high_resolution_clock::now();
        elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        if(elapsed_time_ms > 2000.0) {
          result = -2;
          break;
        }
      }
      synergies_references.pop_back();
      synergies_to_reach.pop_back();
      synergy_to_cancel --;
  }
  if(setControlReferences({0, 0}) != 0){
      return -1;
  }
  return result;
}

int qbSoftHand2MotorsResearch::setParamSerialNumber(const uint32_t &serial_number) {
  if(!((serial_number & qbhand2_mask_) == qbhand2_mask_)) {
      return -3;
  }
  int set_fail = setParameter(28, Communication::vectorSwapAndCast<int8_t, uint32_t>({serial_number}));
  if (!set_fail) {
    params_->serial_number = "00" + std::to_string(serial_number);
  }
  return set_fail;
}
