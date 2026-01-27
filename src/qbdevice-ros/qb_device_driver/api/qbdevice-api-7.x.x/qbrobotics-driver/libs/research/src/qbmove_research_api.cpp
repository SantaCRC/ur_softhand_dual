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

#include <qbrobotics_research_api/qbmove_research_api.h>

using namespace qbrobotics_research_api;

void qbmoveResearch::Params::initParams(const std::vector<int8_t> &param_buffer) {
  Device::Params::initParams(param_buffer);
  getParameter<uint8_t>(14, param_buffer, rate_limiter);
}

qbmoveResearch::qbmoveResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, true, std::make_unique<qbmoveResearch::Params>()){}

qbmoveResearch::qbmoveResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::make_unique<qbmoveResearch::Params>()) {}

qbmoveResearch::qbmoveResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Device::Params> params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::move(params)) {}

int qbmoveResearch::computeAndStoreMaximumStiffness() {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_CALIBRATE, data_in) < 0) {
    return -1;
  }
  return 0;
}

int qbmoveResearch::setPositionAndStiffnessReferences(int16_t position, int16_t stiffness) {
  auto const data_out = Communication::vectorSwapAndCast<int8_t, int16_t>({position, stiffness});
  if (communication_->sendCommand(serial_port_, params_->id, CMD_SET_POS_STIFF, data_out) < 0) {
    return -1;
  }
  return 0;
}

int qbmoveResearch::getParamRateLimiter() {
  return getParamRateLimiter(params_->rate_limiter);
}

int qbmoveResearch::getParamRateLimiter(uint8_t &rate_limiter) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(14, param_buffer, rate_limiter);
  return 0;
}

int qbmoveResearch::setParamRateLimiter(uint8_t rate_limiter) {
  int set_fail = setParameter(14, Communication::vectorSwapAndCast<int8_t, uint8_t>({rate_limiter}));
  if (!set_fail) {
    params_->rate_limiter = rate_limiter;
  }
  return set_fail;
}

int qbmoveResearch::setParamSerialNumber(const uint32_t &serial_number) {
  if(!((serial_number & qbmove_mask_) == qbmove_mask_ || (serial_number & claw_mask_) == claw_mask_)) {
      return -3;
  }
  int set_fail = setParameter(15, Communication::vectorSwapAndCast<int8_t, uint32_t>({serial_number}));
  if (!set_fail) {
    params_->serial_number = "00" + std::to_string(serial_number);
  }
  return set_fail;
}

// ----------------------------------------------------------------


void qbmoveLegacyResearch::Params::initParams(const std::vector<int8_t> &param_buffer) {
  qbmoveResearch::Params::initParams(param_buffer);
  // Legacy firmware needs inverted signs
  std::for_each(encoder_offsets.begin(), encoder_offsets.end(), [](int16_t &x){ x *= -1; } );
}

qbmoveLegacyResearch::qbmoveLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : qbmoveResearch(std::move(communication), std::move(name), std::move(serial_port), id, true, std::make_unique<qbmoveLegacyResearch::Params>()) {}

qbmoveLegacyResearch::qbmoveLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : qbmoveResearch(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::make_unique<qbmoveLegacyResearch::Params>()) {}

int qbmoveLegacyResearch::setParameter(uint16_t param_type, const std::vector<int8_t> &param_data) {
  int set_fail = qbmoveResearch::setParameter(param_type, param_data);
  if (!set_fail && param_type != 1) {  // WARN: the storeUserDataMemory for ID change is called by the specific method
    set_fail = qbmoveResearch::storeUserDataMemory();  // WARN: a store user memory is required on legacy devices!
  }
  return set_fail;
}

int qbmoveLegacyResearch::getControlReferences(std::vector<int16_t> &control_references) {
  int set_fail = qbmoveResearch::getControlReferences(control_references);
  // Legacy firmware needs inverted signs
  std::for_each( control_references.begin(), control_references.end(), [](int16_t &x){ x *= -1; } );
  return set_fail;
}

int qbmoveLegacyResearch::setMotorStates(bool motor_state) {
  int set_fail = qbmoveResearch::setMotorStates(motor_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  return set_fail;
}

int qbmoveLegacyResearch::getParamEncoderOffsets() {
  return getParamEncoderOffsets(params_->encoder_offsets);
}

int qbmoveLegacyResearch::getParamEncoderOffsets(std::vector<int16_t> &encoder_offsets) {
  int set_fail = qbmoveResearch::getParamEncoderOffsets(encoder_offsets);
  // Legacy firmware needs inverted signs
  std::for_each( encoder_offsets.begin(), encoder_offsets.end(), [](int16_t &x){ x *= -1; } );
  return set_fail;
}

int qbmoveLegacyResearch::setParamId(uint8_t id) {  // old method only for legacy devices
  uint8_t previous_id = params_->id;
  int set_fail = qbmoveResearch::setParamId(id);
  if (!set_fail) {
    params_->id = previous_id;  // WARN: need to send the command to the previous ID on legacy devices!
    set_fail = qbmoveResearch::storeUserDataMemory();  // WARN: a store user memory is required on legacy devices!
    params_->id = id;
  }
  return set_fail;
}

int qbmoveLegacyResearch::setParamEncoderOffsets(const std::vector<int16_t> &encoder_offsets) {
  std::vector<int16_t> legacy_offsets = encoder_offsets;
  // Legacy firmware needs inverted signs
  legacy_offsets.at(2) *= -1;
  return qbmoveResearch::setParamEncoderOffsets(legacy_offsets);
}

int qbmoveLegacyResearch::setParamZeros() {  // old method only for legacy devices
  std::vector<int16_t> positions;
  if (getPositions(positions)) {
    return -1;
  }
  std::vector<int16_t> offsets;
  if (getParamEncoderOffsets(offsets)) {
    return -1;
  }

  std::transform(offsets.begin(), offsets.end(), positions.begin(), offsets.begin(), std::plus<int16_t>());  // inverted signs for legacy leads to "plus" operation

  return setParamEncoderOffsets(offsets);
}
