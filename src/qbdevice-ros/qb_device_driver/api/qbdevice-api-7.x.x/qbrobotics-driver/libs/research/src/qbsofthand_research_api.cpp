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

#include <qbrobotics_research_api/qbsofthand_research_api.h>

using namespace qbrobotics_research_api;

void qbSoftHandResearch::Params::initParams(const std::vector<int8_t> &param_buffer) {
  Device::Params::initParams(param_buffer);
  getParameter<uint8_t>(14, param_buffer, use_emg_calibration);
  getParameter<uint16_t>(15, param_buffer, emg_thresholds);
  getParameter<uint32_t>(16, param_buffer, emg_max_value);
  getParameter<uint8_t>(17, param_buffer, emg_speed);
  getParameter<uint8_t>(18, param_buffer, use_double_encoder);
  getParameter<int8_t>(19, param_buffer, handle_ratio);
  getParameter<uint8_t>(20, param_buffer, use_pwm_rescaling);
  getParameter<float>(21, param_buffer, current_lookup_table);
  getParameter<uint8_t>(22, param_buffer, rate_limiter);
}

qbSoftHandResearch::qbSoftHandResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, true, std::make_unique<qbSoftHandResearch::Params>()) {}

qbSoftHandResearch::qbSoftHandResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::make_unique<qbSoftHandResearch::Params>()) {}

qbSoftHandResearch::qbSoftHandResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Device::Params> params)
    : Device(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::move(params)) {}

int qbSoftHandResearch::openCloseCycles(uint16_t speed, uint16_t cycles) {
  std::vector<int8_t> data_in;
  const std::vector<int8_t> data_out = Communication::vectorSwapAndCast<int8_t, uint16_t>({speed, cycles});
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_HAND_CALIBRATE, data_out, data_in) < 0) {
    return -1;
  }
  return 0;
}

int qbSoftHandResearch::getEMGs(std::vector<int16_t> &emg_values) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_EMG, data_in) < 0) {
    return -1;
  }
  emg_values = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int qbSoftHandResearch::getJoystick(std::vector<int16_t> &joystick_values) {
  std::vector<int8_t> data_in;
  if (communication_->sendCommandAndParse(serial_port_, params_->id, CMD_GET_JOYSTICK, data_in) < 0) {
    return -1;
  }
  joystick_values = Communication::vectorCastAndSwap<int16_t>(data_in);
  return 0;
}

int qbSoftHandResearch::getParamUseEMGCalibration() {
  return getParamUseEMGCalibration(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->use_emg_calibration);
}

int qbSoftHandResearch::getParamUseEMGCalibration(uint8_t &use_emg_calibration) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(14, param_buffer, use_emg_calibration);
  return 0;
}

int qbSoftHandResearch::getParamEMGThresholds() {
  return getParamEMGThresholds(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->emg_thresholds);
}

int qbSoftHandResearch::getParamEMGThresholds(std::vector<uint16_t> &emg_thresholds) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint16_t>(15, param_buffer, emg_thresholds);
  return 0;
}

int qbSoftHandResearch::getParamEMGMaxValues() {
  return getParamEMGMaxValues(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->emg_max_value);
}

int qbSoftHandResearch::getParamEMGMaxValues(std::vector<uint32_t> &emg_max_value) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint32_t>(16, param_buffer, emg_max_value);
  return 0;
}

int qbSoftHandResearch::getParamEMGSpeed() {
  return getParamEMGSpeed(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->emg_speed);
}

int qbSoftHandResearch::getParamEMGSpeed(uint8_t &emg_speed) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(17, param_buffer, emg_speed);
  return 0;
}

int qbSoftHandResearch::getParamUseDoubleEncoder() {
  return getParamUseDoubleEncoder(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->use_double_encoder);
}

int qbSoftHandResearch::getParamUseDoubleEncoder(uint8_t &use_double_encoder) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(18, param_buffer, use_double_encoder);
  return 0;
}

int qbSoftHandResearch::getParamHandleRatio() {
  return getParamHandleRatio(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->handle_ratio);
}

int qbSoftHandResearch::getParamHandleRatio(int8_t &handle_ratio) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<int8_t>(19, param_buffer, handle_ratio);
  return 0;
}

int qbSoftHandResearch::getParamUsePWMRescaling() {
  return getParamUsePWMRescaling(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->use_pwm_rescaling);
}

int qbSoftHandResearch::getParamUsePWMRescaling(uint8_t &use_pwm_rescaling) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(20, param_buffer, use_pwm_rescaling);
  return 0;
}

int qbSoftHandResearch::getParamCurrentLookupTable() {
  return getParamCurrentLookupTable(std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->current_lookup_table);
}

int qbSoftHandResearch::getParamCurrentLookupTable(std::vector<float> &current_lookup_table) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<float>(21, param_buffer, current_lookup_table);
  return 0;
}

int qbSoftHandResearch::getParamRateLimiter() {
  return getParamRateLimiter(params_->rate_limiter);
}

int qbSoftHandResearch::getParamRateLimiter(uint8_t &rate_limiter) {
  std::vector<int8_t> param_buffer;
  if (getParameters(param_buffer) != 0) {
    return -1;
  }
  Params::getParameter<uint8_t>(22, param_buffer, rate_limiter);
  return 0;
}

int qbSoftHandResearch::setParamUseEMGCalibration(uint8_t use_emg_calibration) {
  int set_fail = setParameter(14, Communication::vectorSwapAndCast<int8_t, uint8_t>({use_emg_calibration}));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->use_emg_calibration = use_emg_calibration;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamEMGThresholds(const std::vector<uint16_t> &emg_thresholds) {
  int set_fail = setParameter(15, Communication::vectorSwapAndCast<int8_t, uint16_t>(emg_thresholds));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->emg_thresholds = emg_thresholds;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamEMGMaxValues(const std::vector<uint32_t> &emg_max_value) {
  int set_fail = setParameter(16, Communication::vectorSwapAndCast<int8_t, uint32_t>(emg_max_value));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->emg_max_value = emg_max_value;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamEMGSpeed(uint8_t emg_speed) {
  int set_fail = setParameter(17, Communication::vectorSwapAndCast<int8_t, uint8_t>({emg_speed}));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->emg_speed = emg_speed;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamUseDoubleEncoder(uint8_t use_double_encoder) {
  int set_fail = setParameter(18, Communication::vectorSwapAndCast<int8_t, uint8_t>({use_double_encoder}));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->use_double_encoder = use_double_encoder;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamHandleRatio(int8_t handle_ratio) {
  int set_fail = setParameter(19, Communication::vectorSwapAndCast<int8_t, int8_t>({handle_ratio}));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->handle_ratio = handle_ratio;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamUsePWMRescaling(uint8_t use_pwm_rescaling) {
  int set_fail = setParameter(20, Communication::vectorSwapAndCast<int8_t, uint8_t>({use_pwm_rescaling}));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->use_pwm_rescaling = use_pwm_rescaling;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamCurrentLookupTable(const std::vector<float> &current_lookup_table) {
  int set_fail = setParameter(21, Communication::vectorSwapAndCast<int8_t, float>(current_lookup_table));
  if (!set_fail) {
    std::dynamic_pointer_cast<qbrobotics_research_api::qbSoftHandResearch::Params>(params_)->current_lookup_table = current_lookup_table;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamRateLimiter(uint8_t rate_limiter) {
  int set_fail = setParameter(22, Communication::vectorSwapAndCast<int8_t, uint8_t>({rate_limiter}));
  if (!set_fail) {
    params_->rate_limiter = rate_limiter;
  }
  return set_fail;
}

int qbSoftHandResearch::setParamSerialNumber(const uint32_t &serial_number) {
  if(!((serial_number & qbhand_mask_) == qbhand_mask_)) {
      return -3;
  }
  int set_fail = setParameter(23, Communication::vectorSwapAndCast<int8_t, uint32_t>({serial_number}));
  if (!set_fail) {
    params_->serial_number = "00" + std::to_string(serial_number);
  }
  return set_fail;
}


// ----------------------------------------------------------------


qbSoftHandLegacyResearch::qbSoftHandLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id)
    : qbSoftHandResearch(std::move(communication), std::move(name), std::move(serial_port), id, true, std::make_unique<qbSoftHandLegacyResearch::Params>()) {}

qbSoftHandLegacyResearch::qbSoftHandLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params)
    : qbSoftHandResearch(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::make_unique<qbSoftHandLegacyResearch::Params>()) {}

qbSoftHandLegacyResearch::qbSoftHandLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Device::Params> params)
    : qbSoftHandResearch(std::move(communication), std::move(name), std::move(serial_port), id, init_params, std::move(params)) {}

int qbSoftHandLegacyResearch::setMotorStates(bool motor_state) {
  int set_fail = qbSoftHandResearch::setMotorStates(motor_state);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return set_fail;
}

int qbSoftHandLegacyResearch::setParameter(uint16_t param_type, const std::vector<int8_t> &param_data) {
  int set_fail = qbSoftHandResearch::setParameter(param_type, param_data);
  if (!set_fail && param_type != 1) {  // WARN: the storeUserDataMemory for ID change is called by the specific method
    set_fail = qbSoftHandResearch::storeUserDataMemory();  // WARN: a store user memory is required on legacy devices!
  }
  return set_fail;
}

int qbSoftHandLegacyResearch::setParamId(uint8_t id) {  // old method only for legacy devices
  uint8_t previous_id = params_->id;
  int set_fail = qbSoftHandResearch::setParamId(id);
  if (!set_fail) {
    params_->id = previous_id;  // WARN: need to send the command to the previous ID on legacy devices!
    set_fail = qbSoftHandResearch::storeUserDataMemory();  // WARN: a store user memory is required on legacy devices!
    params_->id = id;
  }
  return set_fail;
}

int qbSoftHandLegacyResearch::setParamZeros() {  // old method only for legacy devices
  std::vector<int16_t> positions;
  if (getPositions(positions)) {
    return -1;
  }
  std::vector<int16_t> offsets;
  if (getParamEncoderOffsets(offsets)) {
    return -1;
  }
  std::transform(offsets.begin(), offsets.end(), positions.begin(), offsets.begin(), std::minus<int16_t>());
  return setParamEncoderOffsets(offsets);
}
