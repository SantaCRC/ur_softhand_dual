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

#ifndef QBROBOTICS_DRIVER_QBMOVE_RESEARCH_API_H
#define QBROBOTICS_DRIVER_QBMOVE_RESEARCH_API_H

#include <qbrobotics_research_api/qbrobotics_research_api.h>

namespace qbrobotics_research_api {

class qbmoveResearch : public Device {
 public:
  class Params : public Device::Params {
   public:
    Params() = default;
    ~Params() override = default;

    void initParams(const std::vector<int8_t> &param_buffer) override;
  };

  explicit qbmoveResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id);
  explicit qbmoveResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params);
  explicit qbmoveResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params, std::unique_ptr<Device::Params> params);
  ~qbmoveResearch() override = default;

  int computeAndStoreMaximumStiffness();
  virtual int setPositionAndStiffnessReferences(int16_t position, int16_t stiffness);

  int getParamRateLimiter();
  int getParamRateLimiter(uint8_t &rate_limiter);

  int setParamRateLimiter(uint8_t rate_limiter);

  int setParamSerialNumber(const uint32_t &serial_number) override;

  protected:
    const uint32_t qbmove_mask_ = 0b00000000100110000000000000000000;
    const uint32_t claw_mask_   = 0b00000011101000100000000000000000;
};

class qbmoveLegacyResearch : public qbmoveResearch {
 public:
  class Params : public qbmoveResearch::Params {
   public:
    Params() = default;
    ~Params() override = default;

    void initParams(const std::vector<int8_t> &param_buffer) override;
  };

  explicit qbmoveLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id);
  explicit qbmoveLegacyResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params);
  ~qbmoveLegacyResearch() override = default;

  int getControlReferences(std::vector<int16_t> &control_references) override;
  int setMotorStates(bool motor_state) override;

  int getParamEncoderOffsets() override;
  int getParamEncoderOffsets(std::vector<int16_t> &encoder_offsets) override;

  int setParameter(uint16_t param_type, const std::vector<int8_t> &param_data) override;
  int setParamId(uint8_t id) override;
  int setParamEncoderOffsets(const std::vector<int16_t> &encoder_offsets) override;
  int setParamZeros() override;
};

}  // namespace qbrobotics_research_api

#endif  // QBROBOTICS_DRIVER_QBMOVE_RESEARCH_API_H
