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

#include <qbrobotics_research_api/qbrobotics_research_api.h>

namespace qbrobotics_research_api {
  /**
   * @brief SoftHand 2 class that allows to control and get/set parameters.
   * 
   */
  class qbSoftHand2MotorsResearch : public Device {
  public:
    class Params : public Device::Params {
    public:
      Params() = default;
      ~Params() override = default;

      void initParams(const std::vector<int8_t> &param_buffer) override;

      uint8_t hand_side {0};
    };

    explicit qbSoftHand2MotorsResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id);
    explicit qbSoftHand2MotorsResearch(std::shared_ptr<Communication> communication, std::string name, std::string serial_port, uint8_t id, bool init_params);
    ~qbSoftHand2MotorsResearch() override = default;

    int setMotorStates(bool motor_state) override;

    int setParameter(uint16_t param_type, const std::vector<int8_t> &param_data) override;
    int setParamId(uint8_t id) override;
    int setParamZeros() override;


    /**
     * @brief Update the SoftHand 2 hand_side parameter in the class variable \p param_
     *
     * @return 0 on success, -1 on error 
     */
    int getParamHandSide();

    /**
     * @brief Get the hand_side parameter for RIGHT or LEFT Hand for SoftHand 2.
     * 
     * @param[out] hand_side 0 for RIGHT Hand, 1 for LEFT Hand
     * @return 0 on success, -1 on  error
     * 
     */
    int getParamHandSide(uint8_t &hand_side);

    /**
     * @brief Update the SoftHand 2 encoder offsets in the class variable \p param_
     *
     * @return 0 on success, -1 on error 
     */
    int getParamEncoderOffsets() override;

    /**
     * @brief Get the encoder resolution parameters
     *
     * @param[out] encoder_resolutions
     * @return 0 on success, -1 on error 
     */
    int getParamEncoderOffsets(std::vector<int16_t> &encoder_offsets) override;

    /**
     * @brief Get the synergies values of the SoftHand 2. (ADDITIVE SINERGIES)
     *
     * @param[out] synergies
     * @return 0 on success, -1 on error 
     */
    int getSynergies(std::vector<int16_t> &synergies);

    /**
     * @brief Set the encoder offsets parameters of the SoftHand 2
     * 
     * @param encoder_offsets
     * @return 0 on success, < 0 on error  
     * @warning The improper use of this function could damage the device and invalidate the device warranty. Contact our support team (support@qbrobotics.com) for more information.
     */
    int setParamEncoderOffsets(const std::vector<int16_t> &encoder_offsets) override;

    /**
     * @brief Set the Additive Synergies References to SoftHand 2. This command allows to command the SoftHand closure and the its direction of closure. (ADDITIVE SYNERGIES)
     * 
     * @param synergy_1 qbSoftHand 2 closure
     * @param synergy_2 qbSoftHand 2 direction of closure
     * @return 0 on success, -1 on error
     */
    int setAdditiveSynergiesReferences(int16_t synergy_1, int16_t synergy_2);

    /**
     * @brief Set the Multiplicative Synergies References to SoftHand 2. This command allows to command the SoftHand closure and the its direction of closure. (MULTIPLICATIVE SYNERGIES)
     *
     * @param synergy_1 qbSoftHand 2 closure - [0, 1]
     * @param synergy_2 qbSoftHand 2 direction of closure - [-1, 1]
     * @return 0 on success, -1 on error, -2 if limits are violated
     */
    int setMultiplicativeSynergiesReferences(float synergy_1, float synergy_2);

    /**
     * @brief Set the hand_side parameter for RIGHT or LEFT Hand for SoftHand 2.
     * 
     * @param hand_side 0 for RIGHT Hand, 1 for LEFT Hand
     * @return 0 on success, -1 on  error
     * @warning The improper use of this function could damage the device and invalidate the device warranty. Contact our support team (support@qbrobotics.com) for more information.
     */
    int setParamHandSide(uint8_t hand_side);

    /**
     * @brief Blocking function that moves SoftHand 2 motors to home position
     * 
     * @return 0 on success, -1 on communication error, -2 if the SoftHand 2 returns to home position but not within the defined timeframe.
     * 
     */
    int setHomePosition();

    /**
     * @brief Set device serial number.
     * @return 0 on success, < 0  on error.
     * @warning The improper use of this function could damage the device and invalidate the device warranty. Contact our support team (support@qbrobotics.com) for more information.
     */
    int setParamSerialNumber(const uint32_t &serial_number) override;

  private:
    const uint32_t qbhand2_mask_ = 0b00000011100101100000000000000000;
  };
}
