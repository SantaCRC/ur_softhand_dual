/***
 *  Software License Agreement: BSD 3-Clause License
 *
 *  Copyright (c) 2015-2020, qbrobotics®
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
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <thread>

namespace qbrobotics_research_api {

class Communication : public ::testing::Test {
 protected:
  void SetUp() override {
    handler_ = std::make_shared<qbroboticsResearchAPI::Communication>();
    int new_serial_ports_retrieved = handler_->listSerialPorts(serial_ports_);
    if (new_serial_ports_retrieved < 0) {
      ASSERT_EQ(serial_ports_.size(), 0);
    }
    ASSERT_EQ(serial_ports_.size(), new_serial_ports_retrieved);  // there was no previous data stored

    for (auto const &serial_port : serial_ports_) {
      std::vector<uint8_t> device_ids;
      int device_ids_retrieved = handler_->listConnectedDevices(serial_port.serial_port, device_ids);
      if (device_ids_retrieved < 0) {
        ASSERT_EQ(device_ids.size(), 0);
      }
      ASSERT_EQ(device_ids.size(), device_ids_retrieved);

      std::set<uint8_t> device_ids_set;
      for (auto const &id : device_ids) {
        ASSERT_GT(id, 0);
        ASSERT_LT(id, 128);
        device_ids_set.insert(id);
      }
      ASSERT_EQ(device_ids_set.size(), device_ids.size());  // check uniqueness
      serial_ports_ids_[serial_port.serial_port] = device_ids_set;

      for (auto const &id : device_ids) {
        std::string device_name = serial_port.serial_port + ":" + std::to_string(id);
        devices_.push_back(std::make_unique<qbroboticsResearchAPI::Device>(handler_, device_name, serial_port.serial_port, id));
        ASSERT_EQ(devices_.back()->params_->id, id);  // parameters are automatically retrieved and stored in `device_[device_name]->params_`
        devices_not_init_.push_back(std::make_unique<qbroboticsResearchAPI::Device>(handler_, device_name, serial_port.serial_port, id, false));
        ASSERT_EQ(devices_not_init_.back()->params_->id, id);  // even if params are not initialized, the id is always stored correctly
        //TODO: add tests to show that `devices_not_init_` works as fine as `devices_`
      }

      for (uint8_t id=127; !fake_device_ && id>0; id--) {  // only one serial_port:fake_id couple is initialized
        if (!device_ids_set.count(id)) {  // not connected device
          fake_device_ = std::make_unique<qbroboticsResearchAPI::Device>(handler_, "fake_dev", serial_port.serial_port, id, false);
        }
      }
    }
  }

  void TearDown() override {}

  std::vector<serial::PortInfo> serial_ports_;
  std::map<std::string, std::set<uint8_t>> serial_ports_ids_;
  std::shared_ptr<qbroboticsResearchAPI::Communication> handler_;
  std::vector<std::unique_ptr<qbroboticsResearchAPI::Device>> devices_;
  std::vector<std::unique_ptr<qbroboticsResearchAPI::Device>> devices_not_init_;
  std::unique_ptr<qbroboticsResearchAPI::Device> fake_device_;
};

TEST_F(Communication, ListSerialPorts) {
  for (auto const &serial_port : serial_ports_) {
    EXPECT_GT(serial_port.id_product, 0);
    EXPECT_GT(serial_port.id_vendor, 0);
    EXPECT_EQ(serial_port.manufacturer, "QB Robotics");
    EXPECT_THAT(serial_port.product, ::testing::EndsWith(serial_port.serial_number));
#if !defined(_WIN32)
    EXPECT_THAT(serial_port.product, ::testing::MatchesRegex("[A-Z]+ [0-9]+"));
    EXPECT_THAT(serial_port.serial_number, ::testing::MatchesRegex("[0-9]+"));
    EXPECT_THAT(serial_port.serial_port, ::testing::MatchesRegex("^/dev/([^/]+)/?$"));
#else
    // MSVC does not support [] and {} in gtest
    EXPECT_THAT(serial_port.product, ::testing::MatchesRegex("^\\w+ \\d+$"));
    EXPECT_THAT(serial_port.serial_number, ::testing::MatchesRegex("\\d+"));
    EXPECT_THAT(serial_port.serial_port, ::testing::MatchesRegex("^COM\\d+$"));
#endif
  }
}

TEST_F(Communication, Ping) {
  for (auto const &device : devices_) {
    EXPECT_EQ(device->ping(), 0);
  }
  EXPECT_EQ(fake_device_->ping(), -1);
}

TEST_F(Communication, GetInfo) {
  for (auto const &device : devices_) {
    std::string info;
    EXPECT_EQ(device->getInfo(0, info), 0);
    EXPECT_THAT(info, ::testing::ContainsRegex("Firmware version:.*v?([0-9]+(\\.)){3}"));
  }
  std::string info;
  EXPECT_EQ(fake_device_->getInfo(0, info), -1);
}

TEST_F(Communication, SensorMeasurements) {
  for (auto const &device : devices_) {
    std::vector<int16_t> positions;
    EXPECT_EQ(device->getPositions(positions), 0);
    EXPECT_EQ(positions.size(), 3);
    EXPECT_EQ(positions.at(0), 4094);  //FIXME: -4098 after meminit in some cases on SoftHand
    EXPECT_EQ(positions.at(1), 4094);
    EXPECT_EQ(positions.at(2), 4094);

    std::vector<int16_t> velocities;
    EXPECT_EQ(device->getVelocities(velocities), 0);
    EXPECT_EQ(velocities.size(), 3);
    EXPECT_EQ(velocities.at(0), 0);
    EXPECT_EQ(velocities.at(1), 0);
    EXPECT_EQ(velocities.at(2), 0);

    std::vector<int16_t> accelerations;
    EXPECT_EQ(device->getAccelerations(accelerations), 0);
    EXPECT_EQ(accelerations.size(), 3);
    EXPECT_EQ(accelerations.at(0), 0);
    EXPECT_EQ(accelerations.at(1), 0);
    EXPECT_EQ(accelerations.at(2), 0);

    std::vector<int16_t> currents;
    EXPECT_EQ(device->getCurrents(currents), 0);
    EXPECT_EQ(currents.size(), 2);
    EXPECT_EQ(currents.at(0), 0);
    EXPECT_EQ(currents.at(1), 0);

    EXPECT_EQ(device->getCurrentsAndPositions(currents, positions), 0);
    EXPECT_EQ(currents.size(), 2);
    EXPECT_EQ(currents.at(0), 0);
    EXPECT_EQ(currents.at(1), 0);
    EXPECT_EQ(positions.size(), 3);
    EXPECT_EQ(positions.at(0), 4094);  //FIXME: -4098 after meminit in some cases on SoftHand
    EXPECT_EQ(positions.at(1), 127);  //FIXME: [[firmware bug]] (-129 or -29825 after meminit in some cases)
    EXPECT_EQ(positions.at(2), 4094);
  }
  std::vector<int16_t> fake_data;
  EXPECT_EQ(fake_device_->getPositions(fake_data), -1);
  EXPECT_EQ(fake_device_->getVelocities(fake_data), -1);
  EXPECT_EQ(fake_device_->getAccelerations(fake_data), -1);
  EXPECT_EQ(fake_device_->getCurrents(fake_data), -1);
  EXPECT_EQ(fake_device_->getCurrentsAndPositions(fake_data, fake_data), -1);
}

TEST_F(Communication, ControlReferences) {
  for (auto const &device : devices_) {
    bool motor_state = true;
    EXPECT_EQ(device->getMotorStates(motor_state), 0);
    ASSERT_EQ(motor_state, false);  // skip if motor is active

    std::vector<int16_t> control_references;
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), 4094);  //FIXME: 8188 or 14431 after meminit in some cases on SoftHand
    EXPECT_EQ(control_references.at(1), 0);  // SoftHand
    control_references.at(0) = 12345;
    control_references.at(1) = 3210;
    EXPECT_EQ(device->setControlReferences(control_references), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    control_references.clear();
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), 12345);
    EXPECT_EQ(control_references.at(1), 3210);
    control_references.at(0) = 0;
    control_references.at(1) = 0;
    EXPECT_EQ(device->setControlReferences(control_references), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    control_references.clear();
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), 0);
    EXPECT_EQ(control_references.at(1), 0);
  }
  std::vector<int16_t> fake_data;
  EXPECT_EQ(fake_device_->getControlReferences(fake_data), -1);
  // cannot test setControlReferences failure on fake_device_ because there is no acknowledge
}

TEST_F(Communication, MotorState) {
  for (auto const &device : devices_) {
    bool motor_state = true;
    EXPECT_EQ(device->getMotorStates(motor_state), 0);
    EXPECT_EQ(motor_state, false);

    motor_state = true;
    EXPECT_EQ(device->setMotorStates(motor_state), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    motor_state = false;
    EXPECT_EQ(device->getMotorStates(motor_state), 0);
    EXPECT_EQ(motor_state, true);
    motor_state = false;
    EXPECT_EQ(device->setMotorStates(motor_state), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    motor_state = true;
    EXPECT_EQ(device->getMotorStates(motor_state), 0);
    EXPECT_EQ(motor_state, false);

    std::vector<int16_t> control_references;
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), 4094);  //FIXME: 0 after meminit in some cases on SoftHand
    EXPECT_EQ(control_references.at(1), 4094);
    control_references.at(0) = 4094;
    control_references.at(1) = 0;
    EXPECT_EQ(device->setControlReferences(control_references), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  bool fake_data;
  EXPECT_EQ(fake_device_->getMotorStates(fake_data), -1);
  // cannot test setMotorStates failure on fake_device_ because there is no acknowledge
}

TEST_F(Communication, DefaultParams) {
  for (auto const &device : devices_) {
    EXPECT_EQ(device->defaultParamsToMemoryAndArchive(), 0);

    EXPECT_EQ(device->getParamId(device->params_->id), 0);
    EXPECT_EQ(device->params_->id, 1);
    EXPECT_EQ(device->getParamPositionPID(device->params_->position_pid), 0);
    EXPECT_NEAR(device->params_->position_pid.at(0), 0.04, 0.0001);
    EXPECT_NEAR(device->params_->position_pid.at(1), 0, 0.0001);
    EXPECT_NEAR(device->params_->position_pid.at(2), 0.06, 0.0001);
    EXPECT_EQ(device->getParamCurrentPID(device->params_->current_pid), 0);
    EXPECT_NEAR(device->params_->current_pid.at(0), 1.0, 0.0001);
    EXPECT_NEAR(device->params_->current_pid.at(1), 0.001, 0.0001);
    EXPECT_NEAR(device->params_->current_pid.at(2), 0, 0.0001);
    EXPECT_EQ(device->getParamStartupActivation(device->params_->startup_activation), 0);
    EXPECT_EQ(device->params_->startup_activation, false);
    EXPECT_EQ(device->getParamInputMode(device->params_->input_mode), 0);
    EXPECT_EQ(device->params_->input_mode, 0);
    EXPECT_EQ(device->getParamControlMode(device->params_->control_mode), 0);
    EXPECT_EQ(device->params_->control_mode, 0);
    EXPECT_EQ(device->getParamEncoderResolutions(device->params_->encoder_resolutions), 0);
    EXPECT_EQ(device->params_->encoder_resolutions.at(0), 3);
    EXPECT_EQ(device->params_->encoder_resolutions.at(1), 3);
    EXPECT_EQ(device->params_->encoder_resolutions.at(2), 3);
    EXPECT_EQ(device->getParamEncoderOffsets(device->params_->encoder_offsets), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(0), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(1), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(2), 0);
    EXPECT_EQ(device->getParamEncoderMultipliers(device->params_->encoder_multipliers), 0);
    EXPECT_EQ(device->params_->encoder_multipliers.at(0), 1);
    EXPECT_EQ(device->params_->encoder_multipliers.at(1), 1);
    EXPECT_EQ(device->params_->encoder_multipliers.at(2), 1);
    EXPECT_EQ(device->getParamUsePositionLimits(device->params_->use_position_limits), 0);
    EXPECT_EQ(device->params_->use_position_limits, true);
    EXPECT_EQ(device->getParamPositionLimits(device->params_->position_limits), 0);
    EXPECT_EQ(device->params_->position_limits.at(0), 0);
    EXPECT_EQ(device->params_->position_limits.at(1), 19000);
    EXPECT_EQ(device->getParamPositionMaxSteps(device->params_->position_max_steps), 0);
    EXPECT_EQ(device->params_->position_max_steps.at(0), 0);
    EXPECT_EQ(device->getParamCurrentLimit(device->params_->current_limit), 0);
    EXPECT_EQ(device->params_->current_limit, 750);

    std::vector<int16_t> control_references, positions;
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), 4094);  //FIXME: 8188 after meminit in some cases on SoftHand
    EXPECT_EQ(control_references.at(1), 0);  // SoftHand
    EXPECT_EQ(device->getPositions(positions), 0);
    EXPECT_EQ(positions.size(), 3);
    EXPECT_EQ(positions.at(0), 4094);  //FIXME: -4098 after meminit in some cases on SoftHand
    EXPECT_EQ(positions.at(1), 4094);
    EXPECT_EQ(positions.at(2), 4094);
  }
  EXPECT_EQ(fake_device_->defaultParamsToMemoryAndArchive(), -1);
}

TEST_F(Communication, SetParameters) {
  for (auto const &device : devices_) {
    EXPECT_EQ(device->setParamId(111), 0);  // this is the only one which includes storeParamsToMemory
    EXPECT_EQ(device->params_->id, 111);
    EXPECT_EQ(device->getParamId(device->params_->id), 0);
    ASSERT_EQ(device->params_->id, 111);

    EXPECT_EQ(device->setParamPositionPID({0.123, -0.234, 0.0}), 0);
    EXPECT_EQ(device->setParamCurrentPID({0.0, -0.999, 0.5}), 0);
    EXPECT_EQ(device->setParamStartupActivation(true), 0);
    EXPECT_EQ(device->setParamInputMode(1), 0);
    EXPECT_EQ(device->setParamControlMode(1), 0);
    EXPECT_EQ(device->setParamEncoderResolutions({0,5,2}), 0);
    EXPECT_EQ(device->setParamEncoderOffsets({4094, 0, -4000}), 0);
    EXPECT_EQ(device->setParamEncoderMultipliers({3.4,-1.1,0.0}), 0);
    EXPECT_EQ(device->setParamUsePositionLimits(false), 0);
    EXPECT_EQ(device->setParamPositionLimits({-19000, 0}), 0);
    EXPECT_EQ(device->setParamPositionMaxSteps({-100, 100}), 0);
    EXPECT_EQ(device->setParamCurrentLimit(-1500), 0);
    EXPECT_EQ(device->storeParamsToMemory(), 0);

    EXPECT_EQ(device->getParamPositionPID(device->params_->position_pid), 0);
    EXPECT_NEAR(device->params_->position_pid.at(0), 0.123, 0.0001);
    EXPECT_NEAR(device->params_->position_pid.at(1), -0.234, 0.0001);
    EXPECT_NEAR(device->params_->position_pid.at(2), 0.0, 0.0001);
    EXPECT_EQ(device->getParamCurrentPID(device->params_->current_pid), 0);
    EXPECT_NEAR(device->params_->current_pid.at(0), 0.0, 0.0001);
    EXPECT_NEAR(device->params_->current_pid.at(1), -0.999, 0.0001);
    EXPECT_NEAR(device->params_->current_pid.at(2), 0.5, 0.0001);
    EXPECT_EQ(device->getParamStartupActivation(device->params_->startup_activation), 0);
    EXPECT_EQ(device->params_->startup_activation, true);
    EXPECT_EQ(device->getParamInputMode(device->params_->input_mode), 0);
    EXPECT_EQ(device->params_->input_mode, 1);
    EXPECT_EQ(device->getParamControlMode(device->params_->control_mode), 0);
    EXPECT_EQ(device->params_->control_mode, 1);
    EXPECT_EQ(device->getParamEncoderResolutions(device->params_->encoder_resolutions), 0);
    EXPECT_EQ(device->params_->encoder_resolutions.at(0), 0);
    EXPECT_EQ(device->params_->encoder_resolutions.at(1), 5);
    EXPECT_EQ(device->params_->encoder_resolutions.at(2), 2);
    EXPECT_EQ(device->getParamEncoderOffsets(device->params_->encoder_offsets), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(0), 4094);
    EXPECT_EQ(device->params_->encoder_offsets.at(1), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(2), -4000);
    EXPECT_EQ(device->getParamEncoderMultipliers(device->params_->encoder_multipliers), 0);
    EXPECT_NEAR(device->params_->encoder_multipliers.at(0), 3.4, 0.0001);
    EXPECT_NEAR(device->params_->encoder_multipliers.at(1), -1.1, 0.0001);
    EXPECT_NEAR(device->params_->encoder_multipliers.at(2), 0.0, 0.0001);
    EXPECT_EQ(device->getParamUsePositionLimits(device->params_->use_position_limits), 0);
    EXPECT_EQ(device->params_->use_position_limits, false);
    EXPECT_EQ(device->getParamPositionLimits(device->params_->position_limits), 0);
    EXPECT_EQ(device->params_->position_limits.at(0), -19000);
    EXPECT_EQ(device->params_->position_limits.at(1), 0);
    EXPECT_EQ(device->getParamPositionMaxSteps(device->params_->position_max_steps), 0);
    EXPECT_EQ(device->params_->position_max_steps.at(0), -100);
    EXPECT_EQ(device->params_->position_max_steps.at(1), 100);
    EXPECT_EQ(device->getParamCurrentLimit(device->params_->current_limit), 0);
    EXPECT_EQ(device->params_->current_limit, -1500);

    std::vector<int16_t> control_references, positions;
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), -15722);  //FIXME: [[firmware bug]] (sometimes 8175 or 4081 on SoftHand)
    EXPECT_EQ(control_references.at(1), 0);
    EXPECT_EQ(device->getPositions(positions), 0);
    EXPECT_EQ(positions.size(), 3);
    EXPECT_EQ(positions.at(0), -5796);  //FIXME: [[firmware bug]] (sometimes -4 or 8188 on SoftHand)
    EXPECT_EQ(positions.at(1), -1126);  //FIXME: [[firmware bug]] (should be 4094)
    EXPECT_EQ(positions.at(2), 0);  //FIXME: [[firmware bug]] (should be 0)
  }
  std::vector<int8_t> param_buffer;
  EXPECT_EQ(fake_device_->getParameters(param_buffer), -1);
  // cannot test setParameters failure on fake_device_ because there is no acknowledge
}

TEST_F(Communication, RestoreParams) {
  for (auto const &device : devices_) {
    EXPECT_EQ(device->restoreParamsFromArchive(), 0);

    EXPECT_EQ(device->getParamId(device->params_->id), 0);
    EXPECT_EQ(device->params_->id, 1);
    EXPECT_EQ(device->getParamPositionPID(device->params_->position_pid), 0);
    EXPECT_NEAR(device->params_->position_pid.at(0), 0.04, 0.0001);
    EXPECT_NEAR(device->params_->position_pid.at(1), 0, 0.0001);
    EXPECT_NEAR(device->params_->position_pid.at(2), 0.06, 0.0001);
    EXPECT_EQ(device->getParamCurrentPID(device->params_->current_pid), 0);
    EXPECT_NEAR(device->params_->current_pid.at(0), 1.0, 0.0001);
    EXPECT_NEAR(device->params_->current_pid.at(1), 0.001, 0.0001);
    EXPECT_NEAR(device->params_->current_pid.at(2), 0, 0.0001);
    EXPECT_EQ(device->getParamStartupActivation(device->params_->startup_activation), 0);
    EXPECT_EQ(device->params_->startup_activation, false);
    EXPECT_EQ(device->getParamInputMode(device->params_->input_mode), 0);
    EXPECT_EQ(device->params_->input_mode, 0);
    EXPECT_EQ(device->getParamControlMode(device->params_->control_mode), 0);
    EXPECT_EQ(device->params_->control_mode, 0);
    EXPECT_EQ(device->getParamEncoderResolutions(device->params_->encoder_resolutions), 0);
    EXPECT_EQ(device->params_->encoder_resolutions.at(0), 3);
    EXPECT_EQ(device->params_->encoder_resolutions.at(1), 3);
    EXPECT_EQ(device->params_->encoder_resolutions.at(2), 3);
    EXPECT_EQ(device->getParamEncoderOffsets(device->params_->encoder_offsets), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(0), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(1), 0);
    EXPECT_EQ(device->params_->encoder_offsets.at(2), 0);
    EXPECT_EQ(device->getParamEncoderMultipliers(device->params_->encoder_multipliers), 0);
    EXPECT_EQ(device->params_->encoder_multipliers.at(0), 1);
    EXPECT_EQ(device->params_->encoder_multipliers.at(1), 1);
    EXPECT_EQ(device->params_->encoder_multipliers.at(2), 1);
    EXPECT_EQ(device->getParamUsePositionLimits(device->params_->use_position_limits), 0);
    EXPECT_EQ(device->params_->use_position_limits, true);
    EXPECT_EQ(device->getParamPositionLimits(device->params_->position_limits), 0);
    EXPECT_EQ(device->params_->position_limits.at(0), 0);
    EXPECT_EQ(device->params_->position_limits.at(1), 19000);
    EXPECT_EQ(device->getParamPositionMaxSteps(device->params_->position_max_steps), 0);
    EXPECT_EQ(device->params_->position_max_steps.at(0), 0);
    EXPECT_EQ(device->getParamCurrentLimit(device->params_->current_limit), 0);
    EXPECT_EQ(device->params_->current_limit, 750);

    std::vector<int16_t> control_references, positions;
    EXPECT_EQ(device->getControlReferences(control_references), 0);
    EXPECT_EQ(control_references.size(), 2);
    EXPECT_EQ(control_references.at(0), 14431);  //FIXME: [[firmware bug]] (sometimes 8175 or 8188 on SoftHand)
    EXPECT_EQ(control_references.at(1), 0);
    EXPECT_EQ(device->getPositions(positions), 0);
    EXPECT_EQ(positions.size(), 3);
    EXPECT_EQ(positions.at(0), 4094);  //FIXME: [[firmware bug]] (sometimes -4098 on SoftHand)
    EXPECT_EQ(positions.at(1), 4094);
    EXPECT_EQ(positions.at(2), 4094);
  }
  EXPECT_EQ(fake_device_->restoreParamsFromArchive(), -1);
}

namespace internal {

TEST(CommunicationInternal, SwapEndians) {
  uint8_t u8{0x01};
  qbroboticsResearchAPI::Communication::swapBytes(u8);
  EXPECT_EQ(u8, 0x01);

  uint16_t u16{0x1100};
  qbroboticsResearchAPI::Communication::swapBytes(u16);
  EXPECT_EQ(u16, 0x0011);

  uint32_t u32{0x33221100};
  qbroboticsResearchAPI::Communication::swapBytes(u32);
  EXPECT_EQ(u32, 0x00112233);

  std::vector<uint8_t> u8v{0x01, 0x02, 0x03, 0x04};
  qbroboticsResearchAPI::Communication::swapBytes<uint32_t>(*reinterpret_cast<uint32_t *>(u8v.data()));
  EXPECT_EQ(u8v[0], 0x04);
  EXPECT_EQ(u8v[1], 0x03);
  EXPECT_EQ(u8v[2], 0x02);
  EXPECT_EQ(u8v[3], 0x01);
  qbroboticsResearchAPI::Communication::swapBytes<uint16_t>(*reinterpret_cast<uint16_t *>(u8v.data() + 1));
  EXPECT_EQ(u8v[0], 0x04);
  EXPECT_EQ(u8v[1], 0x02);
  EXPECT_EQ(u8v[2], 0x03);
  EXPECT_EQ(u8v[3], 0x01);

  qbroboticsResearchAPI::Communication::swapBytes(u8v);
  EXPECT_EQ(u8v[0], 0x04);
  EXPECT_EQ(u8v[1], 0x02);
  EXPECT_EQ(u8v[2], 0x03);
  EXPECT_EQ(u8v[3], 0x01);

  std::vector<uint16_t> u16v{0x1100, 0x2200, 0x3300, 0x4400};
  qbroboticsResearchAPI::Communication::swapBytes(u16v);
  EXPECT_EQ(u16v[0], 0x0011);
  EXPECT_EQ(u16v[1], 0x0022);
  EXPECT_EQ(u16v[2], 0x0033);
  EXPECT_EQ(u16v[3], 0x0044);

  std::vector<uint32_t> u32v{0x11567800, 0x22567800, 0x33567800, 0x44567800};
  qbroboticsResearchAPI::Communication::swapBytes(u32v);
  EXPECT_EQ(u32v[0], 0x00785611);
  EXPECT_EQ(u32v[1], 0x00785622);
  EXPECT_EQ(u32v[2], 0x00785633);
  EXPECT_EQ(u32v[3], 0x00785644);

  auto u8v_cast = qbroboticsResearchAPI::Communication::vectorCast<uint8_t>(u32v);
  EXPECT_EQ(u8v_cast[0], 0x11);
  EXPECT_EQ(u8v_cast[4], 0x22);
  EXPECT_EQ(u8v_cast[8], 0x33);
  EXPECT_EQ(u8v_cast[12], 0x44);

  auto u32v_cast = qbroboticsResearchAPI::Communication::vectorCast<uint32_t>(u8v);
  EXPECT_EQ(u32v_cast[0], 0x01030204);

  auto u32v_cast_swap = qbroboticsResearchAPI::Communication::vectorCastAndSwap<uint32_t>(u8v);
  EXPECT_EQ(u32v_cast_swap[0], 0x04020301);
  auto u8v_swap_cast = qbroboticsResearchAPI::Communication::vectorSwapAndCast<uint8_t>(u32v_cast_swap);
  EXPECT_EQ(u8v_swap_cast[0], 0x04);
  EXPECT_EQ(u8v_swap_cast[1], 0x02);
  EXPECT_EQ(u8v_swap_cast[2], 0x03);
  EXPECT_EQ(u8v_swap_cast[3], 0x01);
}

}  // namespace internal

}  // namespace qbrobotics_research_api

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}