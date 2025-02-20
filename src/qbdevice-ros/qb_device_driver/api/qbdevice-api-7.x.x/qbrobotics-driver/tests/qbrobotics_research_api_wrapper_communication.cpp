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

#include <qbrobotics_research_api/qbrobotics_research_api_wrapper.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <thread>

namespace qbrobotics_research_api {

TEST(Communication, ListSerialPorts) {
  char serial_ports[60][255];
  int count = RS485listPorts(serial_ports);
  for (int i=0; i<count; i++) {
    EXPECT_THAT(serial_ports[i], ::testing::MatchesRegex("/dev/ttyUSB[0-9]+"));
  }
}

TEST(Communication, ListConnectedDevices) {
  char serial_ports[60][255];
  int ports_count = RS485listPorts(serial_ports);
  for (int i=0; i<ports_count; i++) {
    ASSERT_THAT(serial_ports[i], ::testing::MatchesRegex("/dev/ttyUSB[0-9]+"));
    comm_settings comm_settings_t;
    openRS485(&comm_settings_t, serial_ports[i]);

    char device_ids[255];
    int ids_count = RS485ListDevices(&comm_settings_t, device_ids);
    for (int j=0; j<ids_count; j++) {
      ASSERT_GT(device_ids[j], 0);
      ASSERT_LT(device_ids[j], 128);
      //TODO: check uniqueness
    }

    closeRS485(&comm_settings_t);
  }
}

TEST(Communication, BasicMethods) {  // TODO: change to a test fixture
  char serial_ports[60][255];
  int ports_count = RS485listPorts(serial_ports);
  for (int i=0; i<ports_count; i++) {
    ASSERT_THAT(serial_ports[i], ::testing::MatchesRegex("/dev/ttyUSB[0-9]+"));
    comm_settings comm_settings_t;
    openRS485(&comm_settings_t, serial_ports[i]);

    char device_ids[255];
    int ids_count = RS485ListDevices(&comm_settings_t, device_ids);
    for (int j=0; j<ids_count; j++) {
      ASSERT_GT(device_ids[j], 0);
      ASSERT_LT(device_ids[j], 128);
      //TODO: check uniqueness
      int device_id = static_cast<uint8_t>(device_ids[j]);

      EXPECT_EQ(commPing(&comm_settings_t, device_id), 0);

      EXPECT_EQ(commInitMem(&comm_settings_t, device_id), 0);

      char motor_state = 0x01;
      EXPECT_EQ(commGetActivate(&comm_settings_t, device_id, &motor_state), 0);
      EXPECT_EQ(motor_state, 0x00);

      short int positions[3];
      EXPECT_EQ(commGetMeasurements(&comm_settings_t, device_id, positions), 0);
      EXPECT_EQ(positions[0], 4094);
      EXPECT_EQ(positions[1], 4094);
      EXPECT_EQ(positions[2], 4094);

      short int velocities[3];
      EXPECT_EQ(commGetVelocities(&comm_settings_t, device_id, velocities), 0);
      EXPECT_EQ(velocities[0], 0);
      EXPECT_EQ(velocities[1], 0);
      EXPECT_EQ(velocities[2], 0);

      short int accelerations[3];
      EXPECT_EQ(commGetAccelerations(&comm_settings_t, device_id, accelerations), 0);
      EXPECT_EQ(accelerations[0], 0);
      EXPECT_EQ(accelerations[1], 0);
      EXPECT_EQ(accelerations[2], 0);

      short int currents[2];
      EXPECT_EQ(commGetCurrents(&comm_settings_t, device_id, currents), 0);
      EXPECT_EQ(currents[0], 0);
      EXPECT_EQ(currents[1], 0);

      short int measurements[5];
      EXPECT_EQ(commGetCurrAndMeas(&comm_settings_t, device_id, measurements), 0);
      EXPECT_EQ(measurements[0], 0);
      EXPECT_EQ(measurements[1], 0);
      EXPECT_EQ(measurements[2], 4094);
      EXPECT_EQ(measurements[3], 127);  //FIXME: firmware error
      EXPECT_EQ(measurements[4], 4094);

      short int control_references[2];
      EXPECT_EQ(commGetInputs(&comm_settings_t, device_id, control_references), 0);
      EXPECT_EQ(control_references[0], 4094);
      EXPECT_EQ(control_references[1], 0);  // SoftHand
      control_references[0] = 12345;
      control_references[1] = 3210;
      commSetInputs(&comm_settings_t, device_id, control_references);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      control_references[0] = 0;
      control_references[1] = 0;
      EXPECT_EQ(commGetInputs(&comm_settings_t, device_id, control_references), 0);
      EXPECT_EQ(control_references[0], 12345);
      EXPECT_EQ(control_references[1], 3210);
      control_references[0] = 0;
      control_references[1] = 0;
      commSetInputs(&comm_settings_t, device_id, control_references);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      control_references[0] = 1;
      control_references[1] = 1;
      EXPECT_EQ(commGetInputs(&comm_settings_t, device_id, control_references), 0);
      EXPECT_EQ(control_references[0], 0);
      EXPECT_EQ(control_references[1], 0);

      motor_state = 0x03;
      commActivate(&comm_settings_t, device_id, motor_state);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      motor_state = 0x00;
      EXPECT_EQ(commGetActivate(&comm_settings_t, device_id, &motor_state), 0);
      EXPECT_EQ(motor_state, 0x03);
      motor_state = 0x00;
      commActivate(&comm_settings_t, device_id, motor_state);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      motor_state = 0x03;
      EXPECT_EQ(commGetActivate(&comm_settings_t, device_id, &motor_state), 0);
      EXPECT_EQ(motor_state, 0x00);

      EXPECT_EQ(commGetInputs(&comm_settings_t, device_id, control_references), 0);
      EXPECT_EQ(control_references[0], 4094);
      EXPECT_EQ(control_references[1], 4094);
      control_references[0] = 4094;
      control_references[1] = 0;
      commSetInputs(&comm_settings_t, device_id, control_references);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      char info[2000];
      EXPECT_EQ(commGetInfo(&comm_settings_t, device_id, INFO_ALL, info), 0);
      EXPECT_EQ(std::string(info).size(), 1092);

      uint8_t buffer[2000];
      std::vector<int8_t> param_buffer;
      std::vector<float> position_pid;
      EXPECT_EQ(commGetParamList(&comm_settings_t, device_id, 0, nullptr, 0, 0, buffer), 0);
      param_buffer = std::vector<int8_t>({buffer, buffer+2000});
      qbroboticsResearchAPI::Device::Params::getParameter<float>(2, param_buffer, position_pid);
      EXPECT_NEAR(position_pid.at(0), 0.0399932861, 0.0001);
      EXPECT_NEAR(position_pid.at(1), 0.0, 0.0001);
      EXPECT_NEAR(position_pid.at(2), 0.0599975586, 0.0001);

      float pid_values[3];
      pid_values[0] = 0.1;
      pid_values[1] = 0.2;
      pid_values[2] = 0.3;
      int param_index = 2;
      int value_size = 4;
      int num_of_values = 3;
      commGetParamList(&comm_settings_t, device_id, param_index, pid_values, value_size, num_of_values, nullptr);

      EXPECT_EQ(commStoreParams(&comm_settings_t, device_id), 0);

      EXPECT_EQ(commGetParamList(&comm_settings_t, device_id, 0, nullptr, 0, 0, buffer), 0);
      param_buffer = std::vector<int8_t>({buffer, buffer+2000});
      qbroboticsResearchAPI::Device::Params::getParameter<float>(2, param_buffer, position_pid);
      EXPECT_NEAR(position_pid.at(0), 0.1, 0.0001);
      EXPECT_NEAR(position_pid.at(1), 0.2, 0.0001);
      EXPECT_NEAR(position_pid.at(2), 0.3, 0.0001);

      EXPECT_EQ(commRestoreParams(&comm_settings_t, device_id), 0);
    }

    closeRS485(&comm_settings_t);
  }
}

//TODO: add tests for non connected device

}  // namespace qbrobotics_research_api

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}