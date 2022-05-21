/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "flight/dronecan.h"
#include "uavcan.h"  // NOLINT
#include "flight/msg.h"

namespace {
/* Config */
DroneCanConfig cfg_;
/* Node constants */
static constexpr uint32_t NODE_ID = 2;
static constexpr uint8_t SW_VER = 1;
static constexpr uint8_t HW_VER = 1;
static const char* NODE_NAME = "FMU-R";
static const uint32_t NODE_MEM = 8192;  // size of node memory
uavcan::equipment::actuator::ArrayCommand act_cmd_msg;
uavcan::equipment::esc::RawCommand esc_cmd_msg;
uavcan::CanDriver<1> *can;
uavcan::Node<NODE_MEM> *node;
uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> *actuator_pub;
uavcan::Publisher<uavcan::equipment::esc::RawCommand> *esc_pub;
float cmd = -1;
uavcan::CanIface<CAN3> *can3;
/* Actuator commands */
DroneCanActCmd act_;
/* ESC commands */
DroneCanEscCmd esc_;

}  // namespace

void DroneCanInit(const DroneCanConfig &cfg) {
  MsgInfo("Initializing DroneCAN node...");
  cfg_ = cfg;
  /* Init CAN interface */
  can3 = new uavcan::CanIface<CAN3>;
  can3->begin();
  can3->setBaudRate(cfg_.baud);
  /* Init CAN driver */
  can = new uavcan::CanDriver<1>({can3});
  /* Init Node */
  node = new uavcan::Node<NODE_MEM>(*can, uavcan::clock);
  uavcan::protocol::SoftwareVersion sw_ver;
  uavcan::protocol::HardwareVersion hw_ver;
  sw_ver.major = SW_VER;
  sw_ver.minor = 0;
  hw_ver.major = HW_VER;
  hw_ver.minor = 0;
  node->setNodeID(NODE_ID);
  node->setName(NODE_NAME);
  node->setSoftwareVersion(sw_ver);
  node->setHardwareVersion(hw_ver);
  if (node->start() < 0) {
    MsgError("unable to start DroneCAN node.");
  }
  /* Initialize actuator publisher */
  actuator_pub =
    new uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand>(*node);
  if (actuator_pub->init() < 0) {
    MsgError("unable to initialize DroneCAN actuator publisher.");
  }
  /* Initialize ESC publisher */
  esc_pub = new uavcan::Publisher<uavcan::equipment::esc::RawCommand>(*node);
  if (esc_pub->init() < 0) {
    MsgError("unable to initialize DroneCAN ESC publisher.");
  }
  /* Configure CAN acceptance filters */
  uavcan::configureCanAcceptanceFilters(*node);
  /* Set mode to operational */
  node->setModeOperational();
  MsgInfo("done.\n");
}

void DroneCanActuatorWrite(const DroneCanActCmd &act,
                           const DroneCanEscCmd &esc) {
  act_ = act;
  esc_ = esc;
}

void DroneCanActuatorSend() {
  /* Send actuator commands */
  act_cmd_msg.commands.resize(cfg_.num_actuators);
  for (std::size_t i = 0; i < cfg_.num_actuators; i++) {
    act_cmd_msg.commands[i].actuator_id = cfg_.config[i].actuator_id;
    act_cmd_msg.commands[i].command_type = cfg_.config[i].command_type;
    act_cmd_msg.commands[i].command_value = act_.cnt[i];
  }
  if (actuator_pub->broadcast(act_cmd_msg) < 0) {
    MsgWarning("issue publishing actuator message");
  }
  /* Send ESC commands */
  esc_cmd_msg.cmd.resize(cfg_.num_esc);
  for (std::size_t i = 0; i < cfg_.num_esc; i++) {
    esc_cmd_msg.cmd[i] = esc_.cnt[i];
  }
  if (esc_pub->broadcast(esc_cmd_msg) < 0) {
    MsgWarning("issue publishing ESC message");
  }
}
