/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the Warsaw University of Technology nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OPTOFORCESENSOR_H_
#define OPTOFORCESENSOR_H_

#include "can_driver/CANDev.h"
#include "Eigen/Dense"
#include <string>

class OptoforceSensor {
 public:
  enum SensorType {
    SensorType1Ch = 0,
    SensorType4Ch = 1
  };

  // configuration
  enum Speed {
    SpeedStop = 0,
    Speed1000 = 1,
    Speed333 = 3,
    Speed100 = 10,
    Speed30 = 33,
    Speed10 = 100
  };
  enum Filter {
    FilterDisabled = 0,
    Filter500 = 1,
    Filter150 = 2,
    Filter50 = 3,
    Filter15 = 4,
    Filter5 = 5,
    Filter1_5 = 6
  };
  enum Zero {
    ZeroRestore = 0,
    ZeroSet = 255
  };

  OptoforceSensor(const std::string &dev_name, SensorType type,
                  uint32_t can_rx_id, uint32_t can_tx_id);
  virtual ~OptoforceSensor();
  bool read(Eigen::Vector3d *f1, Eigen::Vector3d *f2, Eigen::Vector3d *f3);
  bool read(Eigen::Vector3d *f);
  bool isDevOpened();
  void setConfiguration(Speed s, Filter f, Zero z);

 protected:
  OptoforceSensor(const OptoforceSensor &os);
  OptoforceSensor& operator=(const OptoforceSensor &os);

  struct __attribute__((packed)) SensorPacket {
    uint8_t header_[4];
    uint16_t sample_counter_;
    uint16_t status_;
    int16_t fx1_;
    int16_t fy1_;
    int16_t fz1_;
    int16_t fx2_;
    int16_t fy2_;
    int16_t fz2_;
    int16_t fx3_;
    int16_t fy3_;
    int16_t fz3_;
    int16_t fx4_;
    int16_t fy4_;
    int16_t fz4_;
    uint16_t checksum_;
  };

  struct __attribute__((packed)) SensorPacketSmall {
    uint8_t header_[4];
    uint16_t sample_counter_;
    uint16_t status_;
    int16_t fx_;
    int16_t fy_;
    int16_t fz_;
    uint16_t checksum_;
  };

  SensorType type_;
  CANDev *pdev_;
  uint32_t can_rx_id_;
  uint32_t can_tx_id_;
};

#endif  // OPTOFORCESENSOR_H_

