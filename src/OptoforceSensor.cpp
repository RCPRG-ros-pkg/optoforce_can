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

#include "optoforce_can/OptoforceSensor.h"

#include <string>
#include <cstring>
#include <iostream>

OptoforceSensor::OptoforceSensor(const std::string &dev_name, OptoforceSensor::SensorType type, uint32_t can_rx_id, uint32_t can_tx_id):
    can_rx_id_(can_rx_id),
    can_tx_id_(can_tx_id)
{
    // read:   0x0100    001 0000 0000
    // write:  0x0101    001 0000 0001
    // filter: 0x0100    001 0000 0000
    // mask:   0x07FE    111 1111 1110
    std::vector<CANDev::FilterElement > filters;
    filters.push_back( CANDev::FilterElement(can_rx_id_, 0x07FE) );
    pdev_ = new CANDev(dev_name, "OptoforceSensor", filters);

    type_ = type;
    if (type_ == SensorType1Ch) {
        pdev_->setMultiFrameReceiver(16);
    }
    else if (type == SensorType4Ch) {
        pdev_->setMultiFrameReceiver(34);
    }
    else {
        std::cout << "ERROR: OptoforceSensor::OptoforceSensor: wrong sensor type: " << static_cast<int >(type_) << std::endl;
    }
}

OptoforceSensor::~OptoforceSensor() {
    delete pdev_;
}

bool OptoforceSensor::isDevOpened() {
	return pdev_->isOpened();
}

void OptoforceSensor::setConfiguration(OptoforceSensor::Speed s, OptoforceSensor::Filter f, OptoforceSensor::Zero z) {
    uint8_t data[16] = { 170, 0, 50, 3, static_cast<uint8_t >(s), static_cast<uint8_t >(f), static_cast<uint8_t >(z), 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint16_t checksum = 0;
    for (int i = 0; i <7; i++) {
        checksum += data[i];
    }
    data[7] = (checksum>>8)&0xFF;
    data[8] = checksum&0xFF;
    
    pdev_->send(can_tx_id_, 8, &data[0]);
    pdev_->send(can_tx_id_, 8, &data[8]);
}

bool OptoforceSensor::read(Eigen::Vector3d &f1, Eigen::Vector3d &f2, Eigen::Vector3d &f3) {
    if (type_ != SensorType4Ch) {
        return false;
    }

    CANDev::Header header = { 170, 7, 8, 28 };
    struct SensorPacket sp;
    uint8_t *data = reinterpret_cast<uint8_t* >(&sp);
    int bytes = pdev_->readMultiFrameData(can_rx_id_, &data[0], header);

    if (bytes != 34) {
        //std::cout << "ERROR: OptoforceSensor::read: bytes: " << bytes << std::endl;
        return false;
    }

    // change the endianes
    for (int i = 4; i < 34; i+=2) {
        uint8_t tmp = data[i];
        data[i] = data[i+1];
        data[i+1] = tmp;
    }

    uint16_t checksum = 0u;
    for (int i = 0; i < 32; i++) {
        checksum += static_cast<uint16_t >(data[i]);
    }

    if (checksum != sp.checksum_) {
        //std::cout << "ERROR: OptoforceSensor::read: checksum: " << checksum << " " << sp.checksum_ << std::endl;
        return false;
    }

    f1(0) = static_cast<double >(sp.fx1_);
    f1(1) = static_cast<double >(sp.fy1_);
    f1(2) = static_cast<double >(sp.fz1_);

    f2(0) = static_cast<double >(sp.fx2_);
    f2(1) = static_cast<double >(sp.fy2_);
    f2(2) = static_cast<double >(sp.fz2_);

    f3(0) = static_cast<double >(sp.fx3_);
    f3(1) = static_cast<double >(sp.fy3_);
    f3(2) = static_cast<double >(sp.fz3_);

    return true;
}

bool OptoforceSensor::read(Eigen::Vector3d &f) {
    if (type_ != SensorType1Ch) {
        return false;
    }

    CANDev::Header header = { 170, 7, 8, 10 };
    struct SensorPacketSmall sp;
    uint8_t *data = reinterpret_cast<uint8_t* >(&sp);
    int bytes = pdev_->readMultiFrameData(can_rx_id_, &data[0], header);

    if (bytes != 16) {
        //std::cout << "ERROR: OptoforceSensor::read: bytes: " << bytes << std::endl;
        return false;
    }

    // change the endianes
    for (int i = 4; i < 16; i+=2) {
        uint8_t tmp = data[i];
        data[i] = data[i+1];
        data[i+1] = tmp;
    }

    uint16_t checksum = 0u;
    for (int i = 0; i < 14; i++) {
        checksum += static_cast<uint16_t >(data[i]);
    }

    if (checksum != sp.checksum_) {
        //std::cout << "ERROR: OptoforceSensor::read: checksum: " << checksum << " " << sp.checksum_ << std::endl;
        return false;
    }

    f(0) = static_cast<double >(sp.fx_);
    f(1) = static_cast<double >(sp.fy_);
    f(2) = static_cast<double >(sp.fz_);

    return true;
}

