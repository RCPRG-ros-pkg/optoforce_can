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

#include "OptoforceSensor.h"

#include <string>
#include <cstring>
#include <iostream>

OptoforceSensor::OptoforceSensor(std::string dev_name) {
    // read:   0x0100    001 0000 0000
    // write:  0x0101    001 0000 0001
    // filter: 0x0100    001 0000 0000
    // mask:   0x07FE    111 1111 1110
    std::vector<CANDev::FilterElement > filters;
    filters.push_back( CANDev::FilterElement(0x0100, 0x07FE) );
    pdev = new CANDev(dev_name, filters);
    pdev->setMultiFrameReceiver(34);
}

OptoforceSensor::~OptoforceSensor() {
    delete pdev;
}

bool OptoforceSensor::isDevOpened() {
	return pdev->isOpened();
}

bool OptoforceSensor::read(Eigen::Vector3d &f1, Eigen::Vector3d &f2, Eigen::Vector3d &f3) {
    struct SensorPacket sp;
    uint8_t *data = reinterpret_cast<uint8_t* >(&sp);
    int bytes = pdev->readMultiFrameData(0x0100, &data[0]);

    if (bytes != 34) {
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

