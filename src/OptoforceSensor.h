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

#ifndef _OPTOFORCE_SENSOR_H_
#define _OPTOFORCE_SENSOR_H_

#include <inttypes.h>
#include "CANDev.h"
#include <string>
#include "Eigen/Dense"

class OptoforceSensor {
public:
	OptoforceSensor(std::string dev_name = "can0");
	~OptoforceSensor();
    bool read(Eigen::Vector3d &f1, Eigen::Vector3d &f2, Eigen::Vector3d &f3);
	bool isDevOpened();
protected:

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

	CANDev *pdev;
};

#endif  // OPTOFORCE_SENSOR

