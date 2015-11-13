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

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <geometry_msgs/Vector3Stamped.h>

#include "OptoforceSensor.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "optoforce_can_test");
    ros::NodeHandle nh;

    OptoforceSensor os("can0", OptoforceSensor::SensorType4Ch);

    if (os.isDevOpened()) {
        std::cout << "device is opened" << std::endl;
    }
    else {
        std::cout << "could not open the device" << std::endl;
    }

    os.setConfiguration(OptoforceSensor::Speed100, OptoforceSensor::Filter50, OptoforceSensor::ZeroSet);

    ros::Publisher pub1 = nh.advertise<geometry_msgs::Vector3Stamped>("/optoforce/force1", 100);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Vector3Stamped>("/optoforce/force2", 100);
    ros::Publisher pub3 = nh.advertise<geometry_msgs::Vector3Stamped>("/optoforce/force3", 100);

    while (ros::ok()) {
        Eigen::Vector3d f1, f2, f3;
        if (!os.read(f1, f2, f3)) {
            std::cout << "could not read the sensor data" << std::endl;
        }

        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.vector.x = f1(0);
        msg.vector.y = f1(1);
        msg.vector.z = f1(2);
        pub1.publish(msg);

        msg.vector.x = f2(0);
        msg.vector.y = f2(1);
        msg.vector.z = f2(2);
        pub2.publish(msg);

        msg.vector.x = f3(0);
        msg.vector.y = f3(1);
        msg.vector.z = f3(2);
        pub3.publish(msg);

        ros::spinOnce();
    }

    return 0;
}
