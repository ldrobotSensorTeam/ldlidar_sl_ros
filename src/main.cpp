/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD00 LD03 LD08 LD14
 * products sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-10
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>

#include <iostream>

#include "cmd_interface_linux.h"
#include "lipkg.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldlidar_publiser");
  ros::NodeHandle nh; // create a ROS Node
  ros::NodeHandle n("~");
  std::string product_name;
	std::string topic_name;
	std::string port_name;
	std::string frame_id;

  n.getParam("product_name", product_name);
	n.getParam("topic_name", topic_name);
	n.getParam("port_name", port_name);
	n.getParam("frame_id", frame_id);
  
  ROS_INFO("[ldrobot] SDK Pack Version is v2.1.2");
  ROS_INFO("[ldrobot] <product_name>: %s,<topic_name>: %s,<port_name>: %s,<frame_id>: %s", 
    product_name.c_str(), topic_name.c_str(), port_name.c_str(), frame_id.c_str());

  LiPkg *lidar_pkg = nullptr;
  uint32_t baudrate = 0;
 
  if(product_name == "LDLiDAR_LD14") {
    baudrate = 115200;
    lidar_pkg = new LiPkg(frame_id, LDVersion::LD_FOURTEEN);
  } else{
    ROS_ERROR(" [ldrobot] Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  if (topic_name.empty()) {
    ROS_ERROR(" [ldrobot] fail, topic_name is empty!");
    exit(EXIT_FAILURE);
  }
  ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>( topic_name, 10); // create a ROS topic 

  CmdInterfaceLinux cmd_port(baudrate);

  if (port_name.empty() == false) {
    cmd_port.SetReadCallback([&lidar_pkg](const char *byte, size_t len) {
      if (lidar_pkg->Parse((uint8_t *)byte, len)) {
        lidar_pkg->AssemblePacket();
      }
    });

    if (cmd_port.Open(port_name)) {
      ROS_INFO(" [ldrobot] open %s device %s success!", product_name.c_str(), port_name.c_str());
    } else {
      ROS_ERROR(" [ldrobot] open %s device %s fail!", product_name.c_str(), port_name.c_str());
      exit(EXIT_FAILURE);
    }

    ros::Rate r(6); //hz

    while (ros::ok()) {
      if (lidar_pkg->IsFrameReady()) {
        lidar_pub.publish(lidar_pkg->GetLaserScan());
        lidar_pkg->ResetFrameReady();
#if 1
        sensor_msgs::LaserScan data = lidar_pkg->GetLaserScan();
        unsigned int lens = data.ranges.size();
        ROS_INFO_STREAM("current_speed(hz): " << lidar_pkg->GetSpeed() << " "
                        << "len: " << lens << " "
                        << "angle_min: " << RADIAN_TO_ANGLE(data.angle_min) << " "
                        << "angle_max: " << RADIAN_TO_ANGLE(data.angle_max));
        ROS_INFO_STREAM("----------------------------");
        for (int i = 0; i < lens; i++) {
          float angle_n = RADIAN_TO_ANGLE(data.angle_min + i * data.angle_increment);
          ROS_INFO_STREAM("angle: " << angle_n << " "
                          << "range(m): " << data.ranges[i] << " "
                          << "intensites: " << data.intensities[i]);
        }
#endif
      }
      r.sleep();
    }
  } else {
    ROS_ERROR(" [ldrobot] fail, port_name is empty!");
    exit(EXIT_FAILURE);
  }

  return 0;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
