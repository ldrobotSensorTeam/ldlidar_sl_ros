/**
 * @file main.cpp
 * @author LDRobot (contact@ldrobot.com)
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
#include "ldlidar_driver.h"
#include "ros_api.h"

uint64_t GetTimestamp(void);

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, 
    LaserScanSetting& setting, ros::Publisher& lidarpub);

void  ToSensorPointCloudMessagePublish(ldlidar::Points2D& src, LaserScanSetting& setting, 
    ros::Publisher& lidarpub);

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldlidar_publiser");

  ros::NodeHandle nh; // create a ROS Node
  ros::NodeHandle nh_private("~");
  std::string product_name;
	std::string laser_scan_topic_name;
  std::string point_cloud_2d_topic_name;
	std::string port_name;
  LaserScanSetting setting;
  int serial_baudrate = 0;
  ldlidar::LDType lidartypename = ldlidar::LDType::NO_VER;

  nh_private.getParam("product_name", product_name);
	nh_private.param("laser_scan_topic_name", laser_scan_topic_name, std::string("scan"));
  nh_private.param("point_cloud_2d_topic_name", point_cloud_2d_topic_name, std::string("pointcloud2d"));
	nh_private.param("frame_id", setting.frame_id, std::string("base_laser"));
  nh_private.param("port_name", port_name, std::string("/dev/ttyUSB0"));
  nh_private.param("serial_baudrate", serial_baudrate, int(115200));
  nh_private.param("laser_scan_dir", setting.laser_scan_dir, bool(true));
  nh_private.param("enable_angle_crop_func", setting.enable_angle_crop_func, bool(false));
  nh_private.param("angle_crop_min", setting.angle_crop_min, double(0.0));
  nh_private.param("angle_crop_max", setting.angle_crop_max, double(0.0));

  ldlidar::LDLidarDriver* node = new ldlidar::LDLidarDriver();

  ROS_INFO("LDLiDAR SDK Pack Version is:%s", node->GetLidarSdkVersionNumber().c_str());
  ROS_INFO("ROS param input: ");
  ROS_INFO("<product_name>: %s", product_name.c_str());
  ROS_INFO("<laser_scan_topic_name>: %s", laser_scan_topic_name.c_str());
  ROS_INFO("<point_cloud_2d_topic_name>: %s", point_cloud_2d_topic_name.c_str());
  ROS_INFO("<frame_id>: %s", setting.frame_id.c_str());
  ROS_INFO("<port_name>: %s", port_name.c_str());
  ROS_INFO("<serial_baudrate>: %d", serial_baudrate);
  ROS_INFO("<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  ROS_INFO("<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  ROS_INFO("<angle_crop_min>: %f", setting.angle_crop_min);
  ROS_INFO("<angle_crop_max>: %f", setting.angle_crop_max);

  node->RegisterGetTimestampFunctional(std::bind(&GetTimestamp)); 

  node->EnableFilterAlgorithnmProcess(true);

  if (port_name.empty()) {
    ROS_ERROR("fail, input param <port_name> is empty!");
    exit(EXIT_FAILURE);
  }

  if(product_name == "LDLiDAR_LD14") {
    lidartypename = ldlidar::LDType::LD_14;
  } else{
    ROS_ERROR("Error, input param <product_name> is fail!!");
    exit(EXIT_FAILURE);
  }

  if (node->Start(lidartypename, port_name, serial_baudrate)) {
    ROS_INFO("ldlidar node start is success");
  } else {
    ROS_ERROR("ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (node->WaitLidarCommConnect(3500)) {
    ROS_INFO("ldlidar communication is normal.");
  } else {
    ROS_ERROR("ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  ros::Publisher lidar_pub_laserscan = 
    nh.advertise<sensor_msgs::LaserScan>(laser_scan_topic_name, 10); // create a ROS sensor_msgs/LaserScan Message topic
  
  ros::Publisher lidar_pub_pointcloud = 
    nh.advertise<sensor_msgs::PointCloud>(point_cloud_2d_topic_name, 10); // create a ROS sensor_msgs/PointCloud Message topic

  ros::Rate r(6); 
  ldlidar::Points2D laser_scan_points;
  ROS_INFO("start normal, pub lidar data");
  while (ros::ok() && ldlidar::LDLidarDriver::IsOk()) {

    switch (node->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL: {
        double lidar_scan_freq = 0;
        node->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting, lidar_pub_laserscan);
        ToSensorPointCloudMessagePublish(laser_scan_points, setting, lidar_pub_pointcloud);
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT: {
        ROS_ERROR("ldlidar point cloud data publish time out, please check your lidar device.");
        node->Stop();
        break;
      }
      case ldlidar::LidarStatus::DATA_WAIT: {
        break;
      }
      default:
        break;
    }

    r.sleep();
  }

  node->Stop();

  delete node;
  node = nullptr;
  
  return 0;
}

uint64_t GetTimestamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, 
  LaserScanSetting& setting, ros::Publisher& lidarpub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 12;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);

  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (beam_size - 1);
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN(); 
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = (int)((angle - angle_min) / angle_increment);
      if (index < beam_size) {
        if (index < 0) {
          ROS_ERROR("[ldrobot] error index: %d, beam_size: %d, angle: %f, angle_min: %f, angle_increment: %f", 
              index, beam_size, angle, angle_min, angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }
    lidarpub.publish(output);
    end_scan_time = start_scan_time;
  } 
}

void  ToSensorPointCloudMessagePublish(ldlidar::Points2D& src, LaserScanSetting& setting, 
    ros::Publisher& lidarpub) {
  
  ros::Time start_scan_time;
  double scan_time;
  float time_increment;
  static ros::Time end_scan_time;
  static bool first_scan = true;

  ldlidar::Points2D dst = src;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  if (setting.laser_scan_dir) {
    for (auto&point : dst) {
      point.angle = 360.f - point.angle;
      if (point.angle < 0) {
        point.angle += 360.f;
      }
    }
  } 

  int frame_points_num = static_cast<int>(dst.size());

  sensor_msgs::PointCloud output;

  output.header.stamp = start_scan_time;
  output.header.frame_id = setting.frame_id;

  sensor_msgs::ChannelFloat32 defaultchannelval[3];

  defaultchannelval[0].name = std::string("intensity");
  defaultchannelval[0].values.assign(frame_points_num, std::numeric_limits<float>::quiet_NaN());
  // output.channels.assign(1, defaultchannelval);
  output.channels.push_back(defaultchannelval[0]);

  if (frame_points_num <= 1) {
    time_increment = 0;
  } else {
    time_increment = static_cast<float>(scan_time / (double)(frame_points_num - 1));
  }
  defaultchannelval[1].name = std::string("timeincrement");
  defaultchannelval[1].values.assign(1, time_increment);
  output.channels.push_back(defaultchannelval[1]);
  
  defaultchannelval[2].name = std::string("scantime");
  defaultchannelval[2].values.assign(1, scan_time);
  output.channels.push_back(defaultchannelval[2]);

  geometry_msgs::Point32 points_xyz_defaultval;
  points_xyz_defaultval.x = std::numeric_limits<float>::quiet_NaN();
  points_xyz_defaultval.y = std::numeric_limits<float>::quiet_NaN();
  points_xyz_defaultval.z = std::numeric_limits<float>::quiet_NaN();
  output.points.assign(frame_points_num, points_xyz_defaultval);

  for (int i = 0; i < frame_points_num; i++) {
    float range = dst[i].distance / 1000.f;  // distance unit transform to meters
    float intensity = dst[i].intensity;      // laser receive intensity 
    float dir_angle = ANGLE_TO_RADIAN(dst[i].angle);
    //  极坐标系转换为笛卡尔直角坐标系
    output.points[i].x = range * cos(dir_angle);
    output.points[i].y = range * sin(dir_angle);
    output.points[i].z = 0.0;
    output.channels[0].values[i] = intensity;
  }
  
  lidarpub.publish(output);
  end_scan_time = start_scan_time;
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
