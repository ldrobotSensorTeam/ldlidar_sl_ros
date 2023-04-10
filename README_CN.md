# 操作指南

>此SDK仅适用于深圳乐动机器人有限公司销售的激光雷达产品，产品型号为:
> - LDROBOT LiDAR LD14
> - LDROBOT LiDAR LD14P

## 0. 获取雷达的ROS功能包
```bash
cd ~

mkdir -p ldlidar_ros_ws/src

cd ldlidar_ros_ws/src

git clone  https://github.com/ldrobotSensorTeam/ldlidar_sl_ros.git
```

## 1. 系统设置
- 第一步，通过板载串口或者USB转串口模块(例如,cp2102模块)的方式使雷达连接到你的系统主板.
- 第二步，设置雷达在系统中挂载的串口设备-x权限(以/dev/ttyUSB0为例)
	- 实际使用时，根据雷达在你的系统中的实际挂载情况来设置，可以使用`ls -l /dev`命令查看.

``` bash
cd ~/ldlidar_ros_ws

sudo chmod 777 /dev/ttyUSB0
```
- 第三步，修改`launch/`目录下雷达产品型号对应的lanuch文件中的`port_name`值，以ld14.launch为例，如下所示.

```xml
<?xml version="1.0"?>
<launch>
<arg name="topic_name" default="scan"/>
<arg name="frame_id" default="base_laser"/>
<arg name="port_name" default="/dev/ttyUSB0"/>
<arg name="fix_to_base_link" default="true" />

<!-- LDROBOT LiDAR message publisher node -->
 <node name="ldlidar_publisher_ld14" pkg="ldlidar_sl_ros" type="ldlidar_sl_ros_node" output="screen">
  <param name="product_name" value="LDLiDAR_LD14"/>
  <param name="topic_name" value="$(arg topic_name)"/>
  <param name="frame_id" value="$(arg frame_id)"/>
  <param name="port_name" value ="$(arg port_na<?xml version="1.0"?>
<launch>
<arg name="laser_scan_topic_name" default="scan"/>
<arg name="point_cloud_2d_topic_name" default="pointcloud2d"/>
<arg name="frame_id" default="base_laser"/>
<arg name="port_name" default="/dev/ttyUSB0"/>
<arg name="fix_to_base_link" default="true" />

<!-- LDROBOT LiDAR message publisher node -->
 <node name="ldlidar_publisher_ld14" pkg="ldlidar_sl_ros" type="ldlidar_sl_ros_node" output="screen">
  <param name="product_name" value="LDLiDAR_LD14"/>
  <param name="laser_scan_topic_name" value="$(arg laser_scan_topic_name)"/>
  <param name="point_cloud_2d_topic_name" value="$(arg point_cloud_2d_topic_name)"/>
  <param name="frame_id" value="$(arg frame_id)"/>
  <param name="port_name" value="$(arg port_name)"/>
  <param name="serial_baudrate" value="115200"/>
  <!-- Set laser scan directon: -->
  <!--    1. Set counterclockwise, example: <param name="laser_scan_dir" type="bool" value="true"/> -->
  <!--    2. Set clockwise,        example: <param name="laser_scan_dir" type="bool" value="false"/> -->
  <param name="laser_scan_dir" type="bool" value="true"/>
  <!-- Angle crop setting, Mask data within the set angle range -->
  <!--    1. Enable angle crop fuction: -->
  <!--       1.1. enable angle crop,  example: <param name="enable_angle_crop_func" type="bool" value="true"/> -->
  <!--       1.2. disable angle crop, example: <param name="enable_angle_crop_func" type="bool" value="false"/> -->
  <param name="enable_angle_crop_func" type="bool" value="false"/>
  <!--    2. Angle cropping interval setting, The distance and intensity data within the set angle range will be set to 0 --> 
  <!--       angle >= "angle_crop_min" and angle <= "angle_crop_max", unit is degress -->
  <param name="angle_crop_min" type="double" value="135.0"/>
  <param name="angle_crop_max" type="double" value="225.0"/>
 </node>
 <!-- LDROBOT LiDAR message subscriber node -->
 <!-- node name="ldlidar_listener_ld14" pkg="ldlidar_sl_ros" type="ldlidar_sl_ros_listen_node" output="screen">
  <param name="topic_name" value="scan"/>
 </node -->
 <!-- publisher tf transform, parents frame is base_link, child frame is base_laser -->
 <!-- args="x y z yaw pitch roll parents_frame_id child_frame_id period_in_ms"-->
 <node name="base_to_laserLD14" pkg="tf" type="static_transform_publisher"  args="0.0 0.0 0.18 0 0.0 0.0 base_link base_laser 50" if="$(arg fix_to_base_link)"/>
</launch>

```
## 2. 编译方法
- 安装src/目录下功能包的依赖
```bash
cd ~/ldlidar_ros_ws

rosdep install --from-paths src --ignore-src  -r -y
```

- 使用catkin编译.

```bash
cd ~/ldlidar_ros_ws

catkin_make
```
## 3. 运行方法

### 3.1. 设置功能包环境变量

- 编译完成后需要将编译生成的相关文件加入环境变量，便于 ROS 环境可以识别， 执行命令如下所示， 该命令是临时给终端加入环境变量，意味着您如果重新打开新的终端，也需要重新执行如下命令.

    ```bash
    cd ~/ldlidar_ros_ws

    source devel/setup.bash
    ```
- 为了重新打开终端后，永久不用执行上述添加环境变量的命令，可以进行如下操作.

  ```bash
  echo "source ~/ldlidar_ros_ws/devel/setup.bash" >> ~/.bashrc

  source ~/.bashrc
  ```
### 3.2. 启动激光雷达节点

- 产品型号为 LDROBOT LiDAR LD14
  - 启动ld14 lidar node:
  ``` bash
  roslaunch ldlidar_sl_ros ld14.launch
  ```
  - 启动ld14 lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar_sl_ros viewer_ld14_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar_sl_ros viewer_ld14_noetic.launch
  ```
- 产品型号为 LDROBOT LiDAR LD14P
  - 启动ld14p lidar node:
  ``` bash
  roslaunch ldlidar_sl_ros ld14p.launch
  ```
  - 启动ld14p lidar node并显示激光数据在Rviz上:
  ``` bash
  # if ROS_DISTRO in 'kinetic' or 'melodic'
  roslaunch ldlidar_sl_ros viewer_ld14p_kinetic_melodic.launch
  # if ROS_DISTRO in 'noetic'
  roslaunch ldlidar_sl_ros viewer_ld14p_noetic.launch
  ```

##   4. 测试

> 代码支持ubuntu16.04 ROS kinetic、ubuntu18.04 ROS melodic、ubuntu20.04 ROS noetic版本下测试，使用rviz可视化。

- 新打开一个终端 (Ctrl + Alt + T),并通过Rviz工具打开readme文件所在目录的rviz文件夹下面的rviz配置文件
```bash
rviz
```
