# LSLIDAR_CHX1_V1.2.5_240509_ROS 使用说明



## 1.工程介绍

​		LSLIDAR_CHX1_V1.2.5_240509_ROS为linux环境下雷达ros驱动，程序在ubuntu 20.04 ros noetic,ubuntu18.04 ros melodic以及ubuntu16.04 ros kinetic下测试通过。适用于镭神cx1s3、cx6s3、ch16x1、ch64w、cb64s1_a、ch128x1、ch128s1、cx128s2、cx126s3、ch256雷达



## 2.依赖：

1.ros

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros 依赖

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.其他依赖

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适当的版本
~~~



### 3.编译与运行：

#### 3.1 编译

~~~shell
#创建工作空间及src目录  lidar_ws为工作空间名 可自定义
mkdir -p ~/lidar_ws/src
#将驱动压缩包解压缩放到~/lidar_ws/src 目录下
#返回工作空间
cd ~/lidar_ws
#编译
catkin_make
~~~

### 3.2 运行

运行单个雷达:

~~~bash
source devel/setup.bash #刷新当前终端环境
roslaunch lslidar_driver lslidar_ch.launch  #运行(请确保雷达型号与launch文件中一致)
~~~

运行多个雷达：

~~~bash
source devel/setup.bash #刷新当前终端环境
roslaunch lslidar_driver lslidar_double.launch #运行两台雷达(请确保雷达型号与launch文件中一致)
~~~



### 4.launch 文件参数说明：

~~~xml
<launch>
    <arg name="device_ip" default="192.168.1.200"/> <!--雷达IP-->
    <arg name="msop_port" default="2368" />         <!--雷达目的数据端口-->
    <arg name="difop_port" default="2369" />        <!--雷达目的设备端口-->
    <arg name="lidar_type" default="cx128s2"/> <!--请选择正确的雷达型号: cx1s3 cx6s3 ch16x1 ch64w cb64s1_a cx126s3 ch128x1 ch128s1 cx128s2 ch256-->
    <arg name="use_time_service" default="false" /> <!--是否开启外部授时 GPS PTP-->

  <node pkg="lslidar_driver" type="lslidar_ch_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_ch_driver)/pcap/xxx.pcap"/-->  <!--离线播pcap包路径-->
    <param name="packet_rate" value="13475.0"/>     <!--离线播包每秒包数 不同雷达包数不同-->
    <param name="lidar_type" value="$(arg lidar_type)"/>
    <param name="lidar_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="add_multicast" value="false"/>     <!--雷达是否启用组播模式 true: 开启-->
    <param name="group_ip" value="224.1.1.2"/>      <!--组播IP 可在上位机设置需要的IP地址-->
    <param name="frame_id" value="laser_link"/>     <!--雷达点云坐标系ID 可修改-->
    <Param name="pointcloud_topic" value="lslidar_point_cloud"/>  <!--点云话题名称-->
    <param name="min_range" value="0.3"/>           <!--最小距离 小于此值的点将被过滤 可根据实际需求更改 单位:米-->
    <param name="max_range" value="500.0"/>         <!--最大距离 大于此值的点将被过滤 可根据实际需求更改 单位:米-->
    <param name="scan_start_angle" value="3000"/>   <!--最小角度 ch64w cb64s1_a雷达最小角度请改为 0  小于此值的点将被过滤 可根据实际需求更改 单位:0.01°-->
    <param name="scan_end_angle" value="15000"/>    <!--最大角度 ch64w cb64s1_a雷达最大角度请改为 18000  大于此值的点将被过滤 可根据实际需求更改 单位:0.01°-->
    <param name="horizontal_angle_resolution" value="0.15"/> <!--laserscan角度分辨率,根据雷达型号修改-->
    <param name="publish_laserscan" value="false"/> <!--是否发布laserscan true:发布-->
    <param name="channel_num" value="64"/>  <!--laserscan线号 最大值需小于雷达总线数-->
    <param name="echo_mode" value="0"/> <!--双回波模式下生效 0:发布全部点云  1:发布第一次回波点云  2:发布第二次回波点云-->
  </node>

  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_ch_driver)/rviz/lslidar.rviz"/> <!--自动加载rviz-->
</launch>
~~~



### 组播模式：

- 上位机设置雷达开启组播模式

- 修改launch文件以下参数

  ~~~xml
      <param name="add_multicast" value="true"/> 
      <param name="group_ip" value="224.1.1.2"/>  //上位机设置的组播ip地址
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~





### 离线pcap模式：

- 把录制好的pcap文件，拷贝到lslidar_driver/pcap文件夹下

- 修改launch文件以下参数

  ~~~xml
     //取消注释
   	 <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap">  // xxx.pcap改为拷贝的pcap文件名
       <param name="packet_rate" value="13475.0"/>   //pcap包每秒包数 不同雷达包数不同
  ~~~

- 运行驱动即可离线读取pcap包中数据(默认为循环读取)





###  PCL点云：

- 点云中的点为xyzirt字段,定义参考include/lslidar_ch_driver/lslidar_ch_driver.h头文件
- 点云时间为每帧最后一个点的时间
- 点的时间为相对时间 范围(0 - 0.1)





## FAQ

Bug Report

version :LSLIDAR_CHX1_V1.1.9_221107_ROS

Modify:  original version

Date    : 2022-11-07

----



update version:LSLIDAR_CHX1_V1.2.1_230316_ROS

Modify:  

​	1.降低cpu占用；点的时间改为相对时间；新增兼容cx128s2 雷达。

Date    : 2023-03-16

----



update version:LSLIDAR_CHX1_V1.2.2_230414_ROS

Modify:  

1. 新增兼容cx126s3 雷达。

Date    : 2023-04-14

----



update version:LSLIDAR_CHX1_V1.2.3_230704_ROS

Modify:  

​	1.新增兼容cb64s1_a雷达。

Date    : 2023-07-04

----



update version:LSLIDAR_CHX1_V1.2.4_230901_ROS

Modify:  

​	1.新增兼容cx6s3、ch256、cx1s3雷达。

Date    : 2023-09-01

----



update version:LSLIDAR_CHX1_V1.2.5_240509_ROS

Modify:  

​	1.分离雷达坐标解析，降低CPU占用
​    2.更新cx128s2水平角度计算

​    3.合并驱动功能包

Date    : 2024-05-09

----


