# LSLIDAR_LS1550_V1.1.1_240809_ROS使用说明

## 1.工程介绍
​		LSLIDAR_LS1550_V1.1.1_240809_ROS为linux环境下雷达ros驱动，适用于 LS25D、LS128、LS144、LS160、LS180、LS320、LS400、MS06雷达。



## 2.依赖

1.ubuntu20.04 ros noetic/ubuntu18.04 ros melodic/ubuntu16.04 ros kinetic

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions
```

3.其他依赖

pcap,boost

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~



## 3.编译运行

### 3.1 编译

~~~bash
mkdir -p ~/lidar_ws/src
#将ROS压缩包解压缩放到~/lidar_ws/src 目录下
cd ~/lidar_ws
catkin_make
source devel/setup.bash
~~~

### 3.2 运行

运行单个雷达:

~~~bash
roslaunch lslidar_ls_driver lslidar_ls1550.launch
~~~

运行多个雷达：

~~~bash
roslaunch lslidar_ls_driver lslidar_ls1550_double.launch
~~~



## 4.参数介绍

lslidar_ls1550.launch文件内容如下，每个参数含义见注释说明。

~~~xml
<launch>
    <arg name="device_ip" default="192.168.1.200"/> <!--雷达ip-->
    <arg name="msop_port" default="2368"/>          <!--雷达目的数据端口-->
    <arg name="difop_port" default="2369"/>         <!--雷达目的设备端口-->
    <arg name="use_time_service" default="false"/>  <!--是否使用授时,false:使用系统时间-->
    <arg name="packet_rate" default="15000.0"/>     <!--pcap每秒包数，离线解析pcap包时使用-->

    <node pkg="lslidar_ls_driver" type="lslidar_ls_driver_node" name="lslidar_ls_driver_node" output="screen">
        <!--param name="pcap" value="$(find lslidar_ls_driver)/pcap/ls400s3.pcap"/-->
        <param name="packet_rate" value="$(arg packet_rate)"/>
        <param name="lidar_ip" value="$(arg device_ip)"/>
        <param name="msop_port" value="$(arg msop_port)"/>
        <param name="difop_port" value="$(arg difop_port)"/>
        <param name="add_multicast" value="false"/> <!--是否开启组播-->
        <param name="group_ip" value="224.1.1.2"/>  <!--组播ip-->
        <param name="min_range" value="0.15"/>      <!--雷达最小探测距离裁剪-->
        <param name="max_range" value="1500"/>      <!--雷达最大探测距离裁剪-->
        <param name="scan_start_angle" value="-60"/><!--雷达最小探测角度裁剪-->
    	<param name="scan_end_angle" value="60"/>   <!--雷达最小探测角度裁剪-->
        <param name="frame_id" value="laser_link"/> <!--点云帧id-->
        <param name="pointcloud_topic" value="lslidar_point_cloud"/>    <!--点云话题名称-->
        <param name="packet_loss" value="false"/>   <!--是否开启丢包检测-->
        <param name="use_time_service" value="$(arg use_time_service)"/>
    </node>
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_ls_driver)/rviz/lslidar_ls.rviz" output="screen"/>    <!--不启动rviz将此行注释-->
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
  ~~~



## 5.丢包检测

开启丢包检测后驱动将雷达丢包总数以topic的形式发布出来，topic名字为"packet_loss"，消息类型为std_msgs::Int64



## 6.故障码

驱动将雷达故障码以topic的形式发布出来，topic名字为"lslidar_fault_code"，消息类型为std_msgs::String

*每位故障码含义请联系技术支持*



## 7.ROS服务

### 设置雷达帧率：

source devel/setup.bash

可选帧率  正常帧率/50帧率/25帧率

~~~bash
rosservice call /set_frame_rate "frame_rate: 0" 		#正常帧率
rosservice call /set_frame_rate "frame_rate: 1"			#50帧率
rosservice call /set_frame_rate "frame_rate: 2"			#25帧率
~~~



### 设置雷达IP：

~~~bash
source devel/setup.bash
rosservice call /set_data_ip "data_ip: ''"		
#在''内填入ip 例如：192.168.1.201
~~~



### 设置雷达目的ip：

~~~bash
source devel/setup.bash
rosservice call /set_destination_ip "destination_ip: ''"
#在''内填入雷达目的ip 例如：192.168.1.100
~~~



### 设置雷达目的数据端口：

~~~bash
source devel/setup.bash
rosservice call /set_data_port "data_port: " 
#在: "内填入雷达目的设备端口 例如：2370
~~~



### 设置雷达目的设备端口：

~~~bash
source devel/setup.bash
rosservice call /set_dev_port "dev_port: "
#在: "内填入雷达目的设备端口 例如：2371
~~~



### 设置雷达时间源：

**GPS授时**

~~~bash
source devel/setup.bash
rosservice call /set_time_service "time_service_mode: 'gps'
ntp_ip: ''"
~~~

**PTP授时**

~~~bash
source devel/setup.bash
# PTP(L2)
rosservice call /set_time_service "time_service_mode: 'ptp_l2'
ntp_ip: ''"

# PTP(UDPV4)
rosservice call /set_time_service "time_service_mode: 'ptp_udpv4'
ntp_ip: ''"
~~~

**NTP授时**

~~~bash
source devel/setup.bash
rosservice call /set_time_service "time_service_mode: 'ntp'
ntp_ip: '192.168.1.102'"		# 192.168.1.102 为 NTP IP
~~~



### **设置雷达角度畸变校正**

**修改后请重启驱动程序**

~~~bash
rosservice call /set_angle_distortion_correction "angle_distortion_correction: 1"
														# 0: 关闭		1: 开启
~~~



### **设置雷达是否发送无效数据**

~~~bash
rosservice call /set_invalid_data "invalid_data: 1"		# 0: 发送		1: 不发送
~~~



### 设置雷达待机模式

~~~bash
rosservice call /set_standby_mode "standby_mode: 1"		# 0: 正常		1: 待机
~~~



## FAQ

Bug Report

Original version : LSLIDAR_LS128_ROS_V1.0.0_220622_ROS

Modify:  original version

Date    : 2022-06-24

-----------------------------------------------------------------------------------------


Original version : LSLIDAR_LS128_ROS_V1.0.1_220805_ROS

Modify:  1.修复水平角度缺失的问题

Date    : 2022-08-05

-----------------------------------------------------------------------------------------



Original version : LSLIDAR_LS128_ROS_V1.0.2_220901_ROS

Modify:  1.添加GPS授时和PTP授时时间戳解析

Date    : 2022-09-01

----------------------------------------



Original version : LSLIDAR_LS128_ROS_V1.0.3_220929_ROS

Modify:  1.fpga版本升级，设备包包头判断条件更改

2.修改点的时间戳计算方式

3.修改振镜偏移角度 fGalvanometrtAngle计算公式

Date    : 2022-09-29

------------------------------------



Original version : LSLIDAR_LS128_ROS_V1.0.4_221128_ROS

Modify:  1.fpga版本升级，修改点云计算公式

Date    : 2022-11-28

----------------------------



Original version : LSLIDAR_LS128_ROS_V1.0.5_230227_ROS

Modify:  1.新增对LS128S2雷达的支持(叠帧功能)

Date    : 2023-02-27

---------------



Original version : LSLIDAR_LS128_ROS_V1.0.6_230301_ROS

Modify:  1.fpga协议变更，修改叠帧判断条件

Date    : 2023-03-01

---------------



Original version : LSLIDAR_LS128_ROS_V1.0.7_230313_ROS

Modify:  1.预计算水平角度和垂直角度正弦和余弦值，降低CPU占用

2.新增丢包检测的功能

Date    : 2023-03-13

----



Original version : LSLIDAR_LS1550_ROS_V1.0.8_230907_ROS

Modify:  

1.新增兼容MS06雷达

2.修复丢包检测话题数据异常

3.修改了雷达摆镜角度值和偏移值

Date    : 2023-09-07

----



Original version : LSLIDAR_LS1550_ROS_V1.0.9_240301_ROS

Modify:  

1.优化代码

2.增加修改雷达帧率，ip，端口服务

Date    : 2024-03-01

----



Original version : LSLIDAR_LS1550_ROS_V1.1.0_240426_ROS

Modify: 

1.新增支持雷达去畸变解析

Date    : 2024-04-26

----



Original version : LSLIDAR_LS1550_V1.1.1_240809_ROS

Modify: 

1.新增雷达故障码信息发布

2.合并功能包，优化代码结构

3.自动识别雷达是否开启去畸变模式

4.新增设置修改雷达时间源，待机模式，角度畸变矫正，发送无效数据服务

Date    : 2024-08-09

----





