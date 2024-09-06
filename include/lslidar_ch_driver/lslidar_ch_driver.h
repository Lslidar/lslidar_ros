/******************************************************************************
 * This file is part of lslidar driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include <mutex>
#include <cmath>
#include <unordered_map>
#include <functional>
#include <lslidar_ch_driver/LslidarChPacket.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "lslidar_ch_driver/lslidar_control.h"
#include "input.h"
#include "ThreadPool.h"

namespace lslidar_ch_driver {
    constexpr double DEG_TO_RAD = 0.017453292;
    constexpr double RAD_TO_DEG = 57.29577951;
    constexpr double SINGLE_ECHO = 0.005847953;
    constexpr double DOUBLE_ECHO = 0.009174312;

    constexpr double DISTANCE_RESOLUTION = 0.0000390625;
    constexpr double sqrt_0_5 = std::sqrt(0.5);
    constexpr double pow1 = -3.6636 * pow(10, -7);
    constexpr double pow2 = 5.2766 * pow(10, -5);
    constexpr double pow3 = 1.4507 * pow(10, -4);
    constexpr double big_angle[32]={-17,-16,-15,-14,-13,-12,-11,-10,
                                    -9,-8,-7,-6,-5,-4.125,-4,-3.125,
                                    -3,-2.125,-2,-1.125,-1,-0.125,0,
                                    0.875,1,1.875,2,3,4,5,6,7};

    constexpr double big_angle_cx6s3[2] = {-0.6,0.0};

    constexpr double big_angle_ch16x1[4] = {-1.0,0.0,1.0,2.0};

    constexpr double big_angle_ch128s1[32] = {-12, -11, -10, -9, -8, -7, -6, -5,
                                            -4.125, -4, -3.125, -3, -2.125, -2, -1.125, -1,
                                            -0.125, 0, 0.875, 1, 1.875, 2, 3, 4,
                                            5, 6, 7, 8, 9, 10, 11, 12};

    constexpr double big_angle_cx126s3[42] = {-12.0,-11.4,-10.8,-10.2,-9.6,-9.0,-8.4,-7.8,-7.2,-6.6,-6.0,-5.4,-4.8,-4.2,
                                              -3.6,-3.0,-2.4,-1.8,-1.2,-0.6,0.0,0.6,1.2,1.8,2.4,3.0,3.6,4.2,4.8,5.4,6.0,
                                              6.6,7.2,7.8,8.4,9.0,9.6,10.2,10.8,11.4,12.0,12.6};

    struct PointXYZIRT {
        PCL_ADD_POINT4D;  // 添加 x, y, z 和 padding (通常名为 data[4])
        float intensity;
        uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保动态分配时正确对齐

        // 构造
        PointXYZIRT(float x, float y, float z, float intensity, uint16_t ring, float time)
        : x(x), y(y), z(z), intensity(intensity), ring(ring), time(time) {}

    } EIGEN_ALIGN16;  // 确保整个结构体按16字节对齐

    class LslidarChDriver {
    private:
        struct Firing {
            uint16_t vertical_line;
            int azimuth;
            double distance;
            float intensity;
            float time;
        };

        std::unordered_map<std::string, std::function<void(Firing&)>> coordinateConverters;

        std::function<void(Firing&)> currentConverter;  // 当前雷达的处理函数

        std::function<void(const lslidar_ch_driver::LslidarChPacketPtr&)> packetProcess;

    public:

        LslidarChDriver(ros::NodeHandle &n, ros::NodeHandle &pn);

        ~LslidarChDriver();

        bool initialize();

        bool lslidarControl(lslidar_ch_driver::lslidar_control::Request &req,lslidar_ch_driver::lslidar_control::Response &res);

        bool SendPacketTolidar(bool power_switch);

        void publishLaserScan();

        void publishPointCloud();

        void convertCoordinate(Firing &data) {currentConverter(data);}

        void convertCoordinate_cx1s3(struct Firing &lidardata);

        void convertCoordinate_cx6s3(struct Firing &lidardata);

        void convertCoordinate_ch16x1(struct Firing &lidardata);

        void convertCoordinate_ch64w(struct Firing &lidardata);

        void convertCoordinate_cb64s1_a(struct Firing &lidardata);

        void convertCoordinate_cx126s3(struct Firing &lidardata);

        void convertCoordinate_128(struct Firing &lidardata); //ch128x1 ch128s1

        void convertCoordinate_cx128s2(struct Firing &lidardata); //cx128s2

        void convertCoordinate_ch256(struct Firing &lidardata);

        void packetProcessSingle(const lslidar_ch_driver::LslidarChPacketPtr& packet);
        
        void packetProcessDouble(const lslidar_ch_driver::LslidarChPacketPtr& packet);

        void lidarEchoMode(const lslidar_ch_driver::LslidarChPacketPtr& packet);

        bool polling();

        void difopPoll(void);

        bool getLidarEcho(void);

        void initTimeStamp(void);

        // bool isPointInRange(const double& distance) {
        //     return (distance >= min_range && distance <= max_range);
        // }

        inline void resetVariables_ch64w() {
            x = 0.0; y = 0.0; z = 0.0; sin_theat = 0.0; cos_theat = 0.0; _R_ = 0.0; cos_xita = 0.0; sin_xita = 0.0;
            cos_H_xita = 0.0; sin_H_xita = 0.0; cos_xita_F = 0.0; sin_xita_F = 0.0; add_distance = 0.0;
        }

        inline void resetVariables_cb64s1_a() {
            x = 0.0; y = 0.0; z = 0.0; sin_theat = 0.0; cos_theat = 0.0; _R_ = 0.0; cos_xita = 0.0; sin_xita = 0.0;
        }

        typedef boost::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef boost::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:

        bool loadParameters();

        bool createRosIO();

        //socket Parameters
        int msop_udp_port;
        int difop_udp_port;
        int echo_byte = 1211;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string pointcloud_topic;
        std::string lidar_type;

        in_addr lidar_ip;

        int socket_id;

        std::string dump_file;
        double prism_angle[4];
        double sin_list[36000]{};
        double cos_list[36000]{};

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        sensor_msgs::LaserScan::Ptr scan_msg;
        sensor_msgs::LaserScan::Ptr scan_msg_bak;

        uint point_size;
        double prism_offset;
        double min_range;
        double max_range;
        int scan_start_angle;
        int scan_end_angle;
        int channel_num;
        int channel_num1;
        int channel_num2;
        int echo_mode;

        double horizontal_angle_resolution;

        // ROS related variables
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Publisher pointcloud_pub;
        ros::Publisher laserscan_pub;
        ros::ServiceServer lslidar_control;

        unsigned char difop_data[1206] = {0};
        // add for time synchronization
        
        std::string time_service_mode;

        bool use_time_service;
        bool publish_laserscan;
        bool gain_prism_angle;
        bool is_update_difop_packet;
        bool lidarEcho = false;
        bool add_multicast;
        bool packetType;

        uint32_t packet_timestamp_s;
        uint32_t packet_timestamp_ns;
        double packet_timestamp;
        double last_packet_timestamp;
        double point_cloud_timestamp;
        double packet_interval_time;
        double point_interval_time;
        double point_time;
        double packet_rate;

        std::mutex pointcloud_lock;
        std::unique_ptr <ThreadPool> threadPool_;

        unsigned char packetTimeStamp[10];
        struct tm cur_time;

        double sin_theta_1[128];
        double sin_theta_2[128];
        double cos_theta_1[128];
        double cos_theta_2[128];
        double ch256_sin_theta_1[256];
        double ch256_sin_theta_2[256];
        double ch256_cos_theta_1[256];
        double ch256_cos_theta_2[256];

        double ch64w_sin_theta_1[128];
        double ch64w_sin_theta_2[128];
        double ch64w_cos_theta_1[128];
        double ch64w_cos_theta_2[128];

        double cb64s1_A_sin_theta_1[128];
        double cb64s1_A_sin_theta_2[128];
        double cb64s1_A_cos_theta_1[128];
        double cb64s1_A_cos_theta_2[128];

        double ch16x1_sin_theta_1[16];
        double ch16x1_sin_theta_2[16];
        double ch16x1_cos_theta_1[16];
        double ch16x1_cos_theta_2[16];

        double cx126s3_sin_theta_1[126];
        double cx126s3_sin_theta_2[126];
        double cx126s3_cos_theta_1[126];
        double cx126s3_cos_theta_2[126];

        double cx6s3_sin_theta_1[6];
        double cx6s3_sin_theta_2[6];
        double cx6s3_cos_theta_1[6];
        double cx6s3_cos_theta_2[6];

        double x, y, z;
        double sin_theat;
        double cos_theat;
        double _R_;

        double cos_xita;
        double sin_xita;
        double cos_H_xita;
        double sin_H_xita;
        double cos_xita_F;
        double sin_xita_F;
        double add_distance;
    };

    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;
    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_driver::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (float, time, time)
)

#endif // _LSLIDAR_Ch_DRIVER_H_
