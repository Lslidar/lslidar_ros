/******************************************************************************
 * This file is part of lslidar_ls_driver.
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
#define PacketProcess 
#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H

#include <unistd.h>
#include <cstdio>
#include <netinet/in.h>
#include <string>
#include <memory>
#include <thread>
#include <regex>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/register_point_struct.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <chrono>
#include <lslidar_ls_driver/LslidarLsPacket.h>
#include <lslidar_ls_driver/angle_distortion_correction.h>
#include <lslidar_ls_driver/frame_rate.h>
#include <lslidar_ls_driver/data_port.h>
#include <lslidar_ls_driver/dev_port.h>
#include <lslidar_ls_driver/data_ip.h>
#include <lslidar_ls_driver/destination_ip.h>
#include <lslidar_ls_driver/invalid_data.h>
#include <lslidar_ls_driver/standby_mode.h>
#include <lslidar_ls_driver/time_service.h>
#include <deque>
#include <mutex>
#include "lslidar_ls_driver/ThreadPool.h"

#include "input.h"

namespace lslidar_ch_driver {
    /** Special Defines for LSCh support **/
    constexpr int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    constexpr int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    constexpr double SINGLE_ECHO = 0.006711409;
    constexpr double DOUBLE_ECHO = 0.01010101;

    static float g_fDistanceAcc = 0.001f;
    static float m_offset = 6.37f;
    constexpr double cos30 = cos(DEG2RAD(30));
    constexpr double sin30 = sin(DEG2RAD(30));
    constexpr double cos60 = cos(DEG2RAD(60));
    constexpr double sin60 = sin(DEG2RAD(60));

    struct PointXYZIRT {
        PCL_ADD_POINT4D;  // 添加 x, y, z 和 padding (通常名为 data[4])
        PCL_ADD_INTENSITY;
        uint16_t ring;
        float timestamp;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保动态分配时正确对齐

        // 构造
        PointXYZIRT(float x, float y, float z, float intensity, uint16_t ring, float timestamp)
        : x(x), y(y), z(z), intensity(intensity), ring(ring), timestamp(timestamp) {}

    } EIGEN_ALIGN16;  // 确保整个结构体按16字节对齐

    struct Firing {
        double vertical_angle;
        double azimuth;
        double distance;
        float intensity;
        float time;
        int channel_number;
    };

    class LslidarChDriver {
    public:
        LslidarChDriver(ros::NodeHandle &n, ros::NodeHandle &pn);

        ~LslidarChDriver();

        bool initialize();

        bool polling();

        void difopPoll();

        void initTimeStamp();

        bool isPointInRange(const double &distance) const {
            return (distance >= min_range && distance <= max_range);
        }

        std::function<int(const struct Firing &)> lidarConvertCoordinate;

        inline int convertCoordinate(const struct Firing &lidardata);

        inline int convertCoordinateDistortion(const struct Firing &lidardata);

        // Publish data
        void publishPointCloudNew();

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        std::function<void(const lslidar_ls_driver::LslidarLsPacketPtr&)> lslidarPacketProcess;

        void packetProcessSingle(const lslidar_ls_driver::LslidarLsPacketPtr& packet);
        
        void packetProcessDouble(const lslidar_ls_driver::LslidarLsPacketPtr& packet);

        void checkPacketLoss(const lslidar_ls_driver::LslidarLsPacketPtr &msg, int data_offset, int byte_count);

        void updateTimeOffsets(double point_interval_time,int point_size);

        bool getLidarInformation();

        typedef boost::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef boost::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    public:
        bool loadParameters();

        bool createRosIO();
        
        bool setAngleDistortionCorrection(lslidar_ls_driver::angle_distortion_correction::Request &req,
                                          lslidar_ls_driver::angle_distortion_correction::Response &res);

        bool frameRate(lslidar_ls_driver::frame_rate::Request &req,
                        lslidar_ls_driver::frame_rate::Response &res);

        bool setDataIp(lslidar_ls_driver::data_ip::Request &req,
                         lslidar_ls_driver::data_ip::Response &res);

        bool setDestinationIp(lslidar_ls_driver::destination_ip::Request &req,
                        lslidar_ls_driver::destination_ip::Response &res);

        bool setDataPort(lslidar_ls_driver::data_port::Request &req,
                          lslidar_ls_driver::data_port::Response &res);

        bool setDevPort(lslidar_ls_driver::dev_port::Request &req,
                         lslidar_ls_driver::dev_port::Response &res);

        bool setInvalidData(lslidar_ls_driver::invalid_data::Request &req,
                            lslidar_ls_driver::invalid_data::Response &res);
        
        bool setStandbyMode(lslidar_ls_driver::standby_mode::Request &req,
                            lslidar_ls_driver::standby_mode::Response &res);

        bool setTimeService(lslidar_ls_driver::time_service::Request &req,
                            lslidar_ls_driver::time_service::Response &res);

        //socket Parameters
        int msop_udp_port{};
        int difop_udp_port{};

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;

        // Converter convtor_
        std::shared_ptr<std::thread> difop_thread_;

        // Ethernet relate variables
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;

        in_addr lidar_ip{};
        int socket_id;
        bool add_multicast{};
        double prism_angle[4]{};

        // ROS related variables
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        bool use_time_service;
        uint64_t pointcloudTimeStamp{};
        unsigned char packetTimeStamp[10]{};
        unsigned char difop_data[1206]{};

        // Configuration parameters
        double min_range;
        double max_range;
        double packet_rate;

        ros::Time packet_timeStamp;
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;
        double packet_interval_time;
        // double point_interval_time;
        double point_cloud_timestamp;
        std::vector<double> precomputed_point_times;  // 预计算点时间的数组
        
        int return_mode;
        int scan_start_angle{};
        int scan_end_angle{};
        double g_fAngleAcc_V;
        bool is_add_frame_;
        bool is_get_difop_;
        bool packet_loss;
        std::mutex pc_mutex_;
        
        std::string pointcloud_topic;
        ros::Publisher point_cloud_pub;
        ros::Publisher packet_loss_pub;
        ros::Publisher fault_code_pub;
        ros::ServiceServer angle_distortion_correction_service_;
        ros::ServiceServer frame_rate_service_;         //雷达转速设置服务
        ros::ServiceServer data_ip_service_;            //数据包ip
        ros::ServiceServer destination_ip_service_;     //设备包ip
        ros::ServiceServer data_port_service_;          //数据包端口
        ros::ServiceServer dev_port_service_;           //设备包端口
        ros::ServiceServer invalid_data_service_;       // 无效数据
        ros::ServiceServer standby_mode_service_;       // 待机模式
        ros::ServiceServer time_service_; 
        
        int64_t last_packet_number_;
        // int64_t current_packet_number_;
        int64_t tmp_packet_number_;
        int64_t total_packet_loss_;
        int frame_count;
        int m_horizontal_point = -1;
        bool get_ms06_param;
        //double m_offset;

        double cos_table[36000]{};
        double sin_table[36000]{};
        double cos_mirror_angle[4]{};
        double sin_mirror_angle[4]{};
        std::unique_ptr <ThreadPool> threadPool_;

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;
    };
    typedef PointXYZIRT VPoint;
    typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
    typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;

//    typedef pcl::PointCloud<VPoint> VPointCloud;

} // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_driver::PointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (float, timestamp, timestamp)
)

#endif // _LSLIDAR_Ch_DRIVER_H_