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
#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <thread>
#include <memory>
#include <ros/ros.h>
#include <lslidar_msgs/LslidarChPacket.h>
#include <lslidar_msgs/LslidarChScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>


#include "input.h"

namespace lslidar_ch_driver {

    static const double DISTANCE_RESOLUTION = 0.0000390625; /**< meters */
   

    struct PointXYZIRT {
        PCL_ADD_POINT4D
        float intensity;
        uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

    class LslidarChDriver {
    private:
        struct Firing {
            //double vertical_angle;
            int vertical_line;
            double azimuth;
            double distance;
            float intensity;
            double time;
        };

    public:

        LslidarChDriver(ros::NodeHandle &n, ros::NodeHandle &pn);

        ~LslidarChDriver();

        bool initialize();
        void publishLaserScan();
        void publishPointCloud();

        int convertCoordinate(struct Firing lidardata);

        bool polling();

        void difopPoll(void);

        void initTimeStamp(void);

        bool isPointInRange(const double& distance) {
            return (distance >= min_range && distance <= max_range);
        }

        //void getFPGA_GPSTimeStamp(lslidar_msgs::LslidarChPacketPtr &packet);

        typedef std::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
        typedef std::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

    private:


        bool loadParameters();

        bool createRosIO();

        //socket Parameters
        int msop_udp_port;
        int difop_udp_port;

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

        bool add_multicast;
        bool pcl_type;
        std::string dump_file;

        double min_range;
        double max_range;
        double angle_disable_min;
        double angle_disable_max;
        int channel_num;

        int echo_num;

        double horizontal_angle_resolution;
        int field_angle_type;


        // ROS related variables
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Publisher pointcloud_pub;
        ros::Publisher laserscan_pub;
        std::mutex pointcloud_lock;
        lslidar_msgs::LslidarChScanPtr sweep_data;
        lslidar_msgs::LslidarChScanPtr sweep_data_bac;



        // add for time synchronization
        bool use_time_service;

        bool publish_laserscan;
        bool is_filter;
        double filter_distance;
        double filter_angle_min;
        double filter_angle_max;
        double filter_intensity;
        int filter_line;
        int line_num;
        int first_frame_flag;
        double sin_azimuth_table[36000];
        double cos_azimuth_table[36000];
        float cos_theta[120]{};
        float sin_theta[120]{};
        float theta_t[120]{};
        float theta_q[120]{};
        bool is_update_difop;
        bool packetType;

        uint64_t packet_timestamp_s;
        uint64_t packet_timestamp_ns;
        double packet_timestamp;
        double last_packet_timestamp;
        double point_cloud_timestamp;
        double point_time;
        double packet_rate;



        unsigned char packetTimeStamp[10];
        struct tm cur_time;

     //   ros::Time timeStamp;

     //   ros::Time packet_timeStamp;

        double packet_interval_time;
     //   Firing firings[171];

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
