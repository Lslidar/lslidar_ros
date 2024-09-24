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
 ****************************************************************************
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __LSLIDAR_INPUT_H_
#define __LSLIDAR_INPUT_H_

#include <unistd.h>
#include <cstdio>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <lslidar_ls_driver/LslidarLsPacket.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/epoll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <csignal>
#include <cstring>
#include <cerrno>
#include <sensor_msgs/TimeReference.h>

namespace lslidar_ch_driver {
    static uint16_t PACKET_SIZE = 1206;
    static uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
    static uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
/**
 *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
 * @brief The Input class,
     *
     * @param private_nh  一个NodeHandled,用于通过节点传递参数
     * @param port
     * @returns 0 if successful,
     *          -1 if end of file
     *          >0 if incomplete packet (is this possible?)
 */
    class Input {
    public:
        Input(ros::NodeHandle private_nh, uint16_t port);

        virtual ~Input() {
        }

        virtual int getPacket(lslidar_ls_driver::LslidarLsPacketPtr &packet) = 0;

        int getRpm(void);

        int getReturnMode(void);

        bool getUpdateFlag(void);

        void clearUpdateFlag(void);

    protected:
        ros::NodeHandle private_nh_;
        uint16_t port_;
        std::string devip_str_;
        int cur_rpm_;
        int return_mode_;
        bool npkt_update_flag_;
        bool add_multicast;
        std::string group_ip;
    };

/** @brief Live lslidar input from socket. */
    class InputSocket : public Input {
    public:
        InputSocket(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER);

        virtual ~InputSocket();

        virtual int getPacket(lslidar_ls_driver::LslidarLsPacketPtr &packet);

    private:
        int sockfd_;
        int efd;
        in_addr devip_;
        //struct ip_mreq group;

    };

    class InputPCAP : public Input {
    public:
        InputPCAP(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0, \
    std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

        virtual ~InputPCAP();

        virtual int getPacket(lslidar_ls_driver::LslidarLsPacketPtr &packet);

    private:
        ros::Rate packet_rate_;
        std::string filename_;
        pcap_t *pcap_;
        bpf_program pcap_packet_filter_;
        char errbuf_[PCAP_ERRBUF_SIZE];
        bool empty_;
        bool read_once_;
        bool read_fast_;
        double repeat_delay_;
    };

} //namespace

#endif  // __LSLIDAR_INPUT_H
