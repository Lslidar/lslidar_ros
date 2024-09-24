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


#include "lslidar_ls_driver/input.h"

extern volatile sig_atomic_t flag;
namespace lslidar_ch_driver {
    static const size_t packet_size = sizeof(lslidar_ls_driver::LslidarLsPacket().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
    Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port) {
        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;

        private_nh.param("lidar_ip", devip_str_, std::string(""));
        private_nh.param<bool>("add_multicast", add_multicast, false);
        private_nh.param<std::string>("group_ip", group_ip, "224.1.1.2");

        // if (!devip_str_.empty())
        //     ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
    }

    int Input::getRpm(void) {
        return cur_rpm_;
    }

    int Input::getReturnMode(void) {
        return return_mode_;
    }

    bool Input::getUpdateFlag(void) {
        return npkt_update_flag_;
    }

    void Input::clearUpdateFlag(void) {
        npkt_update_flag_ = false;
    }
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port) {

        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }

        ROS_INFO_STREAM("Opening UDP socket port: " << port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket");  // TODO: ROS_ERROR errno
            return;
        }
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0){
            perror("setsockopt error!\n");
            return;
        }
        sockaddr_in my_addr{};                   // my address information
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
        my_addr.sin_family = AF_INET;          // host byte order
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // automatically fill in my IP

        if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");  // TODO: ROS_ERROR errno
            return;
        }
        if (add_multicast) {
            struct ip_mreq group{};
            //char *group_ip_ = (char *) group_ip.c_str();
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            //group.imr_interface.s_addr =  INADDR_ANY;
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            //group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                perror("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }


        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
            perror("non-block");
            return;
        }

        efd = epoll_create1(0);
        if (efd ==-1) {
            perror("Failed to create epoll file descriptor");
            return;
        }

        struct epoll_event ev;
        ev.events = EPOLLIN;
        ev.data.fd = sockfd_;
        if (epoll_ctl(efd, EPOLL_CTL_ADD, sockfd_, &ev) == -1) {
            perror("Failed to add socket to epoll");
            close(efd);
            return;
        }
    }

/** @brief destructor */
    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

#if 0
    int InputSocket::getPacket(lslidar_ls_driver::LslidarLsPacketPtr &packet) {
        struct epoll_event events[1];
        int nfds = epoll_wait(efd, events, 1, 3000);
        
        if (nfds <= 0) {
          if (nfds == 0) {
            ROS_WARN("lslidar poll() timeout, port:%d", port_);
            return 1;
          } else if (errno != EINTR) {
            ROS_ERROR("poll() error: %s", strerror(errno));
          }
            return 1;
        }          

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);


        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(sockfd_, &packet->data[0], PACKET_SIZE, 0,
                                    (sockaddr *) &sender_address, &sender_address_len);

        if (nbytes == PACKET_SIZE && sender_address.sin_addr.s_addr == devip_.s_addr) {
            return 0; // Success
        } else {
            if (nbytes == PACKET_SIZE && sender_address.sin_addr.s_addr != devip_.s_addr) ROS_WARN_THROTTLE(2, "lidar ip  parameter error, please reset lidar ip in launch file.");
            if (nbytes < 0 && errno != EWOULDBLOCK) {
                perror("recvfail");
                ROS_INFO("recvfail");
                return -1;
            }
        }

        return 1;
    }
#else
    int InputSocket::getPacket(lslidar_ls_driver::LslidarLsPacketPtr &packet) {
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 3000; // one second (in msec)

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);

        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                ROS_WARN("lslidar poll() timeout, port:%d",port_);
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                || (fds[0].revents & POLLHUP)
                || (fds[0].revents & POLLNVAL)) // device error?
            {
                ROS_ERROR("poll() reports lslidar error");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(sockfd_, &packet->data[0], PACKET_SIZE, 0,
                                    (sockaddr *) &sender_address, &sender_address_len);

        if (nbytes == PACKET_SIZE && sender_address.sin_addr.s_addr == devip_.s_addr) {
            return 0; // Success
        } else {
            if (nbytes == PACKET_SIZE && sender_address.sin_addr.s_addr != devip_.s_addr) ROS_WARN_THROTTLE(2, "lidar ip  parameter error, please reset lidar ip in launch file.");
            if (nbytes < 0 && errno != EWOULDBLOCK) {
                perror("recvfail");
                ROS_INFO("recvfail");
                return -1;
            }
        }

        return 1;
    }
#endif

    InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                         bool read_once, bool read_fast, double repeat_delay) : Input(private_nh, port),
                                                                                packet_rate_(packet_rate),
                                                                                filename_(filename) {
        pcap_ = nullptr;
        empty_ = true;
        private_nh.param("read_once", read_once_, false);
        private_nh.param("read_fast", read_fast_, false);
        private_nh.param("repeat_delay", repeat_delay_, 0.0);
        if (read_once_)
            ROS_INFO("Read input file only once.");
        if (read_fast_)
            ROS_INFO("Read input file as  quickly as possible.");
        if (repeat_delay_ > 0.0)
            ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);
        ROS_INFO_STREAM("Opening PCAP file: " << filename_);
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == nullptr) {
            ROS_FATAL("Error opening lslidar socket dump file.");
            return;
        }
        std::stringstream filter;
        if (devip_str_ != "") {
            filter << "src host " << devip_str_ << "&&";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

    InputPCAP::~InputPCAP() {
        pcap_close(pcap_);
    }

    int InputPCAP::getPacket(lslidar_ls_driver::LslidarLsPacketPtr &pkt) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;
        static int count_frame= 0;
        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
//                ROS_INFO("read pcap file count = %d",count_frame);
                count_frame++;
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data))) {
                    continue;
                }

                if (read_fast_ == false) {
                    packet_rate_.sleep();
                }

                mempcpy(&pkt->data[0], pkt_data + 42, packet_size);
                if (pkt->data[0] == 0x00 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 &&
                    pkt->data[3] == 0x5A) {
                    int rpm = (pkt->data[8] << 8) | pkt_data[9];
                    int mode = 1;
                    if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) ||
                        (pkt->data[45] > 0x08) ||
                        (pkt_data[45] == 0x08 && pkt->data[46] > 0x02)) {
                        if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02) {
                            mode = 0;
                        }

                    }
                    if (cur_rpm_ != rpm || return_mode_ != mode) {
                        cur_rpm_ = rpm;
                        return_mode_ = mode;
                        npkt_update_flag_ = true;
                    }
                }
                pkt->stamp = ros::Time::now();
                empty_ = false;
                return 0;
            }
            if (empty_) {
                ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
            }
            if (read_once_) {
                ROS_INFO("end of file reached-- done reading.");
            }
            if (repeat_delay_ > 0.0) {
                ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            ROS_DEBUG("replaying lslidar dump file");
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
//            ROS_INFO("replaying lslidar dump file");
            count_frame = 0;
        }
        if (flag == 0) {
            abort();
        }
        return -1;
    }


}  //namespace
