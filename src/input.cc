/*
 * This file is part of lslidar_ch driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "lslidar_ch_driver/input.h"

extern volatile sig_atomic_t flag;
namespace lslidar_ch_driver {

    //static const size_t ch128x1_packet_size = sizeof(lslidar_ch_driver::LslidarChPacket().data);

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
    Input::Input(ros::NodeHandle private_nh, uint16_t port, int packet_size) : private_nh_(private_nh), port_(port),
                                                                               packet_size_(packet_size) {


        private_nh.param("lidar_ip", devip_str_, std::string(""));
        private_nh.param<bool>("add_multicast", add_multicast, false);

        private_nh.param<std::string>("group_ip", group_ip, "224.1.1.2");
        if (!devip_str_.empty())
            ROS_INFO_STREAM_ONCE("Only accepting packets from IP address: " << devip_str_);
    }


////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port, int packet_size) : Input(private_nh, port,packet_size) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }

        ROS_INFO_STREAM("Opening UDP socket: port " << port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket");  // TODO: ROS_ERROR errno
            return;
        }

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *) &opt, sizeof(opt))) {
            perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                   // my address information
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
        my_addr.sin_family = AF_INET;          // host byte order
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

        if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");  // TODO: ROS_ERROR errno
            return;
        }

        if (add_multicast) {
            struct ip_mreq group;
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
        if (efd == -1) {
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
        close(efd);
        close(sockfd_);
    }

#if 0
    int InputSocket::getPacket(lslidar_ch_driver::LslidarChPacketPtr &packet) {
        struct epoll_event events[1];
        int nfds = epoll_wait(efd, events, 1, 2000);
        if (nfds <= 0) {
            if (nfds == 0) {
                ROS_WARN("lslidar epoll() timeout port: %d", port_);
            } else if (errno != EINTR) {
                ROS_ERROR("epoll() error: %s", strerror(errno));
            }
            return 1;
        }

        sockaddr_in sender_address{}; // 发送者地址
        socklen_t sender_address_len = sizeof(sender_address); // 地址长度

        if (events[0].data.fd == sockfd_) {
            ssize_t nbytes = recvfrom(sockfd_, &packet->data[0], packet_size_, 0, (sockaddr *)&sender_address, &sender_address_len);
            if (nbytes == packet_size_ && sender_address.sin_addr.s_addr == devip_.s_addr) {
                return 0; // 成功接收完整数据包
            } else {
                if (nbytes == packet_size_ && sender_address.sin_addr.s_addr != devip_.s_addr) ROS_WARN_THROTTLE(2, "lidar ip  parameter error, please reset lidar ip in launch file.");
                if (nbytes < 0 && errno != EWOULDBLOCK) {
                    perror("recvfail");
                    ROS_INFO("recvfail");
                    return -1;
                }
            }
        }
        return 1;
    }

#else
    int InputSocket::getPacket(lslidar_ch_driver::LslidarChPacketPtr &packet) {
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 2000; // 2000 milliseconds

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);
        
        int retval = poll(fds, 1, POLL_TIMEOUT);
        
        if (!flag) return -2; 

        if (retval <= 0) { // Handle timeout and poll error in one branch
            if (retval == 0) {
                ROS_WARN("lslidar poll() timeout, port: %d", port_);
            } else if (errno != EINTR) {
                ROS_ERROR("poll() error: %s", strerror(errno));
            }
            return 1;
        }

        ssize_t nbytes = recvfrom(sockfd_, &packet->data[0], packet_size_, 0, (sockaddr *)&sender_address, &sender_address_len);
        if (nbytes == packet_size_ && sender_address.sin_addr.s_addr == devip_.s_addr) {
            return 0; // Success
        } else {
            if (nbytes == packet_size_ && sender_address.sin_addr.s_addr != devip_.s_addr) ROS_WARN_THROTTLE(2, "lidar ip  parameter error, please reset lidar ip in launch file.");
            if (nbytes < 0 && errno != EWOULDBLOCK) {
                perror("recvfail");
                ROS_INFO("recvfail");
                return -1;
            }
        }

        return 1; // If incomplete data received
    }
#endif

    InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, int packet_size, double packet_rate,
                         std::string filename,
                         bool read_once, bool read_fast, double repeat_delay) : Input(private_nh, port, packet_size),
                                                                                packet_rate_(packet_rate),
                                                                                filename_(filename) {
        pcap_ = NULL;
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
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
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

    int InputPCAP::getPacket(lslidar_ch_driver::LslidarChPacketPtr &pkt) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;
        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {

                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data))) {
                    continue;
                }

                if (read_fast_ == false) {
                    packet_rate_.sleep();
                }

                mempcpy(&pkt->data[0], pkt_data + 42, packet_size_);
                empty_ = false;
                return 0;

            }
            if (empty_) {
                ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
            }
            if (read_once_) {
                ROS_INFO("end of file reached-- done reading.");
                return -1;
            }
            if (repeat_delay_ > 0.0) {
                ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            ROS_WARN("replaying lslidar dump file");
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;
        }
        if (flag == 0) {
            abort();
        }
        return 0;
    }
}
