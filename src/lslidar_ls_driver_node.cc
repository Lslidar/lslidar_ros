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

#include <lslidar_ls_driver/lslidar_ls_driver.h>

volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
    exit(0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lslidar_ls_driver_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, my_handler);

    // start the driver
    ROS_INFO("namespace is %s", private_nh.getNamespace().c_str());
    lslidar_ch_driver::LslidarChDriver driver(node, private_nh);
    if (!driver.initialize()) {
        ROS_ERROR("Cannot initialize lslidar driver...");
        return 0;
    }
    // loop until shut down or end of file
    while (ros::ok() && driver.polling()) {
        ros::spinOnce();
    }
    sleep(2);
    return 0;
}
