/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
// Copyright (c) 2019, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * tag_serial_driver.cpp
 * Tamagawa IMU Driver
 * Author MapIV Sekino
 * Ver 1.00 2019/4/4
 */

#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"

#include <sys/ioctl.h>

std::string device = "/dev/ttyUSB0";
std::string imu_type = "noGPS";
std::string rate = "50";

struct termios old_conf_tio;
struct termios conf_tio;

int fd;
int counter;
int raw_data;

sensor_msgs::Imu imu_msg;

int serial_setup(const char * device)
{
  int fd = open(device, O_RDWR);
  // fcntl(fd, F_SETFL, 0);
  // tcgetattr(fd, &conf_tio);

  speed_t BAUDRATE = B115200;

  // conf_tio.c_cc[VMIN] = 1;
  // conf_tio.c_cc[VTIME] = 0;

  // conf_tio.c_iflag = IXON | IXOFF ;
  // conf_tio.c_cflag = CLOCAL | CREAD | CSIZE | CS8;
  // conf_tio.c_oflag = ONLCR | OCRNL;

  conf_tio.c_cflag += CREAD;   // 受信有効
  conf_tio.c_cflag += CLOCAL;  // ローカルライン（モデム制御なし）
  conf_tio.c_cflag += CS8;     // データビット:8bit
  conf_tio.c_cflag += 0;       // ストップビット:1bit
  conf_tio.c_cflag += 0;

  cfsetispeed(&conf_tio, BAUDRATE);
  cfsetospeed(&conf_tio, BAUDRATE);

  tcsetattr(fd, TCSANOW, &conf_tio);
  ioctl(fd, TCSETS, &conf_tio);
  return fd;
}

void receive_ver_req(const std_msgs::Int32::ConstPtr & msg)
{
  char ver_req[] = "$TSC,VER*29\x0d\x0a";
  int ver_req_data = write(fd, ver_req, sizeof(ver_req));
  ROS_INFO("Send Version Request:%s", ver_req);
}

void receive_offset_cancel_req(const std_msgs::Int32::ConstPtr & msg)
{
  char offset_cancel_req[32];
  sprintf(offset_cancel_req, "$TSC,OFC,%d\x0d\x0a", msg->data);
  int offset_cancel_req_data = write(fd, offset_cancel_req, sizeof(offset_cancel_req));
  ROS_INFO("Send Offset Cancel Request:%s", offset_cancel_req);
}

void receive_heading_reset_req(const std_msgs::Int32::ConstPtr & msg)
{
  char heading_reset_req[] = "$TSC,HRST*29\x0d\x0a";
  int heading_reset_req_data = write(fd, heading_reset_req, sizeof(heading_reset_req));
  ROS_INFO("Send Heading reset Request:%s", heading_reset_req);
}

void shutdown_cmd(int sig)
{
  tcsetattr(fd, TCSANOW, &old_conf_tio);  // Revert to previous settings
  close(fd);
  ROS_INFO("Port closed");
  ros::shutdown();
}

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "tag_serial_driver", ros::init_options::NoSigintHandler);
//   ros::NodeHandle n;
//   ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
//   ros::Subscriber sub1 = n.subscribe("/tamagawa_imu/receive_ver_req", 10, receive_ver_req);
//   ros::Subscriber sub2 = n.subscribe("/tamagawa_imu/receive_offset_cancel_req", 10, receive_offset_cancel_req);
//   ros::Subscriber sub3 = n.subscribe("/tamagawa_imu/receive_heading_reset_req", 10, receive_heading_reset_req);

//   tcgetattr(fd, &old_conf_tio);  // Last setting value retention

//   // Use configured device
//   if (argc == 4)
//   {
//     device = argv[1];
//     fd = serial_setup(argv[1]);
//     imu_type = argv[2];
//     rate = argv[3];

//     if (fd < 0)
//     {
//       ROS_ERROR("device not found %s", argv[1]);
//       ros::shutdown();
//     }

//     if (imu_type != "withGPS" && imu_type != "noGPS")
//     {
//       ROS_ERROR("Must be withGPS or noGPS");
//       ros::shutdown();
//     }

//   }
//   else if (argc == 3)
//   {
//     device = argv[1];
//     fd = serial_setup(argv[1]);
//     imu_type = argv[2];

//     if (fd < 0)
//     {
//       ROS_ERROR("device not found %s", argv[1]);
//       ros::shutdown();
//     }

//     if (imu_type != "withGPS" && imu_type != "noGPS")
//     {
//       ROS_ERROR("Must be withGPS or noGPS");
//       ros::shutdown();
//     }
//   }
//   else if (argc == 2)
//   {
//     device = argv[1];
//     fd = serial_setup(argv[1]);

//     if (fd < 0)
//     {
//       ROS_ERROR("device not found %s", argv[1]);
//       ros::shutdown();
//     }
//   }
//   else
//   {
//     fd = serial_setup(device.c_str());
//     if (fd < 0)
//     {
//       ROS_ERROR("device not found %s", device.c_str());
//       ros::shutdown();
//     }
//   }

//   ROS_INFO("device=%s imu_type=%s rate=%s", device.c_str(),imu_type.c_str(),rate.c_str());

//   // Data output request to IMU
//   char bin_req[32];
//   memset(bin_req, 0x00, sizeof(bin_req));
//   sprintf(bin_req, "$TSC,BIN,%s\x0d\x0a", rate.c_str());
//   int bin_req_data = write(fd, bin_req, sizeof(bin_req));
//   ROS_INFO("request:%s", bin_req);

//   ros::Rate loop_rate(atoi(rate.c_str())*1.5);

//   imu_msg.orientation.x = 0.0;
//   imu_msg.orientation.y = 0.0;
//   imu_msg.orientation.z = 0.0;
//   imu_msg.orientation.w = 1.0;

//   char rbuf[512];
//   int len;
//   memset(rbuf, 0x00, sizeof(rbuf));

//   while (ros::ok() && (len = read(fd, rbuf, sizeof(rbuf)) > 0))
//   {

//     ROS_INFO("%s",rbuf);

//     // if (len > 0)
//     // {
//     //     if (rbuf[5] == 'B' && rbuf[6] == 'I' && rbuf[7] == 'N' && rbuf[8] == ',')
//     //     {
//     //       imu_msg.header.frame_id = "imu";
//     //       imu_msg.header.stamp = ros::Time::now();

//     //       if (strcmp(imu_type.c_str(), "noGPS") == 0)
//     //       {
//     //         counter = ((rbuf[11] << 24) & 0xFF000000) | ((rbuf[12] << 16) & 0x00FF0000) | ((rbuf[13] << 8) &
//     0x0000FF00) |
//     //                   (rbuf[14] & 0x000000FF);
//     //         raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
//     //         imu_msg.angular_velocity.x =
//     //             raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
//     //         raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
//     //         imu_msg.angular_velocity.y =
//     //             raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
//     //         raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
//     //         imu_msg.angular_velocity.z =
//     //             raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
//     //         raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
//     //         imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
//     //         raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
//     //         imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
//     //         raw_data = ((((rbuf[27] << 8) & 0xFFFFFF00) | (rbuf[28] & 0x000000FF)));
//     //         imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

//     //         pub.publish(imu_msg);
//     //       }
//     //       else if (strcmp(imu_type.c_str(), "withGPS") == 0)
//     //       {
//     //         counter = ((rbuf[11] << 8) & 0x0000FF00) | (rbuf[12] & 0x000000FF);
//     //         raw_data = ((((rbuf[15] << 8) & 0xFFFFFF00) | (rbuf[16] & 0x000000FF)));
//     //         imu_msg.angular_velocity.x =
//     //             raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
//     //         raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
//     //         imu_msg.angular_velocity.y =
//     //             raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
//     //         raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
//     //         imu_msg.angular_velocity.z =
//     //             raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
//     //         raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
//     //         imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
//     //         raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
//     //         imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
//     //         raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
//     //         imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

//     //         pub.publish(imu_msg);
//     //       }

//     //       ROS_INFO("counter: %d", counter);

//     //     }
//     //     else if (rbuf[5] == 'V' && rbuf[6] == 'E' && rbuf[7] == 'R' && rbuf[8] == ',')
//     //     {
//     //       ROS_INFO("%s", rbuf);
//     //     }
//     // }
//     memset(rbuf, 0x00, sizeof(rbuf));
//     signal(SIGINT, shutdown_cmd);
//     ros::spinOnce();
//     loop_rate.sleep();
//   }

//   return 0;
// }
#include <boost/asio.hpp>
using namespace boost::asio;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tag_serial_driver", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
  ros::Subscriber sub1 = n.subscribe("receive_ver_req", 10, receive_ver_req);
  ros::Subscriber sub2 = n.subscribe("receive_offset_cancel_req", 10, receive_offset_cancel_req);
  ros::Subscriber sub3 = n.subscribe("receive_heading_reset_req", 10, receive_heading_reset_req);

  ros::NodeHandle nh("~");
  std::string imu_frame_id = "imu";
  nh.getParam("imu_frame_id", imu_frame_id);

  std::string port = "/dev/ttyUSB0";
  nh.getParam("port", port);

  io_service io;
  serial_port serial_port(io, port.c_str());
  serial_port.set_option(serial_port_base::baud_rate(115200));
  serial_port.set_option(serial_port_base::character_size(8));
  serial_port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
  serial_port.set_option(serial_port_base::parity(serial_port_base::parity::none));
  serial_port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));

  std::string wbuf = "$TSC,BIN,30\x0d\x0a";
  std::size_t length;
  serial_port.write_some(buffer(wbuf));

  ros::Rate loop_rate(30);

  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  while (ros::ok()) {
    ros::spinOnce();
    // loop_rate.sleep();

    boost::asio::streambuf response;
    boost::asio::read_until(serial_port, response, "\n");
    std::string rbuf(
      boost::asio::buffers_begin(response.data()), boost::asio::buffers_end(response.data()));

    length = rbuf.size();
    size_t len = response.size();
    // std::cout << rbuf  << "    " << length <<  " " << len << std::endl;

    if (length > 0) {
      if (rbuf[5] == 'B' && rbuf[6] == 'I' && rbuf[7] == 'N' && rbuf[8] == ',' && length == 58) {
        imu_msg.header.frame_id = imu_frame_id;
        imu_msg.header.stamp = ros::Time::now();

        counter = ((rbuf[11] << 8) & 0x0000FF00) | (rbuf[12] & 0x000000FF);
        raw_data = ((((rbuf[15] << 8) & 0xFFFFFF00) | (rbuf[16] & 0x000000FF)));
        imu_msg.angular_velocity.x =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[17] << 8) & 0xFFFFFF00) | (rbuf[18] & 0x000000FF)));
        imu_msg.angular_velocity.y =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[19] << 8) & 0xFFFFFF00) | (rbuf[20] & 0x000000FF)));
        imu_msg.angular_velocity.z =
          raw_data * (200 / pow(2, 15)) * M_PI / 180;  // LSB & unit [deg/s] => [rad/s]
        raw_data = ((((rbuf[21] << 8) & 0xFFFFFF00) | (rbuf[22] & 0x000000FF)));
        imu_msg.linear_acceleration.x = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[23] << 8) & 0xFFFFFF00) | (rbuf[24] & 0x000000FF)));
        imu_msg.linear_acceleration.y = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]
        raw_data = ((((rbuf[25] << 8) & 0xFFFFFF00) | (rbuf[26] & 0x000000FF)));
        imu_msg.linear_acceleration.z = raw_data * (100 / pow(2, 15));  // LSB & unit [m/s^2]

        pub.publish(imu_msg);

      } else if (rbuf[5] == 'V' && rbuf[6] == 'E' && rbuf[7] == 'R' && rbuf[8] == ',') {
        // ROS_INFO("%s", rbuf.c_str());
      }
    }
  }

  return 0;
}
