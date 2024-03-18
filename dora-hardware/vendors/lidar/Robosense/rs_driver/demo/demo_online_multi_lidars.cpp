/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
#endif

//#define ORDERLY_EXIT

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

class DriverClient
{
public:

  DriverClient(const std::string name)
    : name_(name)
  {
  }

  bool init(const RSDriverParam& param)
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "                      " << name_ << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
    param.print();

    driver_.regPointCloudCallback (std::bind(&DriverClient::driverGetPointCloudFromCallerCallback, this),
                                  std::bind(&DriverClient::driverReturnPointCloudToCallerCallback, this, std::placeholders::_1));
    driver_.regExceptionCallback (std::bind(&DriverClient::exceptionCallback, this, std::placeholders::_1));

    if (!driver_.init(param))
    {
      RS_ERROR << name_ << ": Failed to initialize driver." << RS_REND;
      return false;
    }

    return true;
  }

  bool start()
  {
    to_exit_process_ = false;
    cloud_handle_thread_ = std::thread(std::bind(&DriverClient::processCloud, this));

    driver_.start();
    RS_DEBUG << name_ << ": Started driver." << RS_REND;

    return true;
  }

  void stop()
  {
    driver_.stop();

    to_exit_process_ = true;
    cloud_handle_thread_.join();
  }

protected:

  std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
  {
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
    if (msg.get() != NULL)
    {
      return msg;
    }

    return std::make_shared<PointCloudMsg>();
  }

  void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
  {
    stuffed_cloud_queue_.push(msg);
  }

  void processCloud(void)
  {
    while (!to_exit_process_)
    {
      std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
      if (msg.get() == NULL)
      {
        continue;
      }

      RS_MSG << name_ << ": msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

      free_cloud_queue_.push(msg);
    }
  }

  void exceptionCallback(const Error& code)
  {
    RS_WARNING << name_ << ": " << code.toString() << RS_REND;
  }

  std::string name_;
  LidarDriver<PointCloudMsg> driver_;
  bool to_exit_process_;
  std::thread cloud_handle_thread_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;
};

int main(int argc, char* argv[])
{
  RS_TITLE << "------------------------------------------------------" << RS_REND;
  RS_TITLE << "            RS_Driver Core Version: v" << getDriverVersion() << RS_REND;
  RS_TITLE << "------------------------------------------------------" << RS_REND;

  RSDriverParam param_left;                  ///< Create a parameter object
  param_left.input_type = InputType::ONLINE_LIDAR;
  param_left.input_param.msop_port = 6004;   ///< Set the lidar msop port number, the default is 6699
  param_left.input_param.difop_port = 7004;  ///< Set the lidar difop port number, the default is 7788
  param_left.lidar_type = LidarType::RSM1;   ///< Set the lidar type. Make sure this type is correct

  DriverClient client_left("LEFT ");
  if (!client_left.init(param_left))                         ///< Call the init function
  {
    return -1;
  }

  RSDriverParam param_right;                  ///< Create a parameter object
  param_right.input_type = InputType::ONLINE_LIDAR;
  param_right.input_param.msop_port = 6005;   ///< Set the lidar msop port number, the default is 6699
  param_right.input_param.difop_port = 7005;  ///< Set the lidar difop port number, the default is 7788
  param_right.lidar_type = LidarType::RSM1;   ///< Set the lidar type. Make sure this type is correct

  DriverClient client_right("RIGHT");
  if (!client_right.init(param_right))                         ///< Call the init function
  {
    return -1;
  }

  client_left.start();
  client_right.start();

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  client_left.stop();
  client_right.stop();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}

