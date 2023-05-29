// Copyright 2020 PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PAL Robotics S.L. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 * @author Jeremie Deray
 * @author Brighten Lee
 * @author Hongrui Zheng
 */

#ifndef ACKERMANN_MUX__TOPIC_HANDLE_HPP_
#define ACKERMANN_MUX__TOPIC_HANDLE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <ackermann_mux/utils.hpp>
#include <ackermann_mux/ackermann_mux.hpp>

#include <memory>
#include <string>
#include <vector>

namespace ackermann_mux
{
template<typename T>
class TopicHandle_
{
public:
  // Not copy constructible
  TopicHandle_(TopicHandle_ &) = delete;
  TopicHandle_(const TopicHandle_ &) = delete;

  // Not copy assignable
  TopicHandle_ & operator=(TopicHandle_ &) = delete;
  TopicHandle_ & operator=(const TopicHandle_ &) = delete;

  typedef int priority_type;

  /**
   * @brief TopicHandle_
   * @param nh Node handle
   * @param name Name identifier
   * @param topic Topic name
   * @param timeout Timeout to consider that the messages are old; note
   * that initially the message stamp is set to 0.0, so the message has
   * expired
   * @param priority Priority of the topic
   */
  TopicHandle_(
    const std::string & name, const std::string & topic, const rclcpp::Duration & timeout,
    priority_type priority, AckermannMux * mux)
  : name_(name),
    topic_(topic),
    timeout_(timeout),
    priority_(clamp(priority, priority_type(0), priority_type(255))),
    mux_(mux),
    stamp_(0)
  {
    RCLCPP_INFO(
      mux_->get_logger(),
      "Topic handler '%s' subscribed to topic '%s': timeout = %s , priority = %d.",
      name_.c_str(), topic_.c_str(),
      ((timeout_.seconds() > 0) ? std::to_string(timeout_.seconds()) + "s" : "None").c_str(),
      static_cast<int>(priority_));
  }

  virtual ~TopicHandle_() = default;

  /**
   * @brief hasExpired
   * @return true if the message has expired; false otherwise.
   *         If the timeout is set to 0.0, this function always returns
   *         false
   */
  bool hasExpired() const
  {
    return (timeout_.seconds() > 0.0) && (
      (mux_->now().seconds() - stamp_.seconds()) > timeout_.seconds());
  }

  const std::string & getName() const
  {
    return name_;
  }

  const std::string & getTopic() const
  {
    return topic_;
  }

  const rclcpp::Duration & getTimeout() const
  {
    return timeout_;
  }

  /**
   * @brief getPriority Priority getter
   * @return Priority
   */
  const priority_type & getPriority() const
  {
    return priority_;
  }

  const T & getStamp() const
  {
    return stamp_;
  }

  const T & getMessage() const
  {
    return msg_;
  }

protected:
  std::string name_;
  std::string topic_;
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;
  rclcpp::Duration timeout_;
  priority_type priority_;

protected:
  AckermannMux * mux_;

  rclcpp::Time stamp_;
  T msg_;
};

class VelocityTopicHandle : public TopicHandle_<ackermann_msgs::msg::AckermannDriveStamped>
{
private:
  typedef TopicHandle_<ackermann_msgs::msg::AckermannDriveStamped> base_type;

  // https://index.ros.org/doc/ros2/About-Quality-of-Service-Settings
  // rmw_qos_profile_t ackermann_qos_profile = rmw_qos_profile_sensor_data;

public:
  typedef typename base_type::priority_type priority_type;

  VelocityTopicHandle(
    const std::string & name, const std::string & topic, const rclcpp::Duration & timeout,
    priority_type priority, AckermannMux * mux)
  : base_type(name, topic, timeout, priority, mux)
  {
    subscriber_ = mux_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&VelocityTopicHandle::callback, this, std::placeholders::_1));

    // subscriber_ = nh_.create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    //    topic_, ackermann_qos_profile,
    //  std::bind(&VelocityTopicHandle::callback, this, std::placeholders::_1));
  }

  bool isMasked(priority_type lock_priority) const
  {
    // std::cout << hasExpired() << " / " << (getPriority() < lock_priority) << std::endl;
    return hasExpired() || (getPriority() < lock_priority);
  }

  void callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg)
  {
    stamp_ = mux_->now();
    msg_ = *msg;

    // Check if this ackermann drive has priority.
    // Note that we have to check all the locks because they might time out
    // and since we have several topics we must look for the highest one in
    // all the topic list; so far there's no O(1) solution.
    if (mux_->hasPriority(*this)) {
      mux_->publishAckermann(msg);
    }
  }
};

class LockTopicHandle : public TopicHandle_<std_msgs::msg::Bool>
{
private:
  typedef TopicHandle_<std_msgs::msg::Bool> base_type;

  // https://index.ros.org/doc/ros2/About-Quality-of-Service-Settings
  // rmw_qos_profile_t lock_qos_profile = rmw_qos_profile_sensor_data;

public:
  typedef typename base_type::priority_type priority_type;

  LockTopicHandle(
    const std::string & name, const std::string & topic, const rclcpp::Duration & timeout,
    priority_type priority, AckermannMux * mux)
  : base_type(name, topic, timeout, priority, mux)
  {
    subscriber_ = mux_->create_subscription<std_msgs::msg::Bool>(
      topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&LockTopicHandle::callback, this, std::placeholders::_1));
  }

  /**
   * @brief isLocked
   * @return true if has expired or locked (i.e. bool message data is true)
   */
  bool isLocked() const
  {
    return hasExpired() || getMessage().data;
  }

  void callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    stamp_ = mux_->now();
    msg_ = *msg;
  }
};

}  // namespace ackermann_mux

#endif  // ACKERMANN_MUX__TOPIC_HANDLE_HPP_
