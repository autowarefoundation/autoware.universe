//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_CALLBACK_H
#define UBLOX_GPS_CALLBACK_H

#include <ros/console.h>
#include <ublox/serialization/ublox_msgs.h>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace ublox_gps {

/**
 * @brief A callback handler for a u-blox message.
 */
class CallbackHandler {
 public:
  /**
   * @brief Decode the u-blox message.
   */
  virtual void handle(ublox::Reader& reader) = 0;

  /**
   * @brief Wait for on the condition.
   */
  bool wait(const boost::posix_time::time_duration& timeout) {
    boost::mutex::scoped_lock lock(mutex_);
    return condition_.timed_wait(lock, timeout);
  }

 protected:
  boost::mutex mutex_;                   //!< Lock for the handler
  boost::condition_variable condition_;  //!< Condition for the handler lock
};

/**
 * @brief A callback handler for a u-blox message.
 * @typedef T the message type
 */
template <typename T>
class CallbackHandler_ : public CallbackHandler {
 public:
  typedef boost::function<void(const T&)> Callback;  //!< A callback function

  /**
   * @brief Initialize the Callback Handler with a callback function
   * @param func a callback function for the message, defaults to none
   */
  CallbackHandler_(const Callback& func = Callback()) : func_(func) {}

  /**
   * @brief Get the last received message.
   */
  virtual const T& get() { return message_; }

  /**
   * @brief Decode the U-Blox message & call the callback function if it exists.
   * @param reader a reader to decode the message buffer
   */
  void handle(ublox::Reader& reader) {
    boost::mutex::scoped_lock lock(mutex_);
    try {
      if (!reader.read<T>(message_)) {
        ROS_DEBUG_COND(debug >= 2, "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)",
                       static_cast<unsigned int>(reader.classId()), static_cast<unsigned int>(reader.messageId()),
                       reader.length());
        condition_.notify_all();
        return;
      }
    } catch (std::runtime_error& e) {
      ROS_DEBUG_COND(debug >= 2, "U-Blox Decoder error for 0x%02x / 0x%02x (%d bytes)",
                     static_cast<unsigned int>(reader.classId()), static_cast<unsigned int>(reader.messageId()),
                     reader.length());
      condition_.notify_all();
      return;
    }

    if (func_) func_(message_);
    condition_.notify_all();
  }

 private:
  Callback func_;  //!< the callback function to handle the message
  T message_;      //!< The last received message
};

/**
 * @brief Callback handlers for incoming u-blox messages.
 */
class CallbackHandlers {
 public:
  /**
   * @brief Add a callback handler for the given message type.
   * @param callback the callback handler for the message
   * @typedef.a ublox_msgs message with CLASS_ID and MESSAGE_ID constants
   */
  template <typename T>
  void insert(typename CallbackHandler_<T>::Callback callback) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    callbacks_.insert(
        std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID), boost::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Add a callback handler for the given message type and ID. This is
   * used for messages in which have the same structure (and therefore msg file)
   * and same class ID but different message IDs. (e.g. INF, ACK)
   * @param callback the callback handler for the message
   * @param message_id the ID of the message
   * @typedef.a ublox_msgs message with a CLASS_ID constant
   */
  template <typename T>
  void insert(typename CallbackHandler_<T>::Callback callback, unsigned int message_id) {
    boost::mutex::scoped_lock lock(callback_mutex_);
    CallbackHandler_<T>* handler = new CallbackHandler_<T>(callback);
    callbacks_.insert(
        std::make_pair(std::make_pair(T::CLASS_ID, message_id), boost::shared_ptr<CallbackHandler>(handler)));
  }

  /**
   * @brief Calls the callback handler for the message in the reader.
   * @param reader a reader containing a u-blox message
   */
  void handle(ublox::Reader& reader) {
    // Find the callback handlers for the message & decode it
    boost::mutex::scoped_lock lock(callback_mutex_);
    Callbacks::key_type key = std::make_pair(reader.classId(), reader.messageId());
    for (Callbacks::iterator callback = callbacks_.lower_bound(key); callback != callbacks_.upper_bound(key);
         ++callback)
      callback->second->handle(reader);
  }

  /**
   * @brief Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename T>
  bool read(T& message, const boost::posix_time::time_duration& timeout) {
    bool result = false;
    // Create a callback handler for this message
    callback_mutex_.lock();
    CallbackHandler_<T>* handler = new CallbackHandler_<T>();
    Callbacks::iterator callback = callbacks_.insert(
        (std::make_pair(std::make_pair(T::CLASS_ID, T::MESSAGE_ID), boost::shared_ptr<CallbackHandler>(handler))));
    callback_mutex_.unlock();

    // Wait for the message
    if (handler->wait(timeout)) {
      message = handler->get();
      result = true;
    }

    // Remove the callback handler
    callback_mutex_.lock();
    callbacks_.erase(callback);
    callback_mutex_.unlock();
    return result;
  }

  /**
   * @brief Processes u-blox messages in the given buffer & clears the read
   * messages from the buffer.
   * @param data the buffer of u-blox messages to process
   * @param size the size of the buffer
   */
  void readCallback(unsigned char* data, std::size_t& size) {
    ublox::Reader reader(data, size);
    // Read all U-Blox messages in buffer
    while (reader.search() != reader.end() && reader.found()) {
      if (debug >= 3) {
        // Print the received bytes
        std::ostringstream oss;
        for (ublox::Reader::iterator it = reader.pos(); it != reader.pos() + reader.length() + 8; ++it)
          oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
        ROS_DEBUG("U-blox: reading %d bytes\n%s", reader.length() + 8, oss.str().c_str());
      }

      handle(reader);
    }

    // delete read bytes from ASIO input buffer
    std::copy(reader.pos(), reader.end(), data);
    size -= reader.pos() - data;
  }

 private:
  typedef std::multimap<std::pair<uint8_t, uint8_t>, boost::shared_ptr<CallbackHandler> > Callbacks;

  // Call back handlers for u-blox messages
  Callbacks callbacks_;
  boost::mutex callback_mutex_;
};

}  // namespace ublox_gps

#endif  // UBLOX_GPS_CALLBACK_H
