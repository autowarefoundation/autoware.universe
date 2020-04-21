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

#ifndef UBLOX_GPS_ASYNC_WORKER_H
#define UBLOX_GPS_ASYNC_WORKER_H

#include <ublox_gps/gps.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include "worker.h"

namespace ublox_gps {

int debug;  //!< Used to determine which debug messages to display

/**
 * @brief Handles Asynchronous I/O reading and writing.
 */
template <typename StreamT>
class AsyncWorker : public Worker {
 public:
  typedef boost::mutex Mutex;
  typedef boost::mutex::scoped_lock ScopedLock;

  /**
   * @brief Construct an Asynchronous I/O worker.
   * @param stream the stream for th I/O service
   * @param io_service the I/O service
   * @param buffer_size the size of the input and output buffers
   */
  AsyncWorker(boost::shared_ptr<StreamT> stream, boost::shared_ptr<boost::asio::io_service> io_service,
              std::size_t buffer_size = 8192);
  virtual ~AsyncWorker();

  /**
   * @brief Set the callback function which handles input messages.
   * @param callback the read callback which handles received messages
   */
  void setCallback(const Callback& callback) { read_callback_ = callback; }

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  void setRawDataCallback(const Callback& callback) { write_callback_ = callback; }

  /**
   * @brief Send the data bytes via the I/O stream.
   * @param data the buffer of data bytes to send
   * @param size the size of the buffer
   */
  bool send(const unsigned char* data, const unsigned int size);
  /**
   * @brief Wait for incoming messages.
   * @param timeout the maximum time to wait
   */
  void wait(const boost::posix_time::time_duration& timeout);

  bool isOpen() const { return stream_->is_open(); }

 protected:
  /**
   * @brief Read the input stream.
   */
  void doRead();

  /**
   * @brief Process messages read from the input stream.
   * @param error_code an error code for read failures
   * @param the number of bytes received
   */
  void readEnd(const boost::system::error_code&, std::size_t);

  /**
   * @brief Send all the data in the output buffer.
   */
  void doWrite();

  /**
   * @brief Close the I/O stream.
   */
  void doClose();

  boost::shared_ptr<StreamT> stream_;                      //!< The I/O stream
  boost::shared_ptr<boost::asio::io_service> io_service_;  //!< The I/O service

  Mutex read_mutex_;  //!< Lock for the input buffer
  boost::condition read_condition_;
  std::vector<unsigned char> in_;  //!< The input buffer
  std::size_t in_buffer_size_;     //!< number of bytes currently in the input
                                   //!< buffer

  Mutex write_mutex_;  //!< Lock for the output buffer
  boost::condition write_condition_;
  std::vector<unsigned char> out_;  //!< The output buffer

  boost::shared_ptr<boost::thread> background_thread_;  //!< thread for the I/O
                                                        //!< service
  Callback read_callback_;                              //!< Callback function to handle received messages
  Callback write_callback_;                             //!< Callback function to handle raw data

  bool stopping_;  //!< Whether or not the I/O service is closed
};

template <typename StreamT>
AsyncWorker<StreamT>::AsyncWorker(boost::shared_ptr<StreamT> stream,
                                  boost::shared_ptr<boost::asio::io_service> io_service, std::size_t buffer_size)
    : stopping_(false) {
  stream_ = stream;
  io_service_ = io_service;
  in_.resize(buffer_size);
  in_buffer_size_ = 0;

  out_.reserve(buffer_size);

  io_service_->post(boost::bind(&AsyncWorker<StreamT>::doRead, this));
  background_thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, io_service_)));
}

template <typename StreamT>
AsyncWorker<StreamT>::~AsyncWorker() {
  io_service_->post(boost::bind(&AsyncWorker<StreamT>::doClose, this));
  background_thread_->join();
  // io_service_->reset();
}

template <typename StreamT>
bool AsyncWorker<StreamT>::send(const unsigned char* data, const unsigned int size) {
  ScopedLock lock(write_mutex_);
  if (size == 0) {
    ROS_ERROR("Ublox AsyncWorker::send: Size of message to send is 0");
    return true;
  }

  if (out_.capacity() - out_.size() < size) {
    ROS_ERROR("Ublox AsyncWorker::send: Output buffer too full to send message");
    return false;
  }
  out_.insert(out_.end(), data, data + size);

  io_service_->post(boost::bind(&AsyncWorker<StreamT>::doWrite, this));
  return true;
}

template <typename StreamT>
void AsyncWorker<StreamT>::doWrite() {
  ScopedLock lock(write_mutex_);
  // Do nothing if out buffer is empty
  if (out_.size() == 0) {
    return;
  }
  // Write all the data in the out buffer
  boost::asio::write(*stream_, boost::asio::buffer(out_.data(), out_.size()));

  if (debug >= 2) {
    // Print the data that was sent
    std::ostringstream oss;
    for (std::vector<unsigned char>::iterator it = out_.begin(); it != out_.end(); ++it)
      oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
    ROS_DEBUG("U-Blox sent %li bytes: \n%s", out_.size(), oss.str().c_str());
  }
  // Clear the buffer & unlock
  out_.clear();
  write_condition_.notify_all();
}

template <typename StreamT>
void AsyncWorker<StreamT>::doRead() {
  ScopedLock lock(read_mutex_);
  stream_->async_read_some(boost::asio::buffer(in_.data() + in_buffer_size_, in_.size() - in_buffer_size_),
                           boost::bind(&AsyncWorker<StreamT>::readEnd, this, boost::asio::placeholders::error,
                                       boost::asio::placeholders::bytes_transferred));
}

template <typename StreamT>
void AsyncWorker<StreamT>::readEnd(const boost::system::error_code& error, std::size_t bytes_transfered) {
  ScopedLock lock(read_mutex_);
  if (error) {
    ROS_ERROR("U-Blox ASIO input buffer read error: %s, %li", error.message().c_str(), bytes_transfered);
  } else if (bytes_transfered > 0) {
    in_buffer_size_ += bytes_transfered;

    unsigned char* pRawDataStart = &(*(in_.begin() + (in_buffer_size_ - bytes_transfered)));
    std::size_t raw_data_stream_size = bytes_transfered;

    if (write_callback_) write_callback_(pRawDataStart, raw_data_stream_size);

    if (debug >= 4) {
      std::ostringstream oss;
      for (std::vector<unsigned char>::iterator it = in_.begin() + in_buffer_size_ - bytes_transfered;
           it != in_.begin() + in_buffer_size_; ++it)
        oss << boost::format("%02x") % static_cast<unsigned int>(*it) << " ";
      ROS_DEBUG("U-Blox received %li bytes \n%s", bytes_transfered, oss.str().c_str());
    }

    if (read_callback_) read_callback_(in_.data(), in_buffer_size_);

    read_condition_.notify_all();
  }

  if (!stopping_) io_service_->post(boost::bind(&AsyncWorker<StreamT>::doRead, this));
}

template <typename StreamT>
void AsyncWorker<StreamT>::doClose() {
  ScopedLock lock(read_mutex_);
  stopping_ = true;
  boost::system::error_code error;
  stream_->close(error);
  if (error) ROS_ERROR_STREAM("Error while closing the AsyncWorker stream: " << error.message());
}

template <typename StreamT>
void AsyncWorker<StreamT>::wait(const boost::posix_time::time_duration& timeout) {
  ScopedLock lock(read_mutex_);
  read_condition_.timed_wait(lock, timeout);
}

}  // namespace ublox_gps

#endif  // UBLOX_GPS_ASYNC_WORKER_H
