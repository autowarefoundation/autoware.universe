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
// AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_H
#define UBLOX_GPS_H
// STL
#include <locale>
#include <map>
#include <stdexcept>
#include <vector>
// Boost
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/atomic.hpp>
// ROS
#include <ros/console.h>
// Other u-blox packages
#include <ublox/serialization/ublox_msgs.h>
// u-blox gps
#include <ublox_gps/async_worker.h>
#include <ublox_gps/callback.h>

/**
 * @namespace ublox_gps
 * This namespace is for I/O communication with the u-blox device, including
 * read callbacks.
 */
namespace ublox_gps {
//! Possible baudrates for u-blox devices
constexpr static unsigned int kBaudrates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};
/**
 * @brief Handles communication with and configuration of the u-blox device
 */
class Gps {
 public:
  //! Sleep time [ms] after setting the baudrate
  constexpr static int kSetBaudrateSleepMs = 500;
  //! Default timeout for ACK messages in seconds
  constexpr static double kDefaultAckTimeout = 1.0;
  //! Size of write buffer for output messages
  constexpr static int kWriterSize = 2056;

  Gps();
  virtual ~Gps();

  /**
   * @brief If called, when the node shuts down, it will send a command to
   * save the flash memory.
   */
  void setSaveOnShutdown(bool save_on_shutdown) { save_on_shutdown_ = save_on_shutdown; }

  /**
   * @brief Set the internal flag for enabling or disabling the initial configurations.
   * @param config_on_startup boolean flag
   */
  void setConfigOnStartup(const bool config_on_startup) { config_on_startup_flag_ = config_on_startup; }

  /**
   * @brief Initialize TCP I/O.
   * @param host the TCP host
   * @param port the TCP port
   */
  void initializeTcp(std::string host, std::string port);

  /**
   * @brief Initialize the Serial I/O port.
   * @param port the device port address
   * @param baudrate the desired baud rate of the port
   * @param uart_in the UART In protocol, see CfgPRT for options
   * @param uart_out the UART Out protocol, see CfgPRT for options
   */
  void initializeSerial(std::string port, unsigned int baudrate, uint16_t uart_in, uint16_t uart_out);

  /**
   * @brief Reset the Serial I/O port after u-blox reset.
   * @param port the device port address
   * @param baudrate the desired baud rate of the port
   * @param uart_in the UART In protocol, see CfgPRT for options
   * @param uart_out the UART Out protocol, see CfgPRT for options
   */
  void resetSerial(std::string port);

  /**
   * @brief Closes the I/O port, and initiates save on shutdown procedure
   * if enabled.
   */
  void close();

  /**
   * @brief Reset I/O communications.
   * @param wait Time to wait before restarting communications
   */
  void reset(const boost::posix_time::time_duration& wait);

  /**
   * @brief Send a reset message to the u-blox device.
   * @param nav_bbr_mask The BBR sections to clear, see CfgRST message
   * @param reset_mode The reset type, see CfgRST message
   * @return true if the message was successfully sent, false otherwise
   */
  bool configReset(uint16_t nav_bbr_mask, uint16_t reset_mode);

  /**
   * @brief Configure the GNSS, cold reset the device, and reset the I/O.
   * @param gnss the desired GNSS configuration
   * @param wait the time to wait after resetting I/O before restarting
   * @return true if the GNSS was configured, the device was reset, and the
   * I/O reset successfully
   */
  bool configGnss(ublox_msgs::CfgGNSS gnss, const boost::posix_time::time_duration& wait);

  /**
   * @brief Send a message to the receiver to delete the BBR data stored in
   * flash.
   * @return true if sent message and received ACK, false otherwise
   */
  bool clearBbr();

  /**
   * @brief Configure the UART1 Port.
   * @param baudrate the baudrate of the port
   * @param in_proto_mask the in protocol mask, see CfgPRT message
   * @param out_proto_mask the out protocol mask, see CfgPRT message
   * @return true on ACK, false on other conditions.
   */
  bool configUart1(unsigned int baudrate, uint16_t in_proto_mask, uint16_t out_proto_mask);

  /**
   * @brief Disable the UART Port. Sets in/out protocol masks to 0. Does not
   * modify other values.
   * @param prev_cfg an empty message which will be filled with the previous
   * configuration parameters
   * @return true on ACK, false on other conditions.
   */
  bool disableUart1(ublox_msgs::CfgPRT& prev_cfg);

  /**
   * @brief Configure the USB Port.
   * @param tx_ready the TX ready pin configuration, see CfgPRT message
   * @param in_proto_mask the in protocol mask, see CfgPRT message
   * @param out_proto_mask the out protocol mask, see CfgPRT message
   * @return true on ACK, false on other conditions.
   */
  bool configUsb(uint16_t tx_ready, uint16_t in_proto_mask, uint16_t out_proto_mask);

  /**
   * @brief Configure the device navigation and measurement rate settings.
   * @param meas_rate Period in milliseconds between subsequent measurements.
   * @param nav_rate the rate at which navigation solutions are generated by the
   * receiver in number measurement cycles
   * @return true on ACK, false on other conditions.
   */
  bool configRate(uint16_t meas_rate, uint16_t nav_rate);

  /**
   * @brief Configure the RTCM messages with the given IDs to the set rate.
   * @param ids the RTCM message ids, valid range: [0, 255]
   * @param rates the send rates for each RTCM message ID, valid range: [0, 255]
   * @return true on ACK, false on other conditions.
   */
  bool configRtcm(std::vector<uint8_t> ids, std::vector<uint8_t> rates);

  /**
   * @brief Configure the SBAS settings.
   * @param enable If true, enable SBAS. Deprecated in firmware 8, use CfgGNSS
   * instead.
   * @param usage SBAS usage, see CfgSBAS for options
   * @param max_sbas Maximum Number of SBAS prioritized tracking channels
   * @return true on ACK, false on other conditions.
   */
  bool configSbas(bool enable, uint8_t usage, uint8_t max_sbas);

  /**
   * @brief Set the TMODE3 settings to fixed.
   *
   * @details Sets the at the given antenna reference point (ARP) position in
   * either Latitude Longitude Altitude (LLA) or ECEF coordinates.
   * @param arp_position a vector of size 3 representing the ARP position in
   * ECEF coordinates [m] or LLA coordinates [deg]
   * @param arp_position_hp a vector of size 3 a vector of size 3 representing
   * the ARP position in ECEF coordinates [0.1 mm] or LLA coordinates
   * [deg * 1e-9]
   * @param lla_flag true if position is given in LAT/LON/ALT, false if ECEF
   * @param fixed_pos_acc Fixed position 3D accuracy [m]
   * @return true on ACK, false if settings are incorrect or on other conditions
   */
  bool configTmode3Fixed(bool lla_flag, std::vector<float> arp_position, std::vector<int8_t> arp_position_hp,
                         float fixed_pos_acc);

  /**
   * @brief Set the TMODE3 settings to survey-in.
   * @param svin_min_dur Survey-in minimum duration [s]
   * @param svin_acc_limit Survey-in position accuracy limit [m]
   * @return true on ACK, false on other conditions.
   */
  bool configTmode3SurveyIn(unsigned int svin_min_dur, float svin_acc_limit);

  /**
   * @brief Set the TMODE3 settings to disabled. Should only be called for
   * High Precision GNSS devices, otherwise the device will return a NACK.
   * @return true on ACK, false on other conditions.
   */
  bool disableTmode3();

  /**
   * @brief Set the rate at which the U-Blox device sends the given message
   * @param class_id the class identifier of the message
   * @param message_id the message identifier
   * @param rate the updated rate in Hz
   * @return true on ACK, false on other conditions.
   */
  bool setRate(uint8_t class_id, uint8_t message_id, uint8_t rate);

  /**
   * @brief Set the device dynamic model.
   * @param model Dynamic model to use. Consult ublox protocol spec for details.
   * @return true on ACK, false on other conditions.
   */
  bool setDynamicModel(uint8_t model);

  /**
   * @brief Set the device fix mode.
   * @param mode 2D only, 3D only or auto.
   * @return true on ACK, false on other conditions.
   */
  bool setFixMode(uint8_t mode);

  /**
   * @brief Set the dead reckoning time limit
   * @param limit Time limit in seconds.
   * @return true on ACK, false on other conditions.
   */
  bool setDeadReckonLimit(uint8_t limit);

  /**
   * @brief Enable or disable PPP (precise-point-positioning).
   * @param enable If true, enable PPP.
   * @return true on ACK, false on other conditions.
   *
   * @note This is part of the expert settings. It is recommended you check
   * the ublox manual first.
   */
  bool setPpp(bool enable);

  /**
   * @brief Set the DGNSS mode (see CfgDGNSS message for details).
   * @param mode the DGNSS mode (see CfgDGNSS message for options)
   * @return true on ACK, false on other conditions
   */
  bool setDgnss(uint8_t mode);

  /**
   * @brief Enable or disable ADR (automotive dead reckoning).
   * @param enable If true, enable ADR.
   * @return true on ACK, false on other conditions.
   */
  bool setUseAdr(bool enable);

  /**
   * @brief Configure the U-Blox to UTC time
   * @return true on ACK, false on other conditions.
   *
   * @note This is part of the expert settings. It is recommended you check
   * the ublox manual first.
   */
  bool setUTCtime();

  /**
   * @brief Enable or disable TIM-TM2 (time mark message).
   * @param enable If true, enable TIM-TM2.
   * @return true on ACK, false on other conditions.
   *
   * @note This is part of the expert settings. It is recommended you check
   * the ublox manual first.
   */
  bool setTimtm2(uint8_t rate);

  /**
   * @brief Configure the U-Blox send rate of the message & subscribe to the
   * given message
   * @param the callback handler for the message
   * @param rate the rate in Hz of the message
   */
  template <typename T>
  void subscribe(typename CallbackHandler_<T>::Callback callback, unsigned int rate);
  /**
   * @brief Subscribe to the given Ublox message.
   * @param the callback handler for the message
   */
  template <typename T>
  void subscribe(typename CallbackHandler_<T>::Callback callback);

  /**
   * @brief Subscribe to the message with the given ID. This is used for
   * messages which have the same format but different message IDs,
   * e.g. INF messages.
   * @param the callback handler for the message
   * @param message_id the U-Blox message ID
   */
  template <typename T>
  void subscribeId(typename CallbackHandler_<T>::Callback callback, unsigned int message_id);

  /**
   * Read a u-blox message of the given type.
   * @param message the received u-blox message
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename T>
  bool read(T& message, const boost::posix_time::time_duration& timeout = default_timeout_);

  bool isInitialized() const { return worker_ != 0; }
  bool isConfigured() const { return isInitialized() && configured_; }
  bool isOpen() const { return worker_->isOpen(); }

  /**
   * Poll a u-blox message of the given type.
   * @param message the received u-blox message output
   * @param payload the poll message payload sent to the device
   * defaults to empty
   * @param timeout the amount of time to wait for the desired message
   */
  template <typename ConfigT>
  bool poll(ConfigT& message, const std::vector<uint8_t>& payload = std::vector<uint8_t>(),
            const boost::posix_time::time_duration& timeout = default_timeout_);
  /**
   * Poll a u-blox message.
   * @param class_id the u-blox message class id
   * @param message_id the u-blox message id
   * @param payload the poll message payload sent to the device,
   * defaults to empty
   * @param timeout the amount of time to wait for the desired message
   */
  bool poll(uint8_t class_id, uint8_t message_id, const std::vector<uint8_t>& payload = std::vector<uint8_t>());

  /**
   * @brief Send the given configuration message.
   * @param message the configuration message
   * @param wait if true, wait for an ACK
   * @return true if message sent successfully and either ACK was received or
   * wait was set to false
   */
  template <typename ConfigT>
  bool configure(const ConfigT& message, bool wait = true);

  /**
   * @brief Wait for an acknowledge message until the timeout
   * @param timeout maximum time to wait in seconds
   * @param class_id the expected class ID of the ACK
   * @param msg_id the expected message ID of the ACK
   * @return true if expected ACK received, false otherwise
   */
  bool waitForAcknowledge(const boost::posix_time::time_duration& timeout, uint8_t class_id, uint8_t msg_id);

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  void setRawDataCallback(const Worker::Callback& callback);

 private:
  //! Types for ACK/NACK messages, WAIT is used when waiting for an ACK
  enum AckType {
    NACK,  //! Not acknowledged
    ACK,   //! Acknowledge
    WAIT   //! Waiting for ACK
  };

  //! Stores ACK/NACK messages
  struct Ack {
    AckType type;      //!< The ACK type
    uint8_t class_id;  //!< The class ID of the ACK
    uint8_t msg_id;    //!< The message ID of the ACK
  };

  /**
   * @brief Set the I/O worker
   * @param an I/O handler
   */
  void setWorker(const boost::shared_ptr<Worker>& worker);

  /**
   * @brief Subscribe to ACK/NACK messages and UPD-SOS-ACK messages.
   */
  void subscribeAcks();

  /**
   * @brief Callback handler for UBX-ACK message.
   * @param m the message to process
   */
  void processAck(const ublox_msgs::Ack& m);

  /**
   * @brief Callback handler for UBX-NACK message.
   * @param m the message to process
   */
  void processNack(const ublox_msgs::Ack& m);

  /**
   * @brief Callback handler for UBX-UPD-SOS-ACK message.
   * @param m the message to process
   */
  void processUpdSosAck(const ublox_msgs::UpdSOS_Ack& m);

  /**
   * @brief Execute save on shutdown procedure.
   *
   * @details Execute the procedure recommended in the u-blox 8 documentation.
   * Send a stop message to the receiver and instruct it to dump its
   * current state to the attached flash memory (where fitted) as part of the
   * shutdown procedure. The flash data is automatically retrieved when the
   * receiver is restarted.
   * @return true if the receiver reset & saved the BBR contents to flash
   */
  bool saveOnShutdown();

  //! Processes I/O stream data
  boost::shared_ptr<Worker> worker_;
  //! Whether or not the I/O port has been configured
  bool configured_;
  //! Whether or not to save Flash BBR on shutdown
  bool save_on_shutdown_;
  //!< Whether or not initial configuration to the hardware is done
  bool config_on_startup_flag_;

  //! The default timeout for ACK messages
  static const boost::posix_time::time_duration default_timeout_;
  //! Stores last received ACK accessed by multiple threads
  mutable boost::atomic<Ack> ack_;

  //! Callback handlers for u-blox messages
  CallbackHandlers callbacks_;

  std::string host_, port_;
};

template <typename T>
void Gps::subscribe(typename CallbackHandler_<T>::Callback callback, unsigned int rate) {
  if (!setRate(T::CLASS_ID, T::MESSAGE_ID, rate)) return;
  subscribe<T>(callback);
}

template <typename T>
void Gps::subscribe(typename CallbackHandler_<T>::Callback callback) {
  callbacks_.insert<T>(callback);
}

template <typename T>
void Gps::subscribeId(typename CallbackHandler_<T>::Callback callback, unsigned int message_id) {
  callbacks_.insert<T>(callback, message_id);
}

template <typename ConfigT>
bool Gps::poll(ConfigT& message, const std::vector<uint8_t>& payload, const boost::posix_time::time_duration& timeout) {
  if (!poll(ConfigT::CLASS_ID, ConfigT::MESSAGE_ID, payload)) return false;
  return read(message, timeout);
}

template <typename T>
bool Gps::read(T& message, const boost::posix_time::time_duration& timeout) {
  if (!worker_) return false;
  return callbacks_.read(message, timeout);
}

template <typename ConfigT>
bool Gps::configure(const ConfigT& message, bool wait) {
  if (!worker_) return false;

  // Reset ack
  Ack ack;
  ack.type = WAIT;
  ack_.store(ack, boost::memory_order_seq_cst);

  // Encode the message
  std::vector<unsigned char> out(kWriterSize);
  ublox::Writer writer(out.data(), out.size());
  if (!writer.write(message)) {
    ROS_ERROR("Failed to encode config message 0x%02x / 0x%02x", message.CLASS_ID, message.MESSAGE_ID);
    return false;
  }
  // Send the message to the device
  worker_->send(out.data(), writer.end() - out.data());

  if (!wait) return true;

  // Wait for an acknowledgment and return whether or not it was received
  return waitForAcknowledge(default_timeout_, message.CLASS_ID, message.MESSAGE_ID);
}

}  // namespace ublox_gps

#endif  // UBLOX_GPS_H
