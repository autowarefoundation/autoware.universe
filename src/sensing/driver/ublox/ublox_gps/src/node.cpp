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

#include "ublox_gps/node.h"
#include <cmath>
#include <sstream>
#include <string>

using namespace ublox_node;

//
// ublox_node namespace
//
uint8_t ublox_node::modelFromString(const std::string& model) {
  std::string lower = model;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "portable") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_PORTABLE;
  } else if (lower == "stationary") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_STATIONARY;
  } else if (lower == "pedestrian") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_PEDESTRIAN;
  } else if (lower == "automotive") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AUTOMOTIVE;
  } else if (lower == "sea") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_SEA;
  } else if (lower == "airborne1") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_1G;
  } else if (lower == "airborne2") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_2G;
  } else if (lower == "airborne4") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_AIRBORNE_4G;
  } else if (lower == "wristwatch") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_WRIST_WATCH;
  } else if (lower == "bike") {
    return ublox_msgs::CfgNAV5::DYN_MODEL_BIKE;
  }

  throw std::runtime_error("Invalid settings: " + lower + " is not a valid dynamic model.");
}

uint8_t ublox_node::fixModeFromString(const std::string& mode) {
  std::string lower = mode;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "2d") {
    return ublox_msgs::CfgNAV5::FIX_MODE_2D_ONLY;
  } else if (lower == "3d") {
    return ublox_msgs::CfgNAV5::FIX_MODE_3D_ONLY;
  } else if (lower == "auto") {
    return ublox_msgs::CfgNAV5::FIX_MODE_AUTO;
  }

  throw std::runtime_error("Invalid settings: " + mode + " is not a valid fix mode.");
}

//
// u-blox ROS Node
//
UbloxNode::UbloxNode() { initialize(); }

void UbloxNode::addFirmwareInterface() {
  int ublox_version;
  if (protocol_version_ < 14) {
    components_.push_back(ComponentPtr(new UbloxFirmware6));
    ublox_version = 6;
  } else if (protocol_version_ >= 14 && protocol_version_ <= 15) {
    components_.push_back(ComponentPtr(new UbloxFirmware7));
    ublox_version = 7;
  } else if (protocol_version_ > 15 && protocol_version_ <= 23) {
    components_.push_back(ComponentPtr(new UbloxFirmware8));
    ublox_version = 8;
  } else {
    components_.push_back(ComponentPtr(new UbloxFirmware9));
    ublox_version = 9;
  }

  ROS_INFO("U-Blox Firmware Version: %d", ublox_version);
}

void UbloxNode::addProductInterface(std::string product_category, std::string ref_rov) {
  if (product_category.compare("HPG") == 0 && ref_rov.compare("REF") == 0)
    components_.push_back(ComponentPtr(new HpgRefProduct));
  else if (product_category.compare("HPG") == 0 && ref_rov.compare("ROV") == 0)
    components_.push_back(ComponentPtr(new HpgRovProduct));
  else if (product_category.compare("HPG") == 0)
    components_.push_back(ComponentPtr(new HpPosRecProduct));
  else if (product_category.compare("TIM") == 0)
    components_.push_back(ComponentPtr(new TimProduct));
  else if (product_category.compare("ADR") == 0 || product_category.compare("UDR") == 0)
    components_.push_back(ComponentPtr(new AdrUdrProduct));
  else if (product_category.compare("FTS") == 0)
    components_.push_back(ComponentPtr(new FtsProduct));
  else if (product_category.compare("SPG") != 0)
    ROS_WARN("Product category %s %s from MonVER message not recognized %s", product_category.c_str(), ref_rov.c_str(),
             "options are HPG REF, HPG ROV, HPG #.#, TIM, ADR, UDR, FTS, SPG");
}

void UbloxNode::getRosParams() {
  nh->param("device", device_, std::string("/dev/ttyACM0"));
  nh->param("frame_id", frame_id, std::string("gps"));

  // Save configuration parameters
  getRosUint("load/mask", load_.loadMask, 0);
  getRosUint("load/device", load_.deviceMask, 0);
  getRosUint("save/mask", save_.saveMask, 0);
  getRosUint("save/device", save_.deviceMask, 0);

  // UART 1 params
  getRosUint("uart1/baudrate", baudrate_, 9600);
  getRosUint("uart1/in", uart_in_,
             ublox_msgs::CfgPRT::PROTO_UBX | ublox_msgs::CfgPRT::PROTO_NMEA | ublox_msgs::CfgPRT::PROTO_RTCM);
  getRosUint("uart1/out", uart_out_, ublox_msgs::CfgPRT::PROTO_UBX);
  // USB params
  set_usb_ = false;
  if (nh->hasParam("usb/in") || nh->hasParam("usb/out")) {
    set_usb_ = true;
    if (!getRosUint("usb/in", usb_in_)) {
      throw std::runtime_error(std::string("usb/out is set, therefore ") + "usb/in must be set");
    }
    if (!getRosUint("usb/out", usb_out_)) {
      throw std::runtime_error(std::string("usb/in is set, therefore ") + "usb/out must be set");
    }
    getRosUint("usb/tx_ready", usb_tx_, 0);
  }
  // Measurement rate params
  nh->param("rate", rate_, 4.0);        // in Hz
  getRosUint("nav_rate", nav_rate, 1);  // # of measurement rate cycles
  // RTCM params
  getRosUint("rtcm/ids", rtcm_ids);      // RTCM output message IDs
  getRosUint("rtcm/rates", rtcm_rates);  // RTCM output message rates
  // PPP: Advanced Setting
  nh->param("enable_ppp", enable_ppp_, false);
  // SBAS params, only for some devices
  nh->param("sbas", enable_sbas_, false);
  getRosUint("sbas/max", max_sbas_, 0);  // Maximum number of SBAS channels
  getRosUint("sbas/usage", sbas_usage_, 0);
  nh->param("dynamic_model", dynamic_model_, std::string("portable"));
  nh->param("fix_mode", fix_mode_, std::string("auto"));
  getRosUint("dr_limit", dr_limit_, 0);  // Dead reckoning limit

  if (enable_ppp_) ROS_WARN("Warning: PPP is enabled - this is an expert setting.");

  checkMin(rate_, 0, "rate");

  if (rtcm_ids.size() != rtcm_rates.size())
    throw std::runtime_error(std::string("Invalid settings: size of rtcm_ids") + " must match size of rtcm_rates");

  dmodel_ = modelFromString(dynamic_model_);
  fmode_ = fixModeFromString(fix_mode_);

  nh->param("dat/set", set_dat_, false);
  if (set_dat_) {
    std::vector<float> shift, rot;
    if (!nh->getParam("dat/majA", cfg_dat_.majA) || nh->getParam("dat/flat", cfg_dat_.flat) ||
        nh->getParam("dat/shift", shift) || nh->getParam("dat/rot", rot) || nh->getParam("dat/scale", cfg_dat_.scale))
      throw std::runtime_error(std::string("dat/set is true, therefore ") +
                               "dat/majA, dat/flat, dat/shift, dat/rot, & dat/scale must be set");
    if (shift.size() != 3 || rot.size() != 3)
      throw std::runtime_error(std::string("size of dat/shift & dat/rot ") + "must be 3");
    checkRange(cfg_dat_.majA, 6300000.0, 6500000.0, "dat/majA");
    checkRange(cfg_dat_.flat, 0.0, 500.0, "dat/flat");

    checkRange(shift, 0.0, 500.0, "dat/shift");
    cfg_dat_.dX = shift[0];
    cfg_dat_.dY = shift[1];
    cfg_dat_.dZ = shift[2];

    checkRange(rot, -5000.0, 5000.0, "dat/rot");
    cfg_dat_.rotX = rot[0];
    cfg_dat_.rotY = rot[1];
    cfg_dat_.rotZ = rot[2];

    checkRange(cfg_dat_.scale, 0.0, 50.0, "scale");
  }

  // measurement period [ms]
  meas_rate = 1000 / rate_;

  // activate/deactivate any config
  nh->param("config_on_startup", config_on_startup_flag_, true);

  // raw data stream logging
  rawDataStreamPa_.getRosParams();
}

void UbloxNode::pollMessages(const ros::TimerEvent& event) {
  static std::vector<uint8_t> payload(1, 1);
  if (enabled["aid_alm"]) gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::ALM, payload);
  if (enabled["aid_eph"]) gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::EPH, payload);
  if (enabled["aid_hui"]) gps.poll(ublox_msgs::Class::AID, ublox_msgs::Message::AID::HUI);

  payload[0]++;
  if (payload[0] > 32) {
    payload[0] = 1;
  }
}

void UbloxNode::printInf(const ublox_msgs::Inf& m, uint8_t id) {
  if (id == ublox_msgs::Message::INF::ERROR)
    ROS_ERROR_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  else if (id == ublox_msgs::Message::INF::WARNING)
    ROS_WARN_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  else if (id == ublox_msgs::Message::INF::DEBUG)
    ROS_DEBUG_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
  else
    ROS_INFO_STREAM("INF: " << std::string(m.str.begin(), m.str.end()));
}

void UbloxNode::subscribe() {
  ROS_DEBUG("Subscribing to U-Blox messages");
  // subscribe messages
  nh->param("publish/all", enabled["all"], false);
  nh->param("inf/all", enabled["inf"], true);
  nh->param("publish/nav/all", enabled["nav"], enabled["all"]);
  nh->param("publish/rxm/all", enabled["rxm"], enabled["all"]);
  nh->param("publish/aid/all", enabled["aid"], enabled["all"]);
  nh->param("publish/mon/all", enabled["mon"], enabled["all"]);

  // Nav Messages
  nh->param("publish/nav/status", enabled["nav_status"], enabled["nav"]);
  if (enabled["nav_status"])
    gps.subscribe<ublox_msgs::NavSTATUS>(boost::bind(publish<ublox_msgs::NavSTATUS>, _1, "navstatus"), kSubscribeRate);

  nh->param("publish/nav/posecef", enabled["nav_posecef"], enabled["nav"]);
  if (enabled["nav_posecef"])
    gps.subscribe<ublox_msgs::NavPOSECEF>(boost::bind(publish<ublox_msgs::NavPOSECEF>, _1, "navposecef"),
                                          kSubscribeRate);

  nh->param("publish/nav/clock", enabled["nav_clock"], enabled["nav"]);
  if (enabled["nav_clock"])
    gps.subscribe<ublox_msgs::NavCLOCK>(boost::bind(publish<ublox_msgs::NavCLOCK>, _1, "navclock"), kSubscribeRate);

  // INF messages
  nh->param("inf/debug", enabled["inf_debug"], false);
  if (enabled["inf_debug"])
    gps.subscribeId<ublox_msgs::Inf>(boost::bind(&UbloxNode::printInf, this, _1, ublox_msgs::Message::INF::DEBUG),
                                     ublox_msgs::Message::INF::DEBUG);

  nh->param("inf/error", enabled["inf_error"], enabled["inf"]);
  if (enabled["inf_error"])
    gps.subscribeId<ublox_msgs::Inf>(boost::bind(&UbloxNode::printInf, this, _1, ublox_msgs::Message::INF::ERROR),
                                     ublox_msgs::Message::INF::ERROR);

  nh->param("inf/notice", enabled["inf_notice"], enabled["inf"]);
  if (enabled["inf_notice"])
    gps.subscribeId<ublox_msgs::Inf>(boost::bind(&UbloxNode::printInf, this, _1, ublox_msgs::Message::INF::NOTICE),
                                     ublox_msgs::Message::INF::NOTICE);

  nh->param("inf/test", enabled["inf_test"], enabled["inf"]);
  if (enabled["inf_test"])
    gps.subscribeId<ublox_msgs::Inf>(boost::bind(&UbloxNode::printInf, this, _1, ublox_msgs::Message::INF::TEST),
                                     ublox_msgs::Message::INF::TEST);

  nh->param("inf/warning", enabled["inf_warning"], enabled["inf"]);
  if (enabled["inf_warning"])
    gps.subscribeId<ublox_msgs::Inf>(boost::bind(&UbloxNode::printInf, this, _1, ublox_msgs::Message::INF::WARNING),
                                     ublox_msgs::Message::INF::WARNING);

  // AID messages
  nh->param("publish/aid/alm", enabled["aid_alm"], enabled["aid"]);
  if (enabled["aid_alm"])
    gps.subscribe<ublox_msgs::AidALM>(boost::bind(publish<ublox_msgs::AidALM>, _1, "aidalm"), kSubscribeRate);

  nh->param("publish/aid/eph", enabled["aid_eph"], enabled["aid"]);
  if (enabled["aid_eph"])
    gps.subscribe<ublox_msgs::AidEPH>(boost::bind(publish<ublox_msgs::AidEPH>, _1, "aideph"), kSubscribeRate);

  nh->param("publish/aid/hui", enabled["aid_hui"], enabled["aid"]);
  if (enabled["aid_hui"])
    gps.subscribe<ublox_msgs::AidHUI>(boost::bind(publish<ublox_msgs::AidHUI>, _1, "aidhui"), kSubscribeRate);

  for (int i = 0; i < components_.size(); i++) components_[i]->subscribe();
}

void UbloxNode::initializeRosDiagnostics() {
  if (!nh->hasParam("diagnostic_period")) nh->setParam("diagnostic_period", kDiagnosticPeriod);

  updater.reset(new diagnostic_updater::Updater());
  updater->setHardwareID("ublox");

  // configure diagnostic updater for frequency
  freq_diag.reset(new FixDiagnostic(std::string("fix"), kFixFreqTol, kFixFreqWindow, kTimeStampStatusMin));
  for (int i = 0; i < components_.size(); i++) components_[i]->initializeRosDiagnostics();
}

void UbloxNode::processMonVer() {
  ublox_msgs::MonVER monVer;
  if (!gps.poll(monVer)) throw std::runtime_error("Failed to poll MonVER & set relevant settings");

  ROS_DEBUG("%s, HW VER: %s", monVer.swVersion.c_array(), monVer.hwVersion.c_array());
  // Convert extension to vector of strings
  std::vector<std::string> extension;
  extension.reserve(monVer.extension.size());
  for (std::size_t i = 0; i < monVer.extension.size(); ++i) {
    ROS_DEBUG("%s", monVer.extension[i].field.c_array());
    // Find the end of the string (null character)
    unsigned char* end = std::find(monVer.extension[i].field.begin(), monVer.extension[i].field.end(), '\0');
    extension.push_back(std::string(monVer.extension[i].field.begin(), end));
  }

  // Get the protocol version
  for (std::size_t i = 0; i < extension.size(); ++i) {
    std::size_t found = extension[i].find("PROTVER");
    if (found != std::string::npos) {
      protocol_version_ = ::atof(extension[i].substr(8, extension[i].size() - 8).c_str());
      break;
    }
  }
  if (protocol_version_ == 0)
    ROS_WARN("Failed to parse MonVER and determine protocol version. %s", "Defaulting to firmware version 6.");
  addFirmwareInterface();

  if (protocol_version_ < 18) {
    // Final line contains supported GNSS delimited by ;
    std::vector<std::string> strs;
    if (extension.size() > 0) boost::split(strs, extension[extension.size() - 1], boost::is_any_of(";"));
    for (size_t i = 0; i < strs.size(); i++) supported.insert(strs[i]);
  } else {
    for (std::size_t i = 0; i < extension.size(); ++i) {
      std::vector<std::string> strs;
      // Up to 2nd to last line
      if (i <= extension.size() - 2) {
        boost::split(strs, extension[i], boost::is_any_of("="));
        if (strs.size() > 1) {
          if (strs[0].compare(std::string("FWVER")) == 0) {
            if (strs[1].length() > 8)
              addProductInterface(strs[1].substr(0, 3), strs[1].substr(8, 10));
            else
              addProductInterface(strs[1].substr(0, 3));
            continue;
          }
        }
      }
      // Last 1-2 lines contain supported GNSS
      if (i >= extension.size() - 2) {
        boost::split(strs, extension[i], boost::is_any_of(";"));
        for (size_t i = 0; i < strs.size(); i++) supported.insert(strs[i]);
      }
    }
  }
}

bool UbloxNode::configureUblox() {
  try {
    if (!gps.isInitialized()) throw std::runtime_error("Failed to initialize.");
    if (load_.loadMask != 0) {
      ROS_DEBUG("Loading u-blox configuration from memory. %u", load_.loadMask);
      if (!gps.configure(load_)) throw std::runtime_error(std::string("Failed to load configuration ") + "from memory");
      if (load_.loadMask & load_.MASK_IO_PORT) {
        ROS_DEBUG("Loaded I/O configuration from memory, resetting serial %s", "communications.");
        boost::posix_time::seconds wait(kResetWait);
        gps.reset(wait);
        if (!gps.isConfigured())
          throw std::runtime_error(std::string("Failed to reset serial I/O") +
                                   "after loading I/O configurations from device memory.");
      }
    }

    if (config_on_startup_flag_) {
      if (set_usb_) {
        gps.configUsb(usb_tx_, usb_in_, usb_out_);
      }
      if (!gps.configRate(meas_rate, nav_rate)) {
        std::stringstream ss;
        ss << "Failed to set measurement rate to " << meas_rate << "ms and navigation rate to " << nav_rate;
        throw std::runtime_error(ss.str());
      }
      // If device doesn't have SBAS, will receive NACK (causes exception)
      if (supportsGnss("SBAS")) {
        if (!gps.configSbas(enable_sbas_, sbas_usage_, max_sbas_)) {
          throw std::runtime_error(std::string("Failed to ") + ((enable_sbas_) ? "enable" : "disable") + " SBAS.");
        }
      }
      if (!gps.setPpp(enable_ppp_))
        throw std::runtime_error(std::string("Failed to ") + ((enable_ppp_) ? "enable" : "disable") + " PPP.");
      if (!gps.setDynamicModel(dmodel_)) throw std::runtime_error("Failed to set model: " + dynamic_model_ + ".");
      if (!gps.setFixMode(fmode_)) throw std::runtime_error("Failed to set fix mode: " + fix_mode_ + ".");
      if (!gps.setDeadReckonLimit(dr_limit_)) {
        std::stringstream ss;
        ss << "Failed to set dead reckoning limit: " << dr_limit_ << ".";
        throw std::runtime_error(ss.str());
      }
      if (set_dat_ && !gps.configure(cfg_dat_)) throw std::runtime_error("Failed to set user-defined datum.");
      // Configure each component
      for (int i = 0; i < components_.size(); i++) {
        if (!components_[i]->configureUblox()) return false;
      }
    }
    if (save_.saveMask != 0) {
      ROS_DEBUG("Saving the u-blox configuration, mask %u, device %u", save_.saveMask, save_.deviceMask);
      if (!gps.configure(save_)) ROS_ERROR("u-blox unable to save configuration to non-volatile memory");
    }
  } catch (std::exception& e) {
    ROS_FATAL("Error configuring u-blox: %s", e.what());
    return false;
  }
  return true;
}

void UbloxNode::configureInf() {
  ublox_msgs::CfgINF msg;
  // Subscribe to UBX INF messages
  ublox_msgs::CfgINF_Block block;
  block.protocolID = block.PROTOCOL_ID_UBX;
  // Enable desired INF messages on each UBX port
  uint8_t mask = (enabled["inf_error"] ? block.INF_MSG_ERROR : 0) |
                 (enabled["inf_warning"] ? block.INF_MSG_WARNING : 0) |
                 (enabled["inf_notice"] ? block.INF_MSG_NOTICE : 0) | (enabled["inf_test"] ? block.INF_MSG_TEST : 0) |
                 (enabled["inf_debug"] ? block.INF_MSG_DEBUG : 0);
  for (int i = 0; i < block.infMsgMask.size(); i++) block.infMsgMask[i] = mask;

  msg.blocks.push_back(block);

  // IF NMEA is enabled
  if (uart_in_ & ublox_msgs::CfgPRT::PROTO_NMEA) {
    ublox_msgs::CfgINF_Block block;
    block.protocolID = block.PROTOCOL_ID_NMEA;
    // Enable desired INF messages on each NMEA port
    for (int i = 0; i < block.infMsgMask.size(); i++) block.infMsgMask[i] = mask;
    msg.blocks.push_back(block);
  }

  ROS_DEBUG("Configuring INF messages");
  if (!gps.configure(msg)) ROS_WARN("Failed to configure INF messages");
}

void UbloxNode::initializeIo() {
  gps.setConfigOnStartup(config_on_startup_flag_);

  boost::smatch match;
  if (boost::regex_match(device_, match, boost::regex("(tcp|udp)://(.+):(\\d+)"))) {
    std::string proto(match[1]);
    if (proto == "tcp") {
      std::string host(match[2]);
      std::string port(match[3]);
      ROS_INFO("Connecting to %s://%s:%s ...", proto.c_str(), host.c_str(), port.c_str());
      gps.initializeTcp(host, port);
    } else {
      throw std::runtime_error("Protocol '" + proto + "' is unsupported");
    }
  } else {
    gps.initializeSerial(device_, baudrate_, uart_in_, uart_out_);
  }

  // raw data stream logging
  if (rawDataStreamPa_.isEnabled()) {
    gps.setRawDataCallback(boost::bind(&RawDataStreamPa::ubloxCallback, &rawDataStreamPa_, _1, _2));
    rawDataStreamPa_.initialize();
  }
}

void UbloxNode::initialize() {
  // Params must be set before initializing IO
  getRosParams();
  initializeIo();
  // Must process Mon VER before setting firmware/hardware params
  processMonVer();
  if (protocol_version_ <= 14) {
    if (nh->param("raw_data", false)) components_.push_back(ComponentPtr(new RawDataProduct));
  }
  // Must set firmware & hardware params before initializing diagnostics
  for (int i = 0; i < components_.size(); i++) components_[i]->getRosParams();
  // Do this last
  initializeRosDiagnostics();

  if (configureUblox()) {
    ROS_INFO("U-Blox configured successfully.");
    // Subscribe to all U-Blox messages
    subscribe();
    // Configure INF messages (needs INF params, call after subscribing)
    configureInf();

    ros::Timer poller;
    poller = nh->createTimer(ros::Duration(kPollDuration), &UbloxNode::pollMessages, this);
    poller.start();
    ros::spin();
  }
  shutdown();
}

void UbloxNode::shutdown() {
  if (gps.isInitialized()) {
    gps.close();
    ROS_INFO("Closed connection to %s.", device_.c_str());
  }
}

//
// U-Blox Firmware (all versions)
//
void UbloxFirmware::initializeRosDiagnostics() {
  updater->add("fix", this, &UbloxFirmware::fixDiagnostic);
  updater->force_update();
}

//
// U-Blox Firmware Version 6
//
UbloxFirmware6::UbloxFirmware6() {}

void UbloxFirmware6::getRosParams() {
  // Fix Service type, used when publishing fix status messages
  fix_status_service = sensor_msgs::NavSatStatus::SERVICE_GPS;

  nh->param("nmea/set", set_nmea_, false);
  if (set_nmea_) {
    bool compat, consider;

    if (!getRosUint("nmea/version", cfg_nmea_.version))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/version must be set");
    if (!getRosUint("nmea/num_sv", cfg_nmea_.numSV))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/num_sv must be set");
    if (!nh->getParam("nmea/compat", compat))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/compat must be set");
    if (!nh->getParam("nmea/consider", consider))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/consider must be set");

    // set flags
    cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;

    // set filter
    bool temp;
    nh->param("nmea/filter/pos", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_POS : 0;
    nh->param("nmea/filter/msk_pos", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_MSK_POS : 0;
    nh->param("nmea/filter/time", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TIME : 0;
    nh->param("nmea/filter/date", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_DATE : 0;
    nh->param("nmea/filter/sbas", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_SBAS_FILT : 0;
    nh->param("nmea/filter/track", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TRACK : 0;
  }
}

bool UbloxFirmware6::configureUblox() {
  ROS_WARN("ublox_version < 7, ignoring GNSS settings");

  if (set_nmea_ && !gps.configure(cfg_nmea_)) throw std::runtime_error("Failed to configure NMEA");

  return true;
}

void UbloxFirmware6::subscribe() {
  // Whether or not to publish Nav POS LLH (always subscribes)
  nh->param("publish/nav/posllh", enabled["nav_posllh"], enabled["nav"]);
  nh->param("publish/nav/sol", enabled["nav_sol"], enabled["nav"]);
  nh->param("publish/nav/velned", enabled["nav_velned"], enabled["nav"]);

  // Always subscribes to these messages, but may not publish to ROS topic
  // Subscribe to Nav POSLLH
  gps.subscribe<ublox_msgs::NavPOSLLH>(boost::bind(&UbloxFirmware6::callbackNavPosLlh, this, _1), kSubscribeRate);
  gps.subscribe<ublox_msgs::NavSOL>(boost::bind(
                                        // Subscribe to Nav SOL
                                        &UbloxFirmware6::callbackNavSol, this, _1),
                                    kSubscribeRate);
  // Subscribe to Nav VELNED
  gps.subscribe<ublox_msgs::NavVELNED>(boost::bind(&UbloxFirmware6::callbackNavVelNed, this, _1), kSubscribeRate);

  // Subscribe to Nav SVINFO
  nh->param("publish/nav/svinfo", enabled["nav_svinfo"], enabled["nav"]);
  if (enabled["nav_svinfo"])
    gps.subscribe<ublox_msgs::NavSVINFO>(boost::bind(publish<ublox_msgs::NavSVINFO>, _1, "navsvinfo"),
                                         kNavSvInfoSubscribeRate);

  // Subscribe to Mon HW
  nh->param("publish/mon_hw", enabled["mon_hw"], enabled["mon"]);
  if (enabled["mon_hw"])
    gps.subscribe<ublox_msgs::MonHW6>(boost::bind(publish<ublox_msgs::MonHW6>, _1, "monhw"), kSubscribeRate);
}

void UbloxFirmware6::fixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  // Set the diagnostic level based on the fix status
  if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_DEAD_RECKONING_ONLY) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Dead reckoning only";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_2D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "2D fix";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_3D_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "3D fix";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_GPS_DEAD_RECKONING_COMBINED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "GPS and dead reckoning combined";
  } else if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_TIME_ONLY_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Time fix only";
  }
  // If fix is not ok (within DOP & Accuracy Masks), raise the diagnostic level
  if (!(last_nav_sol_.flags & ublox_msgs::NavSOL::FLAGS_GPS_FIX_OK)) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message += ", fix not ok";
  }
  // Raise diagnostic level to error if no fix
  if (last_nav_sol_.gpsFix == ublox_msgs::NavSOL::GPS_NO_FIX) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "No fix";
  }

  // Add last fix position
  stat.add("iTOW [ms]", last_nav_pos_.iTOW);
  stat.add("Latitude [deg]", last_nav_pos_.lat * 1e-7);
  stat.add("Longitude [deg]", last_nav_pos_.lon * 1e-7);
  stat.add("Altitude [m]", last_nav_pos_.height * 1e-3);
  stat.add("Height above MSL [m]", last_nav_pos_.hMSL * 1e-3);
  stat.add("Horizontal Accuracy [m]", last_nav_pos_.hAcc * 1e-3);
  stat.add("Vertical Accuracy [m]", last_nav_pos_.vAcc * 1e-3);
  stat.add("# SVs used", (int)last_nav_sol_.numSV);
}

void UbloxFirmware6::callbackNavPosLlh(const ublox_msgs::NavPOSLLH& m) {
  if (enabled["nav_posllh"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::NavPOSLLH>("navposllh", kROSQueueSize);
    publisher.publish(m);
  }

  // Position message
  static ros::Publisher fixPublisher = nh->advertise<sensor_msgs::NavSatFix>("fix", kROSQueueSize);
  if (m.iTOW == last_nav_vel_.iTOW)
    fix_.header.stamp = velocity_.header.stamp;  // use last timestamp
  else
    fix_.header.stamp = ros::Time::now();  // new timestamp

  fix_.header.frame_id = frame_id;
  fix_.latitude = m.lat * 1e-7;
  fix_.longitude = m.lon * 1e-7;
  fix_.altitude = m.height * 1e-3;

  if (last_nav_sol_.gpsFix >= last_nav_sol_.GPS_2D_FIX)
    fix_.status.status = fix_.status.STATUS_FIX;
  else
    fix_.status.status = fix_.status.STATUS_NO_FIX;

  // Convert from mm to m
  const double varH = pow(m.hAcc / 1000.0, 2);
  const double varV = pow(m.vAcc / 1000.0, 2);

  fix_.position_covariance[0] = varH;
  fix_.position_covariance[4] = varH;
  fix_.position_covariance[8] = varV;
  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

  fix_.status.service = fix_.status.SERVICE_GPS;
  fixPublisher.publish(fix_);
  last_nav_pos_ = m;
  //  update diagnostics
  freq_diag->diagnostic->tick(fix_.header.stamp);
  updater->update();
}

void UbloxFirmware6::callbackNavVelNed(const ublox_msgs::NavVELNED& m) {
  if (enabled["nav_velned"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::NavVELNED>("navvelned", kROSQueueSize);
    publisher.publish(m);
  }

  // Example geometry message
  static ros::Publisher velocityPublisher =
      nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("fix_velocity", kROSQueueSize);
  if (m.iTOW == last_nav_pos_.iTOW)
    velocity_.header.stamp = fix_.header.stamp;  // same time as last navposllh
  else
    velocity_.header.stamp = ros::Time::now();  // create a new timestamp
  velocity_.header.frame_id = frame_id;

  //  convert to XYZ linear velocity
  velocity_.twist.twist.linear.x = m.velE / 100.0;
  velocity_.twist.twist.linear.y = m.velN / 100.0;
  velocity_.twist.twist.linear.z = -m.velD / 100.0;

  const double varSpeed = pow(m.sAcc / 100.0, 2);

  const int cols = 6;
  velocity_.twist.covariance[cols * 0 + 0] = varSpeed;
  velocity_.twist.covariance[cols * 1 + 1] = varSpeed;
  velocity_.twist.covariance[cols * 2 + 2] = varSpeed;
  velocity_.twist.covariance[cols * 3 + 3] = -1;  //  angular rate unsupported

  velocityPublisher.publish(velocity_);
  last_nav_vel_ = m;
}

void UbloxFirmware6::callbackNavSol(const ublox_msgs::NavSOL& m) {
  if (enabled["nav_sol"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSOL>("navsol", kROSQueueSize);
    publisher.publish(m);
  }
  last_nav_sol_ = m;
}

//
// Ublox Firmware Version 7
//
UbloxFirmware7::UbloxFirmware7() {}

void UbloxFirmware7::getRosParams() {
  //
  // GNSS configuration
  //
  // GNSS enable/disable
  nh->param("gnss/gps", enable_gps_, true);
  nh->param("gnss/glonass", enable_glonass_, false);
  nh->param("gnss/qzss", enable_qzss_, false);
  getRosUint("gnss/qzss_sig_cfg", qzss_sig_cfg_, ublox_msgs::CfgGNSS_Block::SIG_CFG_QZSS_L1CA);
  nh->param("gnss/sbas", enable_sbas_, false);

  if (enable_gps_ && !supportsGnss("GPS")) ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by this device");
  if (enable_glonass_ && !supportsGnss("GLO"))
    ROS_WARN("gnss/glonass is true, but GLONASS is not %s", "supported by this device");
  if (enable_qzss_ && !supportsGnss("QZSS")) ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  if (enable_sbas_ && !supportsGnss("SBAS")) ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");

  if (nh->hasParam("gnss/galileo")) ROS_WARN("ublox_version < 8, ignoring Galileo GNSS Settings");
  if (nh->hasParam("gnss/beidou")) ROS_WARN("ublox_version < 8, ignoring BeiDou Settings");
  if (nh->hasParam("gnss/imes")) ROS_WARN("ublox_version < 8, ignoring IMES GNSS Settings");

  // Fix Service type, used when publishing fix status messages
  fix_status_service =
      sensor_msgs::NavSatStatus::SERVICE_GPS + (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS;

  //
  // NMEA Configuration
  //
  nh->param("nmea/set", set_nmea_, false);
  if (set_nmea_) {
    bool compat, consider;

    if (!getRosUint("nmea/version", cfg_nmea_.nmeaVersion))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/version must be set");
    if (!getRosUint("nmea/num_sv", cfg_nmea_.numSV))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/num_sv must be set");
    if (!getRosUint("nmea/sv_numbering", cfg_nmea_.svNumbering))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/sv_numbering must be set");
    if (!nh->getParam("nmea/compat", compat))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/compat must be set");
    if (!nh->getParam("nmea/consider", consider))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/consider must be set");

    // set flags
    cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;
    // set filter
    bool temp;
    nh->param("nmea/filter/pos", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_POS : 0;
    nh->param("nmea/filter/msk_pos", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_MSK_POS : 0;
    nh->param("nmea/filter/time", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TIME : 0;
    nh->param("nmea/filter/date", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_DATE : 0;
    nh->param("nmea/filter/gps_only", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_GPS_ONLY : 0;
    nh->param("nmea/filter/track", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TRACK : 0;
    // set gnssToFilter
    nh->param("nmea/gnssToFilter/gps", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_GPS : 0;
    nh->param("nmea/gnssToFilter/sbas", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_SBAS : 0;
    nh->param("nmea/gnssToFilter/qzss", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_QZSS : 0;
    nh->param("nmea/gnssToFilter/glonass", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_GLONASS : 0;

    getRosUint("nmea/main_talker_id", cfg_nmea_.mainTalkerId);
    getRosUint("nmea/gsv_talker_id", cfg_nmea_.gsvTalkerId);
  }
}

bool UbloxFirmware7::configureUblox() {
  /** Configure the GNSS **/
  ublox_msgs::CfgGNSS cfgGNSSRead;
  if (gps.poll(cfgGNSSRead)) {
    ROS_DEBUG("Read GNSS config.");
    ROS_DEBUG("Num. tracking channels in hardware: %i", cfgGNSSRead.numTrkChHw);
    ROS_DEBUG("Num. tracking channels to use: %i", cfgGNSSRead.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  ublox_msgs::CfgGNSS cfgGNSSWrite;
  cfgGNSSWrite.numConfigBlocks = 1;  // do services one by one
  cfgGNSSWrite.numTrkChHw = cfgGNSSRead.numTrkChHw;
  cfgGNSSWrite.numTrkChUse = cfgGNSSRead.numTrkChUse;
  cfgGNSSWrite.msgVer = 0;

  // configure GLONASS
  if (supportsGnss("GLO")) {
    ublox_msgs::CfgGNSS_Block block;
    block.gnssId = block.GNSS_ID_GLONASS;
    block.resTrkCh = block.RES_TRK_CH_GLONASS;
    block.maxTrkCh = block.MAX_TRK_CH_GLONASS;
    block.flags = enable_glonass_ ? block.SIG_CFG_GLONASS_L1OF : 0;
    cfgGNSSWrite.blocks.push_back(block);
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") + ((enable_glonass_) ? "enable" : "disable") + " GLONASS.");
    }
  }

  if (supportsGnss("QZSS")) {
    // configure QZSS
    ublox_msgs::CfgGNSS_Block block;
    block.gnssId = block.GNSS_ID_QZSS;
    block.resTrkCh = block.RES_TRK_CH_QZSS;
    block.maxTrkCh = block.MAX_TRK_CH_QZSS;
    block.flags = enable_qzss_ ? qzss_sig_cfg_ : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") + ((enable_glonass_) ? "enable" : "disable") + " QZSS.");
    }
  }

  if (supportsGnss("SBAS")) {
    // configure SBAS
    ublox_msgs::CfgGNSS_Block block;
    block.gnssId = block.GNSS_ID_SBAS;
    block.resTrkCh = block.RES_TRK_CH_SBAS;
    block.maxTrkCh = block.MAX_TRK_CH_SBAS;
    block.flags = enable_sbas_ ? block.SIG_CFG_SBAS_L1CA : 0;
    cfgGNSSWrite.blocks[0] = block;
    if (!gps.configure(cfgGNSSWrite)) {
      throw std::runtime_error(std::string("Failed to ") + ((enable_sbas_) ? "enable" : "disable") + " SBAS.");
    }
  }

  if (set_nmea_ && !gps.configure(cfg_nmea_)) throw std::runtime_error("Failed to configure NMEA");

  return true;
}

void UbloxFirmware7::subscribe() {
  // Whether to publish Nav PVT messages to a ROS topic
  nh->param("publish/nav/pvt", enabled["nav_pvt"], enabled["nav"]);
  // Subscribe to Nav PVT (always does so since fix information is published
  // from this)
  gps.subscribe<ublox_msgs::NavPVT7>(boost::bind(&UbloxFirmware7Plus::callbackNavPvt, this, _1), kSubscribeRate);

  // Subscribe to Nav SVINFO
  nh->param("publish/nav/svinfo", enabled["nav_svinfo"], enabled["nav"]);
  if (enabled["nav_svinfo"])
    gps.subscribe<ublox_msgs::NavSVINFO>(boost::bind(publish<ublox_msgs::NavSVINFO>, _1, "navsvinfo"),
                                         kNavSvInfoSubscribeRate);

  // Subscribe to Mon HW
  nh->param("publish/mon_hw", enabled["mon_hw"], enabled["mon"]);
  if (enabled["mon_hw"])
    gps.subscribe<ublox_msgs::MonHW>(boost::bind(publish<ublox_msgs::MonHW>, _1, "monhw"), kSubscribeRate);
}

//
// Ublox Version 8
//
UbloxFirmware8::UbloxFirmware8() {}

void UbloxFirmware8::getRosParams() {
  // UPD SOS configuration
  nh->param("clear_bbr", clear_bbr_, false);
  gps.setSaveOnShutdown(nh->param("save_on_shutdown", false));

  // GNSS enable/disable
  nh->param("gnss/gps", enable_gps_, true);
  nh->param("gnss/galileo", enable_galileo_, false);
  nh->param("gnss/beidou", enable_beidou_, false);
  nh->param("gnss/imes", enable_imes_, false);
  nh->param("gnss/glonass", enable_glonass_, false);
  nh->param("gnss/qzss", enable_qzss_, false);
  nh->param("gnss/sbas", enable_sbas_, false);
  // QZSS Signal Configuration
  getRosUint("gnss/qzss_sig_cfg", qzss_sig_cfg_, ublox_msgs::CfgGNSS_Block::SIG_CFG_QZSS_L1CA);

  if (enable_gps_ && !supportsGnss("GPS"))
    ROS_WARN("gnss/gps is true, but GPS GNSS is not supported by %s", "this device");
  if (enable_glonass_ && !supportsGnss("GLO"))
    ROS_WARN("gnss/glonass is true, but GLONASS is not supported by %s", "this device");
  if (enable_galileo_ && !supportsGnss("GAL"))
    ROS_WARN("gnss/galileo is true, but Galileo GNSS is not supported %s", "by this device");
  if (enable_beidou_ && !supportsGnss("BDS"))
    ROS_WARN("gnss/beidou is true, but Beidou GNSS is not supported %s", "by this device");
  if (enable_imes_ && !supportsGnss("IMES"))
    ROS_WARN("gnss/imes is true, but IMES GNSS is not supported by %s", "this device");
  if (enable_qzss_ && !supportsGnss("QZSS")) ROS_WARN("gnss/qzss is true, but QZSS is not supported by this device");
  if (enable_sbas_ && !supportsGnss("SBAS")) ROS_WARN("gnss/sbas is true, but SBAS is not supported by this device");

  // Fix Service type, used when publishing fix status messages
  fix_status_service = sensor_msgs::NavSatStatus::SERVICE_GPS +
                       (enable_glonass_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GLONASS +
                       (enable_beidou_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_COMPASS +
                       (enable_galileo_ ? 1 : 0) * sensor_msgs::NavSatStatus::SERVICE_GALILEO;

  //
  // NMEA Configuration
  //
  nh->param("nmea/set", set_nmea_, false);
  if (set_nmea_) {
    bool compat, consider;
    cfg_nmea_.version = cfg_nmea_.VERSION;  // message version

    // Verify that parameters are set
    if (!getRosUint("nmea/version", cfg_nmea_.nmeaVersion))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/version must be set");
    if (!getRosUint("nmea/num_sv", cfg_nmea_.numSV))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/num_sv must be set");
    if (!getRosUint("nmea/sv_numbering", cfg_nmea_.svNumbering))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/sv_numbering must be set");
    if (!nh->getParam("nmea/compat", compat))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/compat must be set");
    if (!nh->getParam("nmea/consider", consider))
      throw std::runtime_error(std::string("Invalid settings: nmea/set is ") +
                               "true, therefore nmea/consider must be set");

    // set flags
    cfg_nmea_.flags = compat ? cfg_nmea_.FLAGS_COMPAT : 0;
    cfg_nmea_.flags |= consider ? cfg_nmea_.FLAGS_CONSIDER : 0;
    bool temp;
    nh->param("nmea/limit82", temp, false);
    cfg_nmea_.flags |= temp ? cfg_nmea_.FLAGS_LIMIT82 : 0;
    nh->param("nmea/high_prec", temp, false);
    cfg_nmea_.flags |= temp ? cfg_nmea_.FLAGS_HIGH_PREC : 0;
    // set filter
    nh->param("nmea/filter/pos", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_POS : 0;
    nh->param("nmea/filter/msk_pos", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_MSK_POS : 0;
    nh->param("nmea/filter/time", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TIME : 0;
    nh->param("nmea/filter/date", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_DATE : 0;
    nh->param("nmea/filter/gps_only", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_GPS_ONLY : 0;
    nh->param("nmea/filter/track", temp, false);
    cfg_nmea_.filter |= temp ? cfg_nmea_.FILTER_TRACK : 0;
    // set gnssToFilter
    nh->param("nmea/gnssToFilter/gps", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_GPS : 0;
    nh->param("nmea/gnssToFilter/sbas", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_SBAS : 0;
    nh->param("nmea/gnssToFilter/qzss", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_QZSS : 0;
    nh->param("nmea/gnssToFilter/glonass", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_GLONASS : 0;
    nh->param("nmea/gnssToFilter/beidou", temp, false);
    cfg_nmea_.gnssToFilter |= temp ? cfg_nmea_.GNSS_TO_FILTER_BEIDOU : 0;

    getRosUint("nmea/main_talker_id", cfg_nmea_.mainTalkerId);
    getRosUint("nmea/gsv_talker_id", cfg_nmea_.gsvTalkerId);

    std::vector<uint8_t> bdsTalkerId;
    getRosUint("nmea/bds_talker_id", bdsTalkerId);
    cfg_nmea_.bdsTalkerId[0] = bdsTalkerId[0];
    cfg_nmea_.bdsTalkerId[1] = bdsTalkerId[1];
  }
}

bool UbloxFirmware8::configureUblox() {
  if (clear_bbr_) {
    // clear flash memory
    if (!gps.clearBbr()) ROS_ERROR("u-blox failed to clear flash memory");
  }
  //
  // Configure the GNSS, only if the configuration is different
  //
  // First, get the current GNSS configuration
  ublox_msgs::CfgGNSS cfg_gnss;
  if (gps.poll(cfg_gnss)) {
    ROS_DEBUG("Read GNSS config.");
    ROS_DEBUG("Num. tracking channels in hardware: %i", cfg_gnss.numTrkChHw);
    ROS_DEBUG("Num. tracking channels to use: %i", cfg_gnss.numTrkChUse);
  } else {
    throw std::runtime_error("Failed to read the GNSS config.");
  }

  // Then, check the configuration for each GNSS. If it is different, change it.
  bool correct = true;
  for (int i = 0; i < cfg_gnss.blocks.size(); i++) {
    ublox_msgs::CfgGNSS_Block block = cfg_gnss.blocks[i];
    if (block.gnssId == block.GNSS_ID_GPS && enable_gps_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_gps_;
      ROS_DEBUG("GPS Configuration is different");
    } else if (block.gnssId == block.GNSS_ID_SBAS && enable_sbas_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_sbas_;
      ROS_DEBUG("SBAS Configuration is different");
    } else if (block.gnssId == block.GNSS_ID_GALILEO && enable_galileo_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_galileo_;
      ROS_DEBUG("Galileo GNSS Configuration is different");
    } else if (block.gnssId == block.GNSS_ID_BEIDOU && enable_beidou_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_beidou_;
      ROS_DEBUG("BeiDou Configuration is different");
    } else if (block.gnssId == block.GNSS_ID_IMES && enable_imes_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_imes_;
    } else if (block.gnssId == block.GNSS_ID_QZSS &&
               (enable_qzss_ != (block.flags & block.FLAGS_ENABLE) ||
                (enable_qzss_ && qzss_sig_cfg_ != (block.flags & block.FLAGS_SIG_CFG_MASK)))) {
      ROS_DEBUG("QZSS Configuration is different %u, %u", block.flags & block.FLAGS_ENABLE, enable_qzss_);
      correct = false;
      ROS_DEBUG("QZSS Configuration: %u", block.flags);
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_qzss_;
      ROS_DEBUG("QZSS Configuration: %u", cfg_gnss.blocks[i].flags);
      if (enable_qzss_)
        // Only change sig cfg if enabling
        cfg_gnss.blocks[i].flags |= qzss_sig_cfg_;
    } else if (block.gnssId == block.GNSS_ID_GLONASS && enable_glonass_ != (block.flags & block.FLAGS_ENABLE)) {
      correct = false;
      cfg_gnss.blocks[i].flags = (cfg_gnss.blocks[i].flags & ~block.FLAGS_ENABLE) | enable_glonass_;
      ROS_DEBUG("GLONASS Configuration is different");
    }
  }

  // If the GNSS is already configured correctly, do not re-configure GNSS
  // since this requires a cold reset
  if (correct)
    ROS_DEBUG("U-Blox GNSS configuration is correct. GNSS not re-configured.");
  else if (!gps.configGnss(cfg_gnss, boost::posix_time::seconds(15)))
    throw std::runtime_error(std::string("Failed to cold reset device ") + "after configuring GNSS");

  //
  // NMEA config
  //
  if (set_nmea_ && !gps.configure(cfg_nmea_)) throw std::runtime_error("Failed to configure NMEA");

  return true;
}

void UbloxFirmware8::subscribe() {
  // Whether to publish Nav PVT messages
  nh->param("publish/nav/pvt", enabled["nav_pvt"], enabled["nav"]);
  // Subscribe to Nav PVT
  gps.subscribe<ublox_msgs::NavPVT>(boost::bind(&UbloxFirmware7Plus::callbackNavPvt, this, _1), kSubscribeRate);

  // Subscribe to Nav SAT messages
  nh->param("publish/nav/sat", enabled["nav_sat"], enabled["nav"]);
  if (enabled["nav_sat"])
    gps.subscribe<ublox_msgs::NavSAT>(boost::bind(publish<ublox_msgs::NavSAT>, _1, "navsat"), kNavSvInfoSubscribeRate);

  // Subscribe to Mon HW
  nh->param("publish/mon/hw", enabled["mon_hw"], enabled["mon"]);
  if (enabled["mon_hw"])
    gps.subscribe<ublox_msgs::MonHW>(boost::bind(publish<ublox_msgs::MonHW>, _1, "monhw"), kSubscribeRate);

  // Subscribe to RTCM messages
  nh->param("publish/rxm/rtcm", enabled["rxm_rtcm"], enabled["rxm"]);
  if (enabled["rxm_rtcm"])
    gps.subscribe<ublox_msgs::RxmRTCM>(boost::bind(publish<ublox_msgs::RxmRTCM>, _1, "rxmrtcm"), kSubscribeRate);
}

//
// Raw Data Products
//
void RawDataProduct::subscribe() {
  // Defaults to true instead of to all
  nh->param("publish/rxm/all", enabled["rxm"], true);

  // Subscribe to RXM Raw
  nh->param("publish/rxm/raw", enabled["rxm_raw"], enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAW>(boost::bind(publish<ublox_msgs::RxmRAW>, _1, "rxmraw"), kSubscribeRate);

  // Subscribe to RXM SFRB
  nh->param("publish/rxm/sfrb", enabled["rxm_sfrb"], enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRB>(boost::bind(publish<ublox_msgs::RxmSFRB>, _1, "rxmsfrb"), kSubscribeRate);

  // Subscribe to RXM EPH
  nh->param("publish/rxm/eph", enabled["rxm_eph"], enabled["rxm"]);
  if (enabled["rxm_eph"])
    gps.subscribe<ublox_msgs::RxmEPH>(boost::bind(publish<ublox_msgs::RxmEPH>, _1, "rxmeph"), kSubscribeRate);

  // Subscribe to RXM ALM
  nh->param("publish/rxm/almRaw", enabled["rxm_alm"], enabled["rxm"]);
  if (enabled["rxm_alm"])
    gps.subscribe<ublox_msgs::RxmALM>(boost::bind(publish<ublox_msgs::RxmALM>, _1, "rxmalm"), kSubscribeRate);
}

void RawDataProduct::initializeRosDiagnostics() {
  if (enabled["rxm_raw"])
    freq_diagnostics_.push_back(
        boost::shared_ptr<UbloxTopicDiagnostic>(new UbloxTopicDiagnostic("rxmraw", kRtcmFreqTol, kRtcmFreqWindow)));
  if (enabled["rxm_sfrb"])
    freq_diagnostics_.push_back(
        boost::shared_ptr<UbloxTopicDiagnostic>(new UbloxTopicDiagnostic("rxmsfrb", kRtcmFreqTol, kRtcmFreqWindow)));
  if (enabled["rxm_eph"])
    freq_diagnostics_.push_back(
        boost::shared_ptr<UbloxTopicDiagnostic>(new UbloxTopicDiagnostic("rxmeph", kRtcmFreqTol, kRtcmFreqWindow)));
  if (enabled["rxm_alm"])
    freq_diagnostics_.push_back(
        boost::shared_ptr<UbloxTopicDiagnostic>(new UbloxTopicDiagnostic("rxmalm", kRtcmFreqTol, kRtcmFreqWindow)));
}

//
// u-blox ADR devices, partially implemented
//
void AdrUdrProduct::getRosParams() {
  nh->param("use_adr", use_adr_, true);
  // Check the nav rate
  float nav_rate_hz = 1000 / (meas_rate * nav_rate);
  if (nav_rate_hz != 1) ROS_WARN("Nav Rate recommended to be 1 Hz");
}

bool AdrUdrProduct::configureUblox() {
  if (!gps.setUseAdr(use_adr_))
    throw std::runtime_error(std::string("Failed to ") + (use_adr_ ? "enable" : "disable") + "use_adr");
  return true;
}

void AdrUdrProduct::subscribe() {
  nh->param("publish/esf/all", enabled["esf"], true);

  // Subscribe to NAV ATT messages
  nh->param("publish/nav/att", enabled["nav_att"], enabled["nav"]);
  if (enabled["nav_att"])
    gps.subscribe<ublox_msgs::NavATT>(boost::bind(publish<ublox_msgs::NavATT>, _1, "navatt"), kSubscribeRate);

  // Subscribe to ESF INS messages
  nh->param("publish/esf/ins", enabled["esf_ins"], enabled["esf"]);
  if (enabled["esf_ins"])
    gps.subscribe<ublox_msgs::EsfINS>(boost::bind(publish<ublox_msgs::EsfINS>, _1, "esfins"), kSubscribeRate);

  // Subscribe to ESF Meas messages
  nh->param("publish/esf/meas", enabled["esf_meas"], enabled["esf"]);
  if (enabled["esf_meas"])
    gps.subscribe<ublox_msgs::EsfMEAS>(boost::bind(publish<ublox_msgs::EsfMEAS>, _1, "esfmeas"), kSubscribeRate);
  // also publish sensor_msgs::Imu
  gps.subscribe<ublox_msgs::EsfMEAS>(boost::bind(&AdrUdrProduct::callbackEsfMEAS, this, _1), kSubscribeRate);

  // Subscribe to ESF Raw messages
  nh->param("publish/esf/raw", enabled["esf_raw"], enabled["esf"]);
  if (enabled["esf_raw"])
    gps.subscribe<ublox_msgs::EsfRAW>(boost::bind(publish<ublox_msgs::EsfRAW>, _1, "esfraw"), kSubscribeRate);

  // Subscribe to ESF Status messages
  nh->param("publish/esf/status", enabled["esf_status"], enabled["esf"]);
  if (enabled["esf_status"])
    gps.subscribe<ublox_msgs::EsfSTATUS>(boost::bind(publish<ublox_msgs::EsfSTATUS>, _1, "esfstatus"), kSubscribeRate);

  // Subscribe to HNR PVT messages
  nh->param("publish/hnr/pvt", enabled["hnr_pvt"], true);
  if (enabled["hnr_pvt"])
    gps.subscribe<ublox_msgs::HnrPVT>(boost::bind(publish<ublox_msgs::HnrPVT>, _1, "hnrpvt"), kSubscribeRate);
}

void AdrUdrProduct::callbackEsfMEAS(const ublox_msgs::EsfMEAS& m) {
  if (enabled["esf_meas"]) {
    static ros::Publisher imu_pub = nh->advertise<sensor_msgs::Imu>("imu_meas", kROSQueueSize);
    static ros::Publisher time_ref_pub = nh->advertise<sensor_msgs::TimeReference>("interrupt_time", kROSQueueSize);

    imu_.header.stamp = ros::Time::now();
    imu_.header.frame_id = frame_id;

    float deg_per_sec = pow(2, -12);
    float m_per_sec_sq = pow(2, -10);
    float deg_c = 1e-2;

    std::vector<unsigned int> imu_data = m.data;
    for (int i = 0; i < imu_data.size(); i++) {
      unsigned int data_type = imu_data[i] >> 24;        // grab the last six bits of data
      double data_sign = (imu_data[i] & (1 << 23));      // grab the sign (+/-) of the rest of the data
      unsigned int data_value = imu_data[i] & 0x7FFFFF;  // grab the rest of the data...should be 23 bits

      if (data_sign == 0) {
        data_sign = -1;
      } else {
        data_sign = 1;
      }

      // ROS_INFO("data sign (+/-): %f", data_sign); //either 1 or -1....set by bit 23 in the data bitarray

      imu_.orientation_covariance[0] = -1;
      imu_.linear_acceleration_covariance[0] = -1;
      imu_.angular_velocity_covariance[0] = -1;

      if (data_type == 14) {
        if (data_sign == 1) {
          imu_.angular_velocity.x = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.x = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 16) {
        // ROS_INFO("data_sign: %f", data_sign);
        // ROS_INFO("data_value: %u", data_value * m);
        if (data_sign == 1) {
          imu_.linear_acceleration.x = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.x = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 13) {
        if (data_sign == 1) {
          imu_.angular_velocity.y = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.y = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 17) {
        if (data_sign == 1) {
          imu_.linear_acceleration.y = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.y = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 5) {
        if (data_sign == 1) {
          imu_.angular_velocity.z = 2048 - data_value * deg_per_sec;
        } else {
          imu_.angular_velocity.z = data_sign * data_value * deg_per_sec;
        }
      } else if (data_type == 18) {
        if (data_sign == 1) {
          imu_.linear_acceleration.z = 8191 - data_value * m_per_sec_sq;
        } else {
          imu_.linear_acceleration.z = data_sign * data_value * m_per_sec_sq;
        }
      } else if (data_type == 12) {
        // ROS_INFO("Temperature in celsius: %f", data_value * deg_c);
      } else {
        ROS_INFO("data_type: %u", data_type);
        ROS_INFO("data_value: %u", data_value);
      }

      // create time ref message and put in the data
      // t_ref_.header.seq = m.risingEdgeCount;
      // t_ref_.header.stamp = ros::Time::now();
      // t_ref_.header.frame_id = frame_id;

      // t_ref_.time_ref = ros::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

      // std::ostringstream src;
      // src << "TIM" << int(m.ch);
      // t_ref_.source = src.str();

      t_ref_.header.stamp = ros::Time::now();  // create a new timestamp
      t_ref_.header.frame_id = frame_id;

      time_ref_pub.publish(t_ref_);
      imu_pub.publish(imu_);
    }
  }

  updater->force_update();
}
//
// u-blox High Precision GNSS Reference Station
//
void HpgRefProduct::getRosParams() {
  if (config_on_startup_flag_) {
    if (nav_rate * meas_rate != 1000) ROS_WARN("For HPG Ref devices, nav_rate should be exactly 1 Hz.");

    if (!getRosUint("tmode3", tmode3_)) throw std::runtime_error("Invalid settings: TMODE3 must be set");

    if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_FIXED) {
      if (!nh->getParam("arp/position", arp_position_))
        throw std::runtime_error(std::string("Invalid settings: arp/position ") + "must be set if TMODE3 is fixed");
      if (!getRosInt("arp/position_hp", arp_position_hp_))
        throw std::runtime_error(std::string("Invalid settings: arp/position_hp ") + "must be set if TMODE3 is fixed");
      if (!nh->getParam("arp/acc", fixed_pos_acc_))
        throw std::runtime_error(std::string("Invalid settings: arp/acc ") + "must be set if TMODE3 is fixed");
      if (!nh->getParam("arp/lla_flag", lla_flag_)) {
        ROS_WARN("arp/lla_flag param not set, assuming ARP coordinates are %s", "in ECEF");
        lla_flag_ = false;
      }
    } else if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
      nh->param("sv_in/reset", svin_reset_, true);
      if (!getRosUint("sv_in/min_dur", sv_in_min_dur_))
        throw std::runtime_error(std::string("Invalid settings: sv_in/min_dur ") +
                                 "must be set if TMODE3 is survey-in");
      if (!nh->getParam("sv_in/acc_lim", sv_in_acc_lim_))
        throw std::runtime_error(std::string("Invalid settings: sv_in/acc_lim ") +
                                 "must be set if TMODE3 is survey-in");
    } else if (tmode3_ != ublox_msgs::CfgTMODE3::FLAGS_MODE_DISABLED) {
      throw std::runtime_error(std::string("tmode3 param invalid. See CfgTMODE3") +
                               " flag constants for possible values.");
    }
  }
}

bool HpgRefProduct::configureUblox() {
  // Configure TMODE3
  if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_DISABLED) {
    if (!gps.disableTmode3()) throw std::runtime_error("Failed to disable TMODE3.");
    mode_ = DISABLED;
  } else if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_FIXED) {
    if (!gps.configTmode3Fixed(lla_flag_, arp_position_, arp_position_hp_, fixed_pos_acc_))
      throw std::runtime_error("Failed to set TMODE3 to fixed.");
    if (!gps.configRtcm(rtcm_ids, rtcm_rates)) throw std::runtime_error("Failed to set RTCM rates");
    mode_ = FIXED;
  } else if (tmode3_ == ublox_msgs::CfgTMODE3::FLAGS_MODE_SURVEY_IN) {
    if (!svin_reset_) {
      ublox_msgs::NavSVIN nav_svin;
      if (!gps.poll(nav_svin))
        throw std::runtime_error(std::string("Failed to poll NavSVIN while") + " configuring survey-in");
      // Don't reset survey-in if it's already active
      if (nav_svin.active) {
        mode_ = SURVEY_IN;
        return true;
      }
      // Don't reset survey-in if it already has a valid value
      if (nav_svin.valid) {
        setTimeMode();
        return true;
      }
      ublox_msgs::NavPVT nav_pvt;
      if (!gps.poll(nav_pvt))
        throw std::runtime_error(std::string("Failed to poll NavPVT while") + " configuring survey-in");
      // Don't reset survey in if in time mode with a good fix
      if (nav_pvt.fixType == nav_pvt.FIX_TYPE_TIME_ONLY && nav_pvt.flags & nav_pvt.FLAGS_GNSS_FIX_OK) {
        setTimeMode();
        return true;
      }
    }
    // Reset the Survey In
    // For Survey in, meas rate must be at least 1 Hz
    uint16_t meas_rate_temp = meas_rate < 1000 ? meas_rate : 1000;  // [ms]
    // If measurement period isn't a factor of 1000, set to default
    if (1000 % meas_rate_temp != 0) meas_rate_temp = kDefaultMeasPeriod;
    // Set nav rate to 1 Hz during survey in
    if (!gps.configRate(meas_rate_temp, (int)1000 / meas_rate_temp))
      throw std::runtime_error(std::string("Failed to set nav rate to 1 Hz") + "before setting TMODE3 to survey-in.");
    // As recommended in the documentation, first disable, then set to survey in
    if (!gps.disableTmode3())
      ROS_ERROR("Failed to disable TMODE3 before setting to survey-in.");
    else
      mode_ = DISABLED;
    // Set to Survey in mode
    if (!gps.configTmode3SurveyIn(sv_in_min_dur_, sv_in_acc_lim_))
      throw std::runtime_error("Failed to set TMODE3 to survey-in.");
    mode_ = SURVEY_IN;
  }
  return true;
}

void HpgRefProduct::subscribe() {
  // Whether to publish Nav Survey-In messages
  nh->param("publish/nav/svin", enabled["nav_svin"], enabled["nav"]);
  // Subscribe to Nav Survey-In
  gps.subscribe<ublox_msgs::NavSVIN>(boost::bind(&HpgRefProduct::callbackNavSvIn, this, _1), kSubscribeRate);
}

void HpgRefProduct::callbackNavSvIn(ublox_msgs::NavSVIN m) {
  if (enabled["nav_svin"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::NavSVIN>("navsvin", kROSQueueSize);
    publisher.publish(m);
  }

  last_nav_svin_ = m;

  if (!m.active && m.valid && mode_ == SURVEY_IN) {
    setTimeMode();
  }

  updater->update();
}

bool HpgRefProduct::setTimeMode() {
  ROS_INFO("Setting mode (internal state) to Time Mode");
  mode_ = TIME;

  // Set the Measurement & nav rate to user config
  // (survey-in sets nav_rate to 1 Hz regardless of user setting)
  if (!gps.configRate(meas_rate, nav_rate))
    ROS_ERROR("Failed to set measurement rate to %d ms %s %d", meas_rate, "navigation rate to ", nav_rate);
  // Enable the RTCM out messages
  if (!gps.configRtcm(rtcm_ids, rtcm_rates)) {
    ROS_ERROR("Failed to configure RTCM IDs");
    return false;
  }
  return true;
}

void HpgRefProduct::initializeRosDiagnostics() {
  updater->add("TMODE3", this, &HpgRefProduct::tmode3Diagnostics);
  updater->force_update();
}

void HpgRefProduct::tmode3Diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  if (mode_ == INIT) {
    stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    stat.message = "Not configured";
  } else if (mode_ == DISABLED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Disabled";
  } else if (mode_ == SURVEY_IN) {
    if (!last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.message = "Survey-In inactive and invalid";
    } else if (last_nav_svin_.active && !last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "Survey-In active but invalid";
    } else if (!last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Survey-In complete";
    } else if (last_nav_svin_.active && last_nav_svin_.valid) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Survey-In active and valid";
    }

    stat.add("iTOW [ms]", last_nav_svin_.iTOW);
    stat.add("Duration [s]", last_nav_svin_.dur);
    stat.add("# observations", last_nav_svin_.obs);
    stat.add("Mean X [m]", last_nav_svin_.meanX * 1e-2);
    stat.add("Mean Y [m]", last_nav_svin_.meanY * 1e-2);
    stat.add("Mean Z [m]", last_nav_svin_.meanZ * 1e-2);
    stat.add("Mean X HP [m]", last_nav_svin_.meanXHP * 1e-4);
    stat.add("Mean Y HP [m]", last_nav_svin_.meanYHP * 1e-4);
    stat.add("Mean Z HP [m]", last_nav_svin_.meanZHP * 1e-4);
    stat.add("Mean Accuracy [m]", last_nav_svin_.meanAcc * 1e-4);
  } else if (mode_ == FIXED) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Fixed Position";
  } else if (mode_ == TIME) {
    stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    stat.message = "Time";
  }
}

//
// U-Blox High Precision GNSS Rover
//
void HpgRovProduct::getRosParams() {
  // default to float, see CfgDGNSS message for details
  getRosUint("dgnss_mode", dgnss_mode_, ublox_msgs::CfgDGNSS::DGNSS_MODE_RTK_FIXED);
}

bool HpgRovProduct::configureUblox() {
  // Configure the DGNSS
  if (!gps.setDgnss(dgnss_mode_)) throw std::runtime_error(std::string("Failed to Configure DGNSS"));
  return true;
}

void HpgRovProduct::subscribe() {
  // Whether to publish Nav Relative Position NED
  nh->param("publish/nav/relposned", enabled["nav_relposned"], enabled["nav"]);
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps.subscribe<ublox_msgs::NavRELPOSNED>(boost::bind(&HpgRovProduct::callbackNavRelPosNed, this, _1), kSubscribeRate);
}

void HpgRovProduct::initializeRosDiagnostics() {
  freq_rtcm_ = UbloxTopicDiagnostic(std::string("rxmrtcm"), kRtcmFreqMin, kRtcmFreqMax, kRtcmFreqTol, kRtcmFreqWindow);
  updater->add("Carrier Phase Solution", this, &HpgRovProduct::carrierPhaseDiagnostics);
  updater->force_update();
}

void HpgRovProduct::carrierPhaseDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  uint32_t carr_soln = last_rel_pos_.flags & last_rel_pos_.FLAGS_CARR_SOLN_MASK;
  stat.add("iTow", last_rel_pos_.iTow);
  if (carr_soln & last_rel_pos_.FLAGS_CARR_SOLN_NONE || !(last_rel_pos_.flags & last_rel_pos_.FLAGS_DIFF_SOLN &&
                                                          last_rel_pos_.flags & last_rel_pos_.FLAGS_REL_POS_VALID)) {
    stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    stat.message = "None";
  } else {
    if (carr_soln & last_rel_pos_.FLAGS_CARR_SOLN_FLOAT) {
      stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.message = "Float";
    } else if (carr_soln & last_rel_pos_.FLAGS_CARR_SOLN_FIXED) {
      stat.level = diagnostic_msgs::DiagnosticStatus::OK;
      stat.message = "Fixed";
    }
    stat.add("Ref Station ID", last_rel_pos_.refStationId);

    double rel_pos_n = (last_rel_pos_.relPosN + (last_rel_pos_.relPosHPN * 1e-2)) * 1e-2;
    double rel_pos_e = (last_rel_pos_.relPosE + (last_rel_pos_.relPosHPE * 1e-2)) * 1e-2;
    double rel_pos_d = (last_rel_pos_.relPosD + (last_rel_pos_.relPosHPD * 1e-2)) * 1e-2;
    stat.add("Relative Position N [m]", rel_pos_n);
    stat.add("Relative Accuracy N [m]", last_rel_pos_.accN * 1e-4);
    stat.add("Relative Position E [m]", rel_pos_e);
    stat.add("Relative Accuracy E [m]", last_rel_pos_.accE * 1e-4);
    stat.add("Relative Position D [m]", rel_pos_d);
    stat.add("Relative Accuracy D [m]", last_rel_pos_.accD * 1e-4);
  }
}

void HpgRovProduct::callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED& m) {
  if (enabled["nav_relposned"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::NavRELPOSNED>("navrelposned", kROSQueueSize);
    publisher.publish(m);
  }

  last_rel_pos_ = m;
  updater->update();
}

//
// U-Blox High Precision Positioning Receiver
//

void HpPosRecProduct::subscribe() {
  // Whether to publish Nav Relative Position NED
  nh->param("publish/nav/relposned", enabled["nav_relposned"], enabled["nav"]);
  // Subscribe to Nav Relative Position NED messages (also updates diagnostics)
  gps.subscribe<ublox_msgs::NavRELPOSNED9>(boost::bind(&HpPosRecProduct::callbackNavRelPosNed, this, _1),
                                           kSubscribeRate);

  // Whether to publish the Heading info from Nav Relative Position NED
  nh->param("publish/nav/heading", enabled["nav_heading"], enabled["nav"]);
}

void HpPosRecProduct::callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9& m) {
  if (enabled["nav_relposned"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::NavRELPOSNED9>("navrelposned", kROSQueueSize);
    publisher.publish(m);
  }

  if (enabled["nav_heading"]) {
    static ros::Publisher imu_pub = nh->advertise<sensor_msgs::Imu>("navheading", kROSQueueSize);

    imu_.header.stamp = ros::Time::now();
    imu_.header.frame_id = frame_id;

    imu_.linear_acceleration_covariance[0] = -1;
    imu_.angular_velocity_covariance[0] = -1;

    double heading = static_cast<double>(m.relPosHeading) * 1e-5 / 180.0 * M_PI;
    tf::Quaternion orientation;
    orientation.setRPY(0, 0, heading);
    imu_.orientation.x = orientation[0];
    imu_.orientation.y = orientation[1];
    imu_.orientation.z = orientation[2];
    imu_.orientation.w = orientation[3];
    // Only heading is reported with an accuracy in 0.1mm units
    imu_.orientation_covariance[0] = 1000.0;
    imu_.orientation_covariance[4] = 1000.0;
    imu_.orientation_covariance[8] = pow(m.accHeading / 10000.0, 2);

    imu_pub.publish(imu_);
  }

  last_rel_pos_ = m;
  updater->update();
}

//
// U-Blox Time Sync Products, partially implemented.
//
void TimProduct::getRosParams() {}

bool TimProduct::configureUblox() {
  uint8_t r = 1;
  // Configure the reciever
  if (!gps.setUTCtime()) throw std::runtime_error(std::string("Failed to Configure TIM Product to UTC Time"));

  if (!gps.setTimtm2(r)) throw std::runtime_error(std::string("Failed to Configure TIM Product"));

  return true;
}

void TimProduct::subscribe() {
  ROS_INFO("TIM is Enabled: %u", enabled["tim"]);
  ROS_INFO("TIM-TM2 is Enabled: %u", enabled["tim_tm2"]);
  // Subscribe to TIM-TM2 messages (Time mark messages)
  nh->param("publish/tim/tm2", enabled["tim_tm2"], enabled["tim"]);

  gps.subscribe<ublox_msgs::TimTM2>(boost::bind(&TimProduct::callbackTimTM2, this, _1), kSubscribeRate);

  ROS_INFO("Subscribed to TIM-TM2 messages on topic tim/tm2");

  // Subscribe to SFRBX messages
  nh->param("publish/rxm/sfrb", enabled["rxm_sfrb"], enabled["rxm"]);
  if (enabled["rxm_sfrb"])
    gps.subscribe<ublox_msgs::RxmSFRBX>(boost::bind(publish<ublox_msgs::RxmSFRBX>, _1, "rxmsfrb"), kSubscribeRate);

  // Subscribe to RawX messages
  nh->param("publish/rxm/raw", enabled["rxm_raw"], enabled["rxm"]);
  if (enabled["rxm_raw"])
    gps.subscribe<ublox_msgs::RxmRAWX>(boost::bind(publish<ublox_msgs::RxmRAWX>, _1, "rxmraw"), kSubscribeRate);
}

void TimProduct::callbackTimTM2(const ublox_msgs::TimTM2& m) {
  if (enabled["tim_tm2"]) {
    static ros::Publisher publisher = nh->advertise<ublox_msgs::TimTM2>("timtm2", kROSQueueSize);
    static ros::Publisher time_ref_pub = nh->advertise<sensor_msgs::TimeReference>("interrupt_time", kROSQueueSize);

    // create time ref message and put in the data
    t_ref_.header.seq = m.risingEdgeCount;
    t_ref_.header.stamp = ros::Time::now();
    t_ref_.header.frame_id = frame_id;

    t_ref_.time_ref = ros::Time((m.wnR * 604800 + m.towMsR / 1000), (m.towMsR % 1000) * 1000000 + m.towSubMsR);

    std::ostringstream src;
    src << "TIM" << int(m.ch);
    t_ref_.source = src.str();

    t_ref_.header.stamp = ros::Time::now();  // create a new timestamp
    t_ref_.header.frame_id = frame_id;

    publisher.publish(m);
    time_ref_pub.publish(t_ref_);
  }

  updater->force_update();
}

void TimProduct::initializeRosDiagnostics() { updater->force_update(); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "ublox_gps");
  nh.reset(new ros::NodeHandle("~"));
  nh->param("debug", ublox_gps::debug, 1);
  if (ublox_gps::debug) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
      ros::console::notifyLoggerLevelsChanged();
  }
  UbloxNode node;
  return 0;
}
