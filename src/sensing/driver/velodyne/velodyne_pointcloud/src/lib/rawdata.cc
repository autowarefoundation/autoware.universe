/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR packets into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 *
 *  HDL-64E S2 calibration support provided by Nick Hillier
 */

#include <math.h>
#include <fstream>

#include <angles/angles.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_rawdata
{
inline float SQR(float val) { return val * val; }

////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData() {}

/** Update parameters: conversions and update */
void RawData::setParameters(
  double min_range, double max_range, double view_direction, double view_width)
{
  config_.min_range = min_range;
  config_.max_range = max_range;

  //converting angle parameters into the velodyne reference (rad)
  config_.tmp_min_angle = view_direction + view_width / 2;
  config_.tmp_max_angle = view_direction - view_width / 2;

  //computing positive modulo to keep theses angles into [0;2*M_PI]
  config_.tmp_min_angle = fmod(fmod(config_.tmp_min_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
  config_.tmp_max_angle = fmod(fmod(config_.tmp_max_angle, 2 * M_PI) + 2 * M_PI, 2 * M_PI);

  //converting into the hardware velodyne ref (negative yaml and degrees)
  //adding 0.5 perfomrs a centered double to int conversion
  config_.min_angle = 100 * (2 * M_PI - config_.tmp_min_angle) * 180 / M_PI + 0.5;
  config_.max_angle = 100 * (2 * M_PI - config_.tmp_max_angle) * 180 / M_PI + 0.5;
  if (config_.min_angle == config_.max_angle) {
    //avoid returning empty cloud if min_angle = max_angle
    config_.min_angle = 0;
    config_.max_angle = 36000;
  }
}

int RawData::scansPerPacket() const
{
  if (calibration_.num_lasers == 16) {
    return BLOCKS_PER_PACKET * VLP16_FIRINGS_PER_BLOCK * VLP16_SCANS_PER_FIRING;
  } else {
    return BLOCKS_PER_PACKET * SCANS_PER_BLOCK;
  }
}

int RawData::getNumLasers() const { return calibration_.num_lasers; }

double RawData::getMaxRange() const { return config_.max_range; }

double RawData::getMinRange() const { return config_.min_range; }

/** Set up for on-line operation. */
int RawData::setup(ros::NodeHandle private_nh)
{
  // get path to angles.config file for this device
  if (!private_nh.getParam("calibration", config_.calibrationFile)) {
    ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

    // have to use something: grab unit test version as a default
    std::string pkgPath = ros::package::getPath("velodyne_pointcloud");
    config_.calibrationFile = pkgPath + "/params/64e_utexas.yaml";
  }

  ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
    return -1;
  }

  ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }

  for (uint8_t i = 0; i < 16; i++) {
    vls_128_laser_azimuth_cache[i] = (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
  }

  return 0;
}

/** Set up for offline operation */
int RawData::setupOffline(std::string calibration_file, double max_range_, double min_range_)
{
  config_.max_range = max_range_;
  config_.min_range = min_range_;
  ROS_INFO_STREAM(
    "data ranges to publish: [" << config_.min_range << ", " << config_.max_range << "]");

  config_.calibrationFile = calibration_file;

  ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    ROS_ERROR_STREAM("Unable to open calibration file: " << config_.calibrationFile);
    return -1;
  }

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }


  for (uint8_t i = 0; i < 16; i++) {
    vls_128_laser_azimuth_cache[i] = (VLS128_CHANNEL_TDURATION / VLS128_SEQ_TDURATION) * (i + i / 8);
  }
  return 0;
}

/** @brief convert raw packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
void RawData::unpack(const velodyne_msgs::VelodynePacket & pkt, DataContainerBase & data)
{
  using velodyne_pointcloud::LaserCorrection;
  ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);

  /** special parsing for the VLP16 **/
  if (calibration_.num_lasers == 16) {
    unpack_vlp16(pkt, data);
    return;
  }

  const raw_packet_t * raw = (const raw_packet_t *)&pkt.data[0];

  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    // upper bank lasers are numbered [0..31]
    // NOTE: this is a change from the old velodyne_common implementation

    int bank_origin = 0;
    if (raw->blocks[i].header == LOWER_BANK) {
      // lower bank lasers are [32..63]
      bank_origin = 32;
    }
    /** special parsing for the VLS128 **/
    else if (calibration_.num_lasers == 128)
    {
      unpack_vls128(pkt, data);
      return;
    }

    for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
      float x, y, z;
      float intensity;
      const uint8_t laser_number = j + bank_origin;

      const LaserCorrection & corrections = calibration_.laser_corrections[laser_number];

      /** Position Calculation */
      const raw_block_t & block = raw->blocks[i];
      union two_bytes tmp;
      tmp.bytes[0] = block.data[k];
      tmp.bytes[1] = block.data[k + 1];
      // if (tmp.bytes[0]==0 &&tmp.bytes[1]==0 ) //no laser beam return
      // {
      //   continue;
      // }

      // float distance = tmp.uint * calibration_.distance_resolution_m;
      // distance += corrections.dist_correction;
      // if (!pointInRange(distance)) continue;

      /*condition added to avoid calculating points which are not
          in the interesting defined area (min_angle < area < max_angle)*/
      if (
        (block.rotation >= config_.min_angle && block.rotation <= config_.max_angle &&
         config_.min_angle < config_.max_angle) ||
        (config_.min_angle > config_.max_angle && (raw->blocks[i].rotation <= config_.max_angle ||
                                                   raw->blocks[i].rotation >= config_.min_angle))) {
        float distance = tmp.uint * calibration_.distance_resolution_m;

        bool is_invalid_distance = false;
        if (distance == 0.0) {
          is_invalid_distance = true;
          distance = 0.3;
        } else {
          distance += corrections.dist_correction;
        }

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle = cos_rot_table_[block.rotation] * cos_rot_correction +
                              sin_rot_table_[block.rotation] * sin_rot_correction;
        float sin_rot_angle = sin_rot_table_[block.rotation] * cos_rot_correction -
                              cos_rot_table_[block.rotation] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
        if (xx < 0) xx = -xx;
        if (yy < 0) yy = -yy;

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;
        if (corrections.two_pt_correction_available) {
          distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                              (xx - 2.4) / (25.04 - 2.4) +
                            corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                              (yy - 1.93) / (25.04 - 1.93) +
                            corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        ///the expression wiht '-' is proved to be better than the one with '+'
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */

        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[i].data[k + 2];

        float focal_offset =
          256 * (1 - corrections.focal_distance / 13100) * (1 - corrections.focal_distance / 13100);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope *
                     (std::abs(focal_offset - 256 * SQR(1 - static_cast<float>(tmp.uint) / 65535)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        double time_stamp =
          i * 55.296 / 1000.0 / 1000.0 + j * 2.304 / 1000.0 / 1000.0 + pkt.stamp.toSec();
        if (is_invalid_distance) {
          data.addPoint(
            x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[i].rotation, 0,
            intensity, time_stamp);
        } else {
          data.addPoint(
            x_coord, y_coord, z_coord, corrections.laser_ring, raw->blocks[i].rotation, distance,
            intensity, time_stamp);
        }
      }
    }
  }
}

/** @brief convert raw VLP16 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
void RawData::unpack_vlp16(const velodyne_msgs::VelodynePacket & pkt, DataContainerBase & data)
{
  float azimuth;
  float azimuth_diff;
  int raw_azimuth_diff;
  static float last_azimuth_diff = 0;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  const raw_packet_t * raw = (const raw_packet_t *)&pkt.data[0];

  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {
    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header) {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      ROS_WARN_STREAM_THROTTLE(
        60, "skipping invalid VLP-16 packet: block " << block << " header value is "
                                                     << raw->blocks[block].header);
      return;  // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - 1)) {
      raw_azimuth_diff = raw->blocks[block + 1].rotation - raw->blocks[block].rotation;
      azimuth_diff = (float)((36000 + raw_azimuth_diff) % 36000);
      // // some packets contain an angle overflow where azimuth_diff < 0
      // if(raw_azimuth_diff < 0)//raw->blocks[block+1].rotation - raw->blocks[block].rotation < 0)
      //   {
      //     ROS_WARN_STREAM_THROTTLE(60, "Packet containing angle overflow, first angle: " << raw->blocks[block].rotation << " second angle: " << raw->blocks[block+1].rotation);
      //     // if last_azimuth_diff was not zero, we can assume that the velodyne's speed did not change very much and use the same difference
      //     if(last_azimuth_diff > 0){
      //       azimuth_diff = last_azimuth_diff;
      //     }
      //     // otherwise we are not able to use this data
      //     // TODO: we might just not use the second 16 firings
      //     else{
      //       continue;
      //     }
      //   }
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
        velodyne_pointcloud::LaserCorrection & corrections = calibration_.laser_corrections[dsr];

        /** Position Calculation */
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];

        // float distance = tmp.uint * calibration_.distance_resolution_m;
        // distance += corrections.dist_correction;

        // skip the point if out of range
        //if ( !pointInRange(distance)) continue;

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f =
          azimuth + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) /
                     VLP16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

        /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
        if (
          (azimuth_corrected >= config_.min_angle && azimuth_corrected <= config_.max_angle &&
           config_.min_angle < config_.max_angle) ||
          (config_.min_angle > config_.max_angle &&
           (azimuth_corrected <= config_.max_angle || azimuth_corrected >= config_.min_angle))) {
          // convert polar coordinates to Euclidean XYZ
          float distance = tmp.uint * calibration_.distance_resolution_m;
          bool is_invalid_distance = false;
          if (distance == 0.0) {
            is_invalid_distance = true;
            distance = 0.3;
          } else {
            distance += corrections.dist_correction;
          }

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                                sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle = sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                                cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0) xx = -xx;
          if (yy < 0) yy = -yy;

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available) {
            distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) *
                                (xx - 2.4) / (25.04 - 2.4) +
                              corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) *
                                (yy - 1.93) / (25.04 - 1.93) +
                              corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
             * was added to the expression due to the mathemathical
             * model we used.
             */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k + 2];

          float focal_offset = 256 * SQR(1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (std::abs(focal_offset - 256 * SQR(1 - tmp.uint / 65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          double time_stamp = (block * 2 + firing) * 55.296 / 1000.0 / 1000.0 +
                              dsr * 2.304 / 1000.0 / 1000.0 + pkt.stamp.toSec();
          if (is_invalid_distance) {
            data.addPoint(
              x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, 0, intensity,
              time_stamp);
          } else {
            data.addPoint(
              x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance,
              intensity, time_stamp);
          }
        }
      }
    }
  }
}


  /** @brief convert raw VLS128 packet to point cloud
   *
   *  @param pkt raw packet to unpack
   *  @param pc shared pointer to point cloud (points are appended)
   */
  // void RawData::unpack_vls128(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase &data, const ros::Time& scan_start_time) {
  void RawData::unpack_vls128(const velodyne_msgs::VelodynePacket &pkt, DataContainerBase &data) {
    float last_azimuth_diff = 0;
    float azimuth_diff, azimuth_corrected_f;
    uint16_t azimuth, azimuth_next, azimuth_corrected;
    float x_coord, y_coord, z_coord;
    float distance, intensity;
    const raw_packet_t *raw = (const raw_packet_t *) &pkt.data[0];
    union two_bytes tmp;

    // float time_diff_start_to_this_packet = (pkt.stamp - scan_start_time).toSec();

    float cos_vert_angle, sin_vert_angle, cos_rot_correction, sin_rot_correction;
    float cos_rot_angle, sin_rot_angle;
    float xy_distance;

    uint8_t laser_number, firing_order;
    bool dual_return = (pkt.data[1204] == 57);

    for (int block = 0; block < BLOCKS_PER_PACKET - (4* dual_return); block++) {
      // cache block for use
      const raw_block_t &current_block = raw->blocks[block];

      int bank_origin = 0;
      // Used to detect which bank of 32 lasers is in this block
      switch (current_block.header) {
        case VLS128_BANK_1:
          bank_origin = 0;
          break;
        case VLS128_BANK_2:
          bank_origin = 32;
          break;
        case VLS128_BANK_3:
          bank_origin = 64;
          break;
        case VLS128_BANK_4:
          bank_origin = 96;
          break;
        default:
          // ignore packets with mangled or otherwise different contents
          // Do not flood the log with messages, only issue at most one
          // of these warnings per minute.
          ROS_WARN_STREAM_THROTTLE(60, "skipping invalid VLS-128 packet: block "
                    << block << " header value is "
                    << raw->blocks[block].header);
          return; // bad packet: skip the rest
      }

      // Calculate difference between current and next block's azimuth angle.
      if (block == 0) {
        azimuth = current_block.rotation;
      } else {
        azimuth = azimuth_next;
      }
      if (block < (BLOCKS_PER_PACKET - (1+dual_return))) {
        // Get the next block rotation to calculate how far we rotate between blocks
        azimuth_next = raw->blocks[block + (1+dual_return)].rotation;

        // Finds the difference between two sucessive blocks
        azimuth_diff = (float)((36000 + azimuth_next - azimuth) % 36000);

        // This is used when the last block is next to predict rotation amount
        last_azimuth_diff = azimuth_diff;
      } else {
        // This makes the assumption the difference between the last block and the next packet is the
        // same as the last to the second to last.
        // Assumes RPM doesn't change much between blocks
        azimuth_diff = (block == BLOCKS_PER_PACKET - (4*dual_return)-1) ? 0 : last_azimuth_diff;
      }

      // condition added to avoid calculating points which are not in the interesting defined area (min_angle < area < max_angle)
      if ((config_.min_angle < config_.max_angle && azimuth >= config_.min_angle && azimuth <= config_.max_angle) || (config_.min_angle > config_.max_angle)) {
        for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE) {
          // distance extraction
          tmp.bytes[0] = current_block.data[k];
          tmp.bytes[1] = current_block.data[k + 1];
          if (tmp.bytes[0]==0 &&tmp.bytes[1]==0 ) //no laser beam return
          {
            continue;
          }

          // if (data.pointInRange(distance)) {
          {
            laser_number = j + bank_origin;   // Offset the laser in this block by which block it's in
            firing_order = laser_number / 8;  // VLS-128 fires 8 lasers at a time

            velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[laser_number];

            distance = tmp.uint * VLP128_DISTANCE_RESOLUTION;
            bool is_invalid_distance = false;
            if(distance == 0.0) {
                is_invalid_distance = true;
                distance = 0.3;
            }
            else {
              distance += corrections.dist_correction;
            }

            // correct for the laser rotation as a function of timing during the firings
            azimuth_corrected_f = azimuth + (azimuth_diff * vls_128_laser_azimuth_cache[firing_order]);
            azimuth_corrected = ((uint16_t) round(azimuth_corrected_f)) % 36000;

            /*condition added to avoid calculating points which are not
            in the interesting defined area (min_angle < area < max_angle)*/
            if ((azimuth_corrected >= config_.min_angle
                 && azimuth_corrected <= config_.max_angle
                 && config_.min_angle < config_.max_angle)
                 ||(config_.min_angle > config_.max_angle
                 && (azimuth_corrected <= config_.max_angle
                 || azimuth_corrected >= config_.min_angle))){

              // convert polar coordinates to Euclidean XYZ
              cos_vert_angle = corrections.cos_vert_correction;
              sin_vert_angle = corrections.sin_vert_correction;
              cos_rot_correction = corrections.cos_rot_correction;
              sin_rot_correction = corrections.sin_rot_correction;

              // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
              // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
              cos_rot_angle =
                cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                sin_rot_table_[azimuth_corrected] * sin_rot_correction;
              sin_rot_angle =
                sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                cos_rot_table_[azimuth_corrected] * sin_rot_correction;

              // Compute the distance in the xy plane (w/o accounting for rotation)
              xy_distance = distance * cos_vert_angle;

              /** Use standard ROS coordinate system (right-hand rule) */
              // append this point to the cloud
              //VPoint point;
              //point.ring = corrections.laser_ring;
              //point.x = xy_distance * cos_rot_angle;    // velodyne y
              //point.y = -(xy_distance * sin_rot_angle); // velodyne x
              //point.z = distance * sin_vert_angle;      // velodyne z
              //// Intensity extraction
              //point.intensity = current_block.data[k + 2];
              //pc.points.push_back(point);
              //++pc.width;
              x_coord = xy_distance * cos_rot_angle;    // velodyne y
              y_coord = -(xy_distance * sin_rot_angle); // velodyne x
              z_coord = distance * sin_vert_angle;      // velodyne z
              intensity = current_block.data[k + 2];

              // float time = 0;
              // if (timing_offsets.size())
              //   time = timing_offsets[block][j] + time_diff_start_to_this_packet;

              double time_stamp = block * 55.3 / 1000.0 / 1000.0 + j * 2.665 / 1000.0 / 1000.0 + pkt.stamp.toSec();

              if (is_invalid_distance) {
                data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, 0, intensity, time_stamp);
              }
              else {
                data.addPoint(x_coord, y_coord, z_coord, corrections.laser_ring, azimuth_corrected, distance, intensity, time_stamp);
              }
            }
          }
        }
        // data.newLine();
      }
    }
  }

} // namespace velodyne_rawdata
