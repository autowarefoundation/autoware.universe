/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#pragma once
#include <opencv2/core.hpp>

namespace cv::lsd
{

//! Variants of Line Segment %Detector
enum LineSegmentDetectorModes {
  LSD_REFINE_NONE = 0,  //!< No refinement applied
  LSD_REFINE_STD = 1,   //!< Standard refinement is applied. E.g. breaking arches into smaller straighter line approximations.
  LSD_REFINE_ADV = 2    //!< Advanced refinement. Number of false alarms is calculated, lines are
                        //!< refined through increase of precision, decrement in size, etc.
};

class CV_EXPORTS_W LineSegmentDetector : public cv::Algorithm
{
public:
  /** @brief Finds lines in the input image.
    This is the output of the default parameters of the algorithm on the above shown image.
    ![image](pics/building_lsd.png)
    @param image A grayscale (CV_8UC1) input image. If only a roi needs to be selected, use:
    `lsd_ptr-\>detect(image(roi), lines, ...); lines += Scalar(roi.x, roi.y, roi.x, roi.y);`
    @param lines A vector of Vec4f elements specifying the beginning and ending point of a line. Where
    Vec4f is (x1, y1, x2, y2), point 1 is the start, point 2 - end. Returned lines are strictly
    oriented depending on the gradient.
    @param width Vector of widths of the regions, where the lines are found. E.g. Width of line.
    @param prec Vector of precisions with which the lines are found.
    @param nfa Vector containing number of false alarms in the line region, with precision of 10%. The
    bigger the value, logarithmically better the detection.
    - -1 corresponds to 10 mean false alarms
    - 0 corresponds to 1 mean false alarm
    - 1 corresponds to 0.1 mean false alarms
    This vector will be calculated only when the objects type is #LSD_REFINE_ADV.
    */
  CV_WRAP virtual void detect(InputArray image, OutputArray lines,
      OutputArray width = noArray(), OutputArray prec = noArray(),
      OutputArray nfa = noArray())
      = 0;

  /** @brief Draws the line segments on a given image.
    @param image The image, where the lines will be drawn. Should be bigger or equal to the image,
    where the lines were found.
    @param lines A vector of the lines that needed to be drawn.
     */
  CV_WRAP virtual void drawSegments(InputOutputArray image, InputArray lines) = 0;

  /** @brief Draws two groups of lines in blue and red, counting the non overlapping (mismatching) pixels.
    @param size The size of the image, where lines1 and lines2 were found.
    @param lines1 The first group of lines that needs to be drawn. It is visualized in blue color.
    @param lines2 The second group of lines. They visualized in red color.
    @param image Optional image, where the lines will be drawn. The image should be color(3-channel)
    in order for lines1 and lines2 to be drawn in the above mentioned colors.
     */
  CV_WRAP virtual int compareSegments(const Size& size, InputArray lines1, InputArray lines2, InputOutputArray image = noArray()) = 0;

  virtual ~LineSegmentDetector() {}
};

/** @brief Creates a smart pointer to a LineSegmentDetector object and initializes it.
The LineSegmentDetector algorithm is defined using the standard values. Only advanced users may want
to edit those, as to tailor it for their own application.
@param refine The way found lines will be refined, see #LineSegmentDetectorModes
@param scale The scale of the image that will be used to find the lines. Range (0..1].
@param sigma_scale Sigma for Gaussian filter. It is computed as sigma = sigma_scale/scale.
@param quant Bound to the quantization error on the gradient norm.
@param ang_th Gradient angle tolerance in degrees.
@param log_eps Detection threshold: -log10(NFA) \> log_eps. Used only when advance refinement is chosen.
@param density_th Minimal density of aligned region points in the enclosing rectangle.
@param n_bins Number of bins in pseudo-ordering of gradient modulus.
 */
CV_EXPORTS_W Ptr<LineSegmentDetector> createLineSegmentDetector(
    int refine = LSD_REFINE_STD, double scale = 0.8,
    double sigma_scale = 0.6, double quant = 2.0, double ang_th = 22.5,
    double log_eps = 0, double density_th = 0.7, int n_bins = 1024);

}  // namespace cv::lsd
