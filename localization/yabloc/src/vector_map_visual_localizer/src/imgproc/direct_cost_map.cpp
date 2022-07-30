#include "imgproc/direct_cost_map.hpp"

#include <iostream>

namespace imgproc
{
cv::Mat directCostMap(const cv::Mat & cost_map, const cv::Mat & intensity)
{
  std::vector<std::vector<int>> distances;
  distances.resize(cost_map.rows);
  for (int i = 0; i < cost_map.rows; i++) {
    distances.at(i).resize(cost_map.cols);
    std::fill(distances.at(i).begin(), distances.at(i).end(), -1);

    const uchar * intensity_ptr = intensity.ptr<uchar>(i);
    for (int j = 0; j < cost_map.cols; j++) {
      if (intensity_ptr[j] != 0) distances.at(i).at(j) = 0;
    }
  }

  cv::Mat dst = cost_map.clone();

  // Forward
  int cnt = 0;
  for (int r = 1; r < cost_map.rows; r++) {
    const uchar * upper_ptr = dst.ptr<uchar>(r - 1);
    uchar * current_ptr = dst.ptr<uchar>(r);

    for (int c = 1; c < cost_map.cols; c++) {
      int u = distances.at(r - 1).at(c);
      int l = distances.at(r).at(c - 1);

      if ((u == -1) & (l == -1)) continue;
      u = (u < 0) ? std::numeric_limits<int>::max() : u;
      l = (l < 0) ? std::numeric_limits<int>::max() : l;
      cnt++;
      if (u < l) {
        distances.at(r).at(c) = u + 1;
        if (upper_ptr[c] == 0) {
          std::cout << "r=" << r << " c=" << c << std::endl;
          std::cout << "upper_ptr[c]==0" << std::endl;
          std::cout << u << " " << l << std::endl;
          std::cout << +upper_ptr[c] << std::endl;
          std::cout << +current_ptr[c] << std::endl;
          exit(EXIT_FAILURE);
        }
        current_ptr[c] = upper_ptr[c];
      } else {
        distances.at(r).at(c) = l + 1;
        if (current_ptr[c - 1] == 0) {
          std::cout << "r=" << r << " c=" << c << std::endl;
          std::cout << "upper_ptr[c-1]==0" << std::endl;
          std::cout << u << " " << l << std::endl;
          std::cout << +current_ptr[c - 1] << std::endl;
          std::cout << +current_ptr[c] << std::endl;
          exit(EXIT_FAILURE);
        }
        current_ptr[c] = current_ptr[c - 1];
      }
    }
  }
  std::cout << cnt << std::endl;

  // Backward

  return dst;
}

cv::Mat visualizeDirectionMap(const cv::Mat & cost_map)
{
  cv::Mat s = 255 * cv::Mat::ones(cost_map.size(), CV_8UC1);
  cv::Mat v = 255 * cv::Mat::ones(cost_map.size(), CV_8UC1);
  cv::Mat hsv, rgb;
  cv::merge(std::vector<cv::Mat>{cost_map, s, v}, hsv);
  cv::cvtColor(hsv, rgb, cv::COLOR_HSV2BGR);

  for (int r = 0; r < cost_map.rows; r++) {
    const uchar * src_ptr = cost_map.ptr<uchar>(r);
    cv::Vec3b * dst_ptr = rgb.ptr<cv::Vec3b>(r);
    for (int c = 0; c < cost_map.cols; c++) {
      if (src_ptr[c] == 0) dst_ptr[c] = cv::Vec3b(0, 0, 0);
    }
  }
  return rgb;
}

}  // namespace imgproc

#include <opencv4/opencv2/highgui.hpp>

int main()
{
  // 0~180
  cv::Mat raw_map = cv::Mat::zeros(cv::Size(800, 800), CV_8UC1);
  cv::line(raw_map, cv::Point(400, 0), cv::Point(400, 800), cv::Scalar::all(10), 3);
  cv::line(raw_map, cv::Point(0, 400), cv::Point(800, 400), cv::Scalar::all(80), 3);
  cv::line(raw_map, cv::Point(0, 0), cv::Point(400, 400), cv::Scalar::all(160), 3);
  cv::line(raw_map, cv::Point(400, 400), cv::Point(800, 800), cv::Scalar::all(30), 3);

  cv::Mat intensity = raw_map.clone();

  cv::Mat directed = imgproc::directCostMap(raw_map, intensity);
  cv::Mat show1 = imgproc::visualizeDirectionMap(raw_map);
  cv::Mat show2 = imgproc::visualizeDirectionMap(directed);
  cv::hconcat(show1, show2, show1);
  cv::imshow("raw + directed", show1);
  cv::waitKey(0);
}