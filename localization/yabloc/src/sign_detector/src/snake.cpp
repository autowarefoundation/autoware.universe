#include <opencv4/opencv2/opencv.hpp>

#include <boost/range/adaptors.hpp>

#include <iostream>

float ALPHA = 1.0;
float BETA = 1.0;
float GAMMA = 0.5;

cv::Mat makeEdgeImage(const cv::Mat & src_image)
{
  cv::Mat gray_image;
  cv::cvtColor(src_image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Size kernel_size(2 * 3 + 1, 2 * 3 + 1);
  cv::GaussianBlur(gray_image, gray_image, kernel_size, 0, 0);
  cv::Mat edge_image;
  cv::Laplacian(gray_image, edge_image, CV_16SC1, 5);

  cv::Mat show_edge_image;
  cv::convertScaleAbs(edge_image, show_edge_image, (3 + 1) * 0.25);
  cv::imshow("Laplacian", show_edge_image);

  return edge_image;
}

std::vector<cv::Point2i> makeInitialContour(const cv::Rect2i & rect)
{
  std::vector<cv::Point2i> points;
  std::vector<cv::Point2i> corners = {
    rect.tl(), rect.tl() + cv::Point2i(rect.width, 0), rect.br(),
    rect.tl() + cv::Point2i(0, rect.height)};

  const int N = 5;
  for (int i = 0; i < 4; i++) {
    cv::Point2i from = corners.at(i);
    cv::Point2i to = corners.at((i + 1) % 4) - from;
    for (int j = 0; j < N; j++) {
      points.push_back(from + 1.0f * to * j / N);
    }
  }

  return points;
}

std::vector<cv::Point2i> neighborIterator(const cv::Point2i & pt)
{
  std::vector<cv::Point2i> neighbors;
  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      neighbors.push_back(cv::Point2i(pt.x + i, pt.y + j));
    }
  }
  return neighbors;
}

float computeContourEnergy(const cv::Point2i & pre, const cv::Point2i & cur)
{
  cv::Point2i dp = pre - cur;
  return dp.x * dp.x + dp.y * dp.y;
}

float computeCurveEnergy(const cv::Point2i & pre, const cv::Point2i & cur, const cv::Point2i & sub)
{
  cv::Point2i dp = (sub - 2 * cur + pre);
  return dp.x * dp.x + dp.y * dp.y;
}

float computeColorEnergy(float grad, float grad_max, float grad_min)
{
  return -((grad - grad_min) / std::max(grad_max - grad_min, 5.f));
}

std::pair<float, float> neighborMinMaxGrad(const cv::Mat & edge_image, const cv::Point2i & pt)
{
  std::vector<float> grads;
  for (const cv::Point2i & p : neighborIterator(pt)) {
    grads.push_back(edge_image.at<short>(pt));
  }
  float min_grad = *std::min_element(grads.begin(), grads.end());
  float max_grad = *std::max_element(grads.begin(), grads.end());
  return {min_grad, max_grad};
}

std::tuple<cv::Point2i, cv::Point2i, cv::Point2i> triplet(
  const std::vector<cv::Point2i> & points, int index)
{
  const int N = points.size();
  int pre = (index - 1 + N) % N;
  int cur = index;
  int sub = (index + 1) % N;
  return {points.at(pre), points.at(cur), points.at(sub)};
}

std::vector<cv::Point2i> iterate(const cv::Mat & edge_image, std::vector<cv::Point2i> & points)
{
  std::optional<cv::Point2i> last_pre = std::nullopt;
  cv::Rect2i size(0, 0, edge_image.cols, edge_image.rows);

  std::vector<cv::Point2i> new_points;
  for (int i = 0; i < points.size(); i++) {
    auto [pre, cur, sub] = triplet(points, i);

    if (last_pre.has_value()) pre = last_pre.value();

    std::vector<cv::Point2i> neighbors = neighborIterator(cur);

    auto [grad_min, grad_max] = neighborMinMaxGrad(edge_image, cur);

    std::vector<cv::Point2i> energy_point;
    std::vector<float> energy_contour, energy_curve, energy_color;
    for (const auto & neighbor : neighbors | boost::adaptors::indexed()) {
      if (!size.contains(neighbor.value())) continue;
      float grad = edge_image.at<short>(neighbor.value());
      energy_point.push_back(neighbor.value());
      energy_contour.push_back(computeContourEnergy(pre, neighbor.value()));
      energy_curve.push_back(computeCurveEnergy(pre, neighbor.value(), sub));
      energy_color.push_back(computeColorEnergy(grad, grad_max, grad_min));
    }
    float max_contour = *std::max_element(energy_contour.begin(), energy_contour.end());
    float max_curve = *std::max_element(energy_curve.begin(), energy_curve.end());
    float max_color = *std::max_element(energy_color.begin(), energy_color.end());

    std::vector<std::pair<cv::Point2i, float>> energy_map;
    for (int index = 0; index < energy_point.size(); index++) {
      cv::Point2i pt = energy_point.at(index);
      float energy = ALPHA * energy_contour.at(index) / max_contour +
                     BETA * energy_curve.at(index) / max_curve +
                     GAMMA * energy_color.at(index) / max_color;
      energy_map.emplace_back(pt, energy);
    }

    auto min_key_value = *std::min_element(
      energy_map.begin(), energy_map.end(),
      [](const auto & a, const auto & b) { return a.second < b.second; });
    new_points.push_back(min_key_value.first);

    last_pre = min_key_value.first;
  }

  return new_points;
}

int main(int argc, char * argv[])
{
  if (argc == 4) {
    ALPHA = std::atof(argv[1]);
    BETA = std::atof(argv[2]);
    GAMMA = std::atof(argv[3]);
  }

  cv::Mat src_image = cv::imread("sample_data/sign01.png");

  cv::Mat edge_image = makeEdgeImage(src_image);

  std::vector<cv::Point2i> points = makeInitialContour(cv::Rect2i(350, 10, 100, 100));
  cv::polylines(src_image, points, true, cv::Scalar(0, 0, 255), 1);

  for (int itr = 0; itr < 100; itr++) {
    cv::Mat show_image = src_image.clone();
    points = iterate(edge_image, points);
    cv::polylines(show_image, points, false, cv::Scalar(0, 0, 255), 1);
    cv::imshow("src", show_image);
    cv::waitKey(50);
  }
  cv::waitKey(0);
}