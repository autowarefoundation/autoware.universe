#pragma once

#include <QLabel>
#include <QPushButton>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <optional>

namespace initialpose_plugins
{

class InitialPosePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  explicit InitialPosePanel(QWidget * parent = nullptr);
  virtual ~InitialPosePanel();

  void onInitialize() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;
  rclcpp::Logger logger_;

private Q_SLOTS:
  void toggle();

private:
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  QPushButton * toggle_button_{nullptr};
  QLabel * pose_label_{nullptr};
  QLabel * time_label_{nullptr};

  rclcpp::Clock::SharedPtr clock_{nullptr};
  std::optional<PoseCovStamped> last_initial_pose_{std::nullopt};

  void poseCallback(const PoseCovStamped & msg);

  void timerCallback();
};

}  // namespace initialpose_plugins
