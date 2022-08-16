#include "initialpose_panel.hpp"

#include <QHBoxLayout>
#include <QWidget>
#include <rviz_common/display_context.hpp>

#include <sstream>

namespace initialpose_plugins
{

InitialPosePanel::InitialPosePanel(QWidget * parent) : Panel(parent)
{
  // Button
  toggle_button_ = new QPushButton();
  toggle_button_->setText("CLEAR");
  toggle_button_->setFixedSize(QSize(60, 30));
  QObject::connect(toggle_button_, SIGNAL(clicked()), this, SLOT(toggle()));

  // Label
  pose_label_ = new QLabel();
  time_label_ = new QLabel();
  pose_label_->setText("");
  time_label_->setText("remaining time: ");

  // Layout
  QHBoxLayout * main_layout = new QHBoxLayout;
  main_layout->setContentsMargins(10, 10, 10, 10);

  // Add widges
  main_layout->addWidget(toggle_button_);
  main_layout->addWidget(pose_label_);
  main_layout->addWidget(time_label_);

  setLayout(main_layout);
}

InitialPosePanel::~InitialPosePanel()
{
  timer_.reset();
  pose_sub_.reset();
  pose_pub_.reset();
}

void InitialPosePanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  clock_ = node->get_clock();

  auto sub_cb = std::bind(&InitialPosePanel::poseCallback, this, std::placeholders::_1);
  pose_sub_ = node->create_subscription<PoseCovStamped>("/initialpose", 10, std::move(sub_cb));
  pose_pub_ = node->create_publisher<PoseCovStamped>("/initialpose", 10);

  auto timer_cb = std::bind(&InitialPosePanel::timerCallback, this);
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(node, clock_, 100ms, std::move(timer_cb));
}

void InitialPosePanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  if (last_initial_pose_.has_value()) {
    auto pos = last_initial_pose_->pose.pose.position;
    auto ori = last_initial_pose_->pose.pose.orientation;
    config.mapSetValue("last_initial_pose.x", pos.x);
    config.mapSetValue("last_initial_pose.y", pos.y);
    config.mapSetValue("last_initial_pose.w", ori.w);
    config.mapSetValue("last_initial_pose.z", ori.z);
    ulong t = rclcpp::Time(last_initial_pose_->header.stamp).nanoseconds();
    QVariant var;
    var.setValue(t);
    config.mapSetValue("last_initial_pose.t", var);
  }
}

void InitialPosePanel::load(const rviz_common::Config & config)
{
  Panel::load(config);

  QVariant var;
  QVariant x, y, w, z;
  bool f1 = config.mapGetValue("last_initial_pose.x", &x);
  bool f2 = config.mapGetValue("last_initial_pose.y", &y);
  bool f3 = config.mapGetValue("last_initial_pose.w", &w);
  bool f4 = config.mapGetValue("last_initial_pose.z", &z);
  bool f5 = config.mapGetValue("last_initial_pose.t", &var);

  if (f1 & f2 & f3 & f4 & f5) {
    rclcpp::Time stamp(var.value<ulong>());
    last_initial_pose_ = PoseCovStamped();
    last_initial_pose_->header.stamp = stamp;
    last_initial_pose_->pose.pose.position.x = x.value<double>();
    last_initial_pose_->pose.pose.position.y = y.value<double>();
    last_initial_pose_->pose.pose.orientation.w = w.value<double>();
    last_initial_pose_->pose.pose.orientation.z = z.value<double>();
    poseCallback(last_initial_pose_.value());
  }
}

void InitialPosePanel::timerCallback()
{
  if (last_initial_pose_.has_value()) {
    rclcpp::Time stamp = clock_->now();

    rclcpp::Time target = last_initial_pose_->header.stamp;
    double dt = (target - stamp).seconds();
    std::stringstream ss;
    ss << std::setprecision(3) << std::fixed << dt << " sec";
    time_label_->setText(ss.str().c_str());

    if ((dt < 0) & (last_dt_ > 0)) {
      pose_pub_->publish(last_initial_pose_.value());
    }
    QFont font;
    if (dt > 0) font.setBold(true);
    time_label_->setFont(font);
    last_dt_ = dt;
  }
}

void InitialPosePanel::poseCallback(const PoseCovStamped & msg)
{
  last_initial_pose_ = msg;

  auto pos = msg.pose.pose.position;
  auto ori = msg.pose.pose.orientation;

  double theta = 2 * std::atan2(ori.w, ori.z);
  double deg = theta * 180 / M_PI;
  std::stringstream ss;
  ss << std::setprecision(1) << std::fixed;
  ss << "X:" << pos.x << ", Y=" << pos.y << ", Deg=" << deg;
  pose_label_->setText(ss.str().c_str());
  time_label_->setText("0");
}

void InitialPosePanel::toggle()
{
  last_initial_pose_ = std::nullopt;
  pose_label_->setText("");
  time_label_->setText("remaining time: ");
}

}  // namespace initialpose_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(initialpose_plugins::InitialPosePanel, rviz_common::Panel)
