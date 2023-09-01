#ifndef MATRIX_DISPLAY_HPP_
#define MATRIX_DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <QTableWidget>
#include <QTableWidgetItem>

#include <vector>

class TargetObjectTypePanel : public rviz_common::Panel
{
Q_OBJECT

public:
    TargetObjectTypePanel(QWidget* parent = 0);

protected:
    QTableWidget* matrix_widget_;
    std::shared_ptr<rclcpp::Node> node_;
    std::vector<std::string> modules_ = {
      "avoidance",
      "avoidance_by_lane_change",
      "lane_change",
      "obstacle_cruise (inside)",
      "obstacle_cruise (outside)",
      "obstacle_stop",
      "obstacle_slowdown"};
    std::vector<std::string> targets_ = {"car",     "truck",   "bus",        "trailer",
                                         "unknown", "bicycle", "motorcycle", "pedestrian"};

    struct ParamNameEnableObject
    {
      std::string node;
      std::string ns;
      std::unordered_map<std::string, std::string> name;
  };
  std::unordered_map<std::string, ParamNameEnableObject> param_names_;

private:
    void updateMatrix();
    void setParamName();

};

#endif // MATRIX_DISPLAY_HPP_
