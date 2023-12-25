#ifndef SPEEDLIMIT_DISPLAY_H
#define SPEEDLIMIT_DISPLAY_H
#ifndef Q_MOC_RUN
#include <rviz_common/ros_topic_display.hpp>
#include "overlay_utils.hpp"
#include <OgreColourValue.h>
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <QImage>
#include <QString>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#endif

namespace awf_2d_overlay_vehicle
{

    class SpeedLimitDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        SpeedLimitDisplay();
        virtual ~SpeedLimitDisplay() override;
        void drawSpeedLimitIndicator(QPainter &painter, const QRectF &backgroundRect);
        void updateSpeedLimitData(const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);

    private:
        float current_limit; // Internal variable to store current gear
    };

} // namespace awf_2d_overlay_vehicle

#endif // SPEEDLIMIT_DISPLAY_H
