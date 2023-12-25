#ifndef STEERING_WHEEL_DISPLAY_H
#define STEERING_WHEEL_DISPLAY_H
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
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include <QString>
#endif

namespace awf_2d_overlay_vehicle
{

    class SteeringWheelDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        SteeringWheelDisplay();
        virtual ~SteeringWheelDisplay() override;
        void drawSteeringWheel(QPainter &painter, const QRectF &backgroundRect);
        void updateSteeringData(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr &msg);

    private:
        float steering_angle_ = 0.0f;

        QImage wheelImage;
        QImage coloredImage(const QImage &source, const QColor &color);
    };

} // namespace awf_2d_overlay_vehicle

#endif // STEERING_WHEEL_DISPLAY_H
