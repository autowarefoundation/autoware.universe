#include "SteeringWheelDisplay.h"

#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTexture.h>
#include <OgreTechnique.h>
#include <OgreHardwarePixelBuffer.h>
#include <rviz_rendering/render_system.hpp>
#include <QPainter>
#include <QFontDatabase>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <memory>
#include <string>

namespace awf_2d_overlay_vehicle
{

    SteeringWheelDisplay::SteeringWheelDisplay()
    {

        int fontId = QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Regular.ttf");
        int fontId2 = QFontDatabase::addApplicationFont(":/assets/font/Quicksand/static/Quicksand-Bold.ttf");
        if (fontId == -1 || fontId2 == -1)
        {
            std::cout << "Failed to load the Quicksand font.";
        }

        // Load the wheel image
        wheelImage.load(":/assets/images/wheel.png");
    }

    SteeringWheelDisplay::~SteeringWheelDisplay()
    {
        // Cleanup if necessary
    }

    void SteeringWheelDisplay::updateSteeringData(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr &msg)
    {
        try
        {
            // Assuming msg->steering_angle is the field you're interested in
            float steeringAngle = msg->steering_tire_angle;
            // we received it as a radian value, but we want to display it in degrees
            steering_angle_ = (steeringAngle * -180 / M_PI) * 17; // 17 is the ratio between the steering wheel and the steering tire angle i assume

            queueRender();
        }
        catch (const std::exception &e)
        {
            // Log the error
            std::cerr << "Error in processMessage: " << e.what() << std::endl;
        }
    }

    void SteeringWheelDisplay::drawSteeringWheel(QPainter &painter, const QRectF &backgroundRect)
    {

        int wheelCenterX = backgroundRect.width() - wheelImage.width() - 20;       // Adjust X position
        int wheelCenterY = backgroundRect.height() / 2 + wheelImage.height() - 10; // Center Y position

        // Enable Antialiasing for smoother drawing
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

        // Draw the wheel image
        QImage smallerImage = wheelImage.scaled(54, 54, Qt::KeepAspectRatio, Qt::SmoothTransformation);

        QImage wheel = coloredImage(smallerImage, QColor(255, 255, 255, 255)); // White overlay

        // Rotate the wheel according to the steering angle
        qreal steeringAngle = steering_angle_;
        // Use QMatrix for rotation
        QMatrix rotation_matrix;
        rotation_matrix.rotate(
            std::round(
                steeringAngle));

        // Transform the wheel image
        QImage rotatedWheel = wheel.transformed(QTransform(rotation_matrix), Qt::SmoothTransformation);

        // Crop the rotated image to its original size
        rotatedWheel = rotatedWheel.copy(
            (rotatedWheel.width() - wheel.width()) / 2,
            (rotatedWheel.height() - wheel.height()) / 2,
            wheel.width(),
            wheel.height());

        // Draw the rotated image at the correct position
        painter.drawImage(QPointF(wheelCenterX - wheelImage.width() / 2, wheelCenterY - wheelImage.height() / 2), rotatedWheel);

        QString steeringAngleStringAfterModulo = QString::number(fmod(steeringAngle, 360), 'f', 0);

        // Draw the steering angle text
        QFont steeringFont("Quicksand", 9, QFont::Bold);
        painter.setFont(steeringFont);
        painter.setPen(QColor(0, 0, 0, 255));
        QRect steeringRect(wheelCenterX - wheelImage.width() / 2 - 2, wheelCenterY - wheelImage.height() / 2 - 2, wheelImage.width(), wheelImage.height());
        painter.drawText(steeringRect, Qt::AlignCenter, steeringAngleStringAfterModulo + "°");
    }

    QImage SteeringWheelDisplay::coloredImage(const QImage &source, const QColor &color)
    {
        QImage result = source;
        QPainter p(&result);
        p.setCompositionMode(QPainter::CompositionMode_SourceAtop);
        p.fillRect(result.rect(), color);
        p.end();
        return result;
    }

} // namespace awf_2d_overlay_vehicle

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(awf_2d_overlay_vehicle::SteeringWheelDisplay, rviz_common::Display)
