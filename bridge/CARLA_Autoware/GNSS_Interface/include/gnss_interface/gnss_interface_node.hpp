
#ifndef GNSS_
#define GNSS_

#include <string>
#include <vector>
#include <algorithm>
#include <proj.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <math.h>

class GnssInterface : public rclcpp::Node
{
public:
   explicit GnssInterface(const rclcpp::NodeOptions & node_options);
   virtual ~GnssInterface();
   std::string tf_output_frame_;
  
private:

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gnss_fix;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pup_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pup_pose_with_cov;


  void GnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

};

namespace interface {
	namespace gnss {

class GPSPoint
{
public:
	double x, y, z;
	double lat, lon, alt;
	double dir, a;

	GPSPoint()
	{
		x = y = z = 0;
		lat = lon = alt = 0;
		dir = a = 0;
	}

	GPSPoint(const double& x, const double& y, const double& z, const double& a)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->a = a;

		lat = 0;
		lon = 0;
		alt = 0;
		dir = 0;
	}

	std::string ToString()
	{
		std::stringstream str;
		str.precision(12);
		str << "X:" << x << ", Y:" << y << ", Z:" << z << ", A:" << a << std::endl;
		str << "Lon:" << lon << ", Lat:" << lat << ", Alt:" << alt << ", Dir:" << dir << std::endl;
		return str.str();
	}
};


class WayPoint
{
public:
	GPSPoint	pos;
	WayPoint()
	{
	}

	WayPoint(const double& x, const double& y, const double& z, const double& a)
	{
		pos.x = x;
		pos.y = y;
		pos.z = z;
		pos.a = a;
	}
};

class MappingUtils {
public:
	MappingUtils();
	virtual ~MappingUtils();

	static void llaToxyz(const std::string& proj_str, const WayPoint& origin, const double& lat,
			const double& lon, const double& alt, double& x_out, double& y_out, double& z_out);
};

} 
} 

#endif


