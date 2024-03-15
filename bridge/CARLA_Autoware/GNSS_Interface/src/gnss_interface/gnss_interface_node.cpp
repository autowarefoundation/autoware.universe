#include "gnss_interface/gnss_interface_node.hpp"
#include <proj.h>


namespace interface {
	namespace gnss {

using namespace std;


MappingUtils::MappingUtils() {
}

MappingUtils::~MappingUtils() {
}


void MappingUtils::llaToxyz(const std::string& proj_str, const WayPoint& origin, 
                            const double& lat, const double& lon, const double& alt, 
                            double& x_out, double& y_out, double& z_out) {
    if (proj_str.size() < 8) return;

    PJ_CONTEXT *C = proj_context_create();
    PJ *P = proj_create_crs_to_crs(C, "EPSG:4326", proj_str.c_str(), NULL);

    if (P == 0) return;

    PJ_COORD gps_degrees = proj_coord(lat, lon, alt, 0);
    PJ_COORD xyz_out = proj_trans(P, PJ_FWD, gps_degrees);
    x_out = xyz_out.enu.e + origin.pos.x;
    y_out = xyz_out.enu.n + origin.pos.y;
    z_out = xyz_out.enu.u + origin.pos.z;

    proj_destroy(P);
    proj_context_destroy(C);
}
} 
} 


void GnssInterface::GnssCallBack(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
	geometry_msgs::msg::PoseStamped pose_;
	geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_;
	interface::gnss::WayPoint origin, p;
	interface::gnss::MappingUtils::llaToxyz(
		"+proj=tmerc +lat_0=0 +lon_0=0 +k=0.9999 +x_0=0 +y_0=0 +ellps=GRS80 +units=m +no_defs",
		origin, msg->latitude, msg->longitude, msg->altitude,
		p.pos.x, p.pos.y, p.pos.z);
	pose_.header = msg->header;
	pose_.header.frame_id = "map";
	pose_.pose.position.x = p.pos.x;
	pose_.pose.position.y = p.pos.y;
	pose_.pose.position.z = p.pos.z;

	pose_cov_.header = pose_.header;
	pose_cov_.pose.pose = pose_.pose;

	pup_pose->publish(pose_);
	pup_pose_with_cov->publish(pose_cov_);

}

GnssInterface::~GnssInterface()
{

}

GnssInterface::GnssInterface(const rclcpp::NodeOptions & node_options)
: Node("gnss_interface_node", node_options), tf_output_frame_("base_link")
{
	sub_gnss_fix = this->create_subscription<sensor_msgs::msg::NavSatFix>(
		    "carla/ego_vehicle/gnss", 1,
		    std::bind(&GnssInterface::GnssCallBack, this, std::placeholders::_1));

	pup_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>(
		"/sensing/gnss/pose", 1);
	pup_pose_with_cov = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
		"/sensing/gnss/pose_with_covariance", 1);	  
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(GnssInterface)
