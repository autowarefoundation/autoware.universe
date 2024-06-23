
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <fstream>

#include <custom_msgs/LidarRawObject.h>
#include <custom_msgs/LidarRawObjectArray.h>

#include <geometry_msgs/Point.h>


using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidardata_saver");
    ros::NodeHandle nh;

    std::string filename;
    //std::string chatter_name;
    ros::param::get("~lidar_data_savepos",filename);
    
    //ros::param::get("~chatter_name",chatter_name);
    std::ofstream fout;
    fout.open(filename,ios::out);

   boost::function<void(const custom_msgs::LidarRawObjectArray::ConstPtr &lidar_msg)> save_func =
            [&fout](const custom_msgs::LidarRawObjectArray::ConstPtr &lidar_msg){

        if (fout.is_open())
        {
          /*  double longitude1 = msg->longitude;
            double latitude1 = msg->latitude;
            float pose = msg->heading;

            fout << setprecision(12)<<longitude1 << " "
                 << setprecision(12)<<latitude1 << "	"
                 << pose << " "<< endl;*/
           std::vector<custom_msgs::LidarRawObject> lidar_object; 
           lidar_object = lidar_msg->objs;
          //custom_msgs::LidarRawObject lidar_object;
           for(int i = 0; i < lidar_object.size(); i++)
           {
                fout << "目标：" << i <<endl;
                for(int j = 0; j < 8; j++)
                {
                    fout << setprecision(12)<<lidar_object[i].bbox_point[j].x<< " "
                    << setprecision(12)<<lidar_object[i].bbox_point[j].y<< " "
                    << setprecision(12)<<lidar_object[i].bbox_point[j].z<< " "
                    << endl;
                }
                fout << setprecision(12)<<lidar_object[i].lwh<< " "
                 << setprecision(12)<<lidar_object[i].x_pos << " "
                 << setprecision(12)<<lidar_object[i].y_pos << " "
                 << endl;
           }
        }
    };
    ros::Subscriber sub = nh.subscribe("detect_topic", 1, save_func);

    ros::spin();
    fout.close();

    return 0;
}
