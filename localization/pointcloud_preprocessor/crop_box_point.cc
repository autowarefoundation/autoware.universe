extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <pcl/point_types.h>

struct PointT
{
    float x;
    float y;
    float z;
    float i;
};

static Vec_uint8_t point_data;

namespace pointcloud_preprocessor
{
    struct CropBoxParam
    {
        float min_x, min_y, min_z, max_x, max_y, max_z;
        bool negative;
    };

    class CropBoxFilter
    {
    public:
        CropBoxFilter(const CropBoxParam &param) : param_(param) {}

        void filter(const std::vector<Eigen::Vector4f> &input, std::vector<Eigen::Vector4f> &output);
        void publishCropBoxPolygon();

    private:
        CropBoxParam param_;
    };
}

void pointcloud_preprocessor::CropBoxFilter::filter(
    const std::vector<Eigen::Vector4f> &input, std::vector<Eigen::Vector4f> &output)
{
    output.clear();
    for (const auto &pt : input)
    {
        if ((!param_.negative &&
             param_.min_z < pt.z() && pt.z() < param_.max_z &&
             param_.min_y < pt.y() && pt.y() < param_.max_y &&
             param_.min_x < pt.x() && pt.x() < param_.max_x) ||
            (param_.negative &&
             (param_.min_z > pt.z() || pt.z() > param_.max_z ||
              param_.min_y > pt.y() || pt.y() > param_.max_y ||
              param_.min_x > pt.x() || pt.x() > param_.max_x)))
        {
            output.push_back(pt);
        }
    }
    publishCropBoxPolygon();
}

void pointcloud_preprocessor::CropBoxFilter::publishCropBoxPolygon()
{
    std::vector<Eigen::Vector3f> polygon_points;

    auto generatePoint = [&](double x, double y, double z)
    {
        return Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    };
    const double x1 = param_.max_x;
    const double x2 = param_.min_x;
    const double x3 = param_.min_x;
    const double x4 = param_.max_x;

    const double y1 = param_.max_y;
    const double y2 = param_.max_y;
    const double y3 = param_.min_y;
    const double y4 = param_.min_y;

    const double z1 = param_.min_z;
    const double z2 = param_.max_z;

    polygon_points.push_back(generatePoint(x1, y1, z1));
    polygon_points.push_back(generatePoint(x2, y2, z1));
    polygon_points.push_back(generatePoint(x3, y3, z1));
    polygon_points.push_back(generatePoint(x4, y4, z1));
    polygon_points.push_back(generatePoint(x1, y1, z1));

    polygon_points.push_back(generatePoint(x1, y1, z2));

    polygon_points.push_back(generatePoint(x2, y2, z2));
    polygon_points.push_back(generatePoint(x2, y2, z1));
    polygon_points.push_back(generatePoint(x2, y2, z2));

    polygon_points.push_back(generatePoint(x3, y3, z2));
    polygon_points.push_back(generatePoint(x3, y3, z1));
    polygon_points.push_back(generatePoint(x3, y3, z2));

    polygon_points.push_back(generatePoint(x4, y4, z2));
    polygon_points.push_back(generatePoint(x4, y4, z1));
    polygon_points.push_back(generatePoint(x4, y4, z2));

    polygon_points.push_back(generatePoint(x1, y1, z2));
}

int run(void *dora_context)
{

    while (true)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            char *data;
            size_t data_len;
            read_dora_input_data(event, &data, &data_len);
            std::cout << "lenth : " << data_len << std::endl;
            size_t point_mum = data_len / 16 - 1;
            std::cout << "point_mum : " << point_mum << std::endl;

            PointT *points = reinterpret_cast<PointT *>(data + 16);
            std::vector<Eigen::Vector4f> input_cloud;
            for (size_t i = 0; i < (data_len / 16 - 1); i++)
            {
                // std::cout << " x: " <<points[i].x << " y: " << points[i].y << " z: " << points[i].z << " i: " << points[i].i << std::endl;
                Eigen::Vector4f point_xyzi(points[i].x, points[i].y, points[i].z, points[i].i);
                input_cloud.push_back(point_xyzi);
            }
            // {
            //     Eigen ::Vector4f point_xyzi;
            //     point_xyzi.x() = *(float*) (data + 16 + 16 * i);
            //     point_xyzi.y() = *(float*) (data + 16 + 4 + 16 * i);
            //     point_xyzi.z() = *(float*) (data + 16 + 8 + 16 * i);
            //     point_xyzi.w() = *(float*) (data + 16 + 12 + 16 * i);
            //     std::cout << " x: " <<point_xyzi.x() << " y: " << point_xyzi.y() << " z: " << point_xyzi.z() << " i: " << point_xyzi.w() << std::endl;
            //     input_cloud.push_back(point_xyzi);
            // }
            std::cout << "input cloud size: " << input_cloud.size() << std::endl;

            std::vector<Eigen::Vector4f> output_cloud;

            pointcloud_preprocessor::CropBoxParam param;
            param.min_x = -100.0;
            param.min_y = -100.0;
            param.min_z = -100.0;
            param.max_x = 100.0;
            param.max_y = 100.0;
            param.max_z = 100.0;
            param.negative = false;

            pointcloud_preprocessor::CropBoxFilter filter(param);
            filter.filter(input_cloud, output_cloud);

            // std::cout << "x: " << output_cloud[1].x() << std::endl;
            // std::cout << "size: " << output_cloud.size() << std::endl;

            PointT *points_out;
            points_out = new PointT[output_cloud.size()];
            for (size_t i = 0; i < output_cloud.size(); i++)
            {
                points_out[i].x = output_cloud[i].x();
                points_out[i].y = output_cloud[i].y();
                points_out[i].z = output_cloud[i].z();
                points_out[i].i = output_cloud[i].w();
                // std::cout << "x: " << point[i].x << ", y: " << point[i].y << ", z: " << point[i].z << ", i: " << point[i].i << std::endl;
            }

            std::cout << "output cloud size: " << output_cloud.size() << std::endl;

            size_t all_size = 16 + output_cloud.size() * 16;
            point_data.ptr = new uint8_t[all_size];

            memcpy(point_data.ptr, data, 16);
            memcpy(point_data.ptr + 16, &points_out[0], output_cloud.size() * 16);


            char *output_data = (char *)point_data.ptr;
            size_t output_data_len = ((output_cloud.size() + 1) * 16);
            std::string out_id = "pointcloud";
            std::cout << "output_data_len: " << output_data_len << std::endl;
            int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
            delete[] point_data.ptr;
            // int resultend = 0;

            if (resultend != 0)
            {
                std::cerr << "failed to send output" << std::endl;
                return 1;
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "crop_box_point node" << std::endl;
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    return ret;
}
