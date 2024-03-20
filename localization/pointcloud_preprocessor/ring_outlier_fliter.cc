// clang++ ring_outlier_fliter.cc -std=c++14 -o ring_outlier_fliter -I/usr/include/eigen3 -I/usr/include/pcl-1.12  -lpcap -ldora_node_api_c -L /home/zxd/dora-rs/dora/target/release -lpthread -ldl -lrt -I /home/zxd/rs_driver/src -I /home/zxd/dora-rs/dora/apis/c -l dora_operator_api_c -L /home/zxd/dora-rs/dora/target/debug

// clang++ ring_outlier_fliter.cc -std=c++14 -o ring_outlier_fliter -I/usr/include/eigen3 -I/usr/include/pcl-1.12  -lpcap -ldora_node_api_c -L /home/zxd/dora-rs/dora/target/release -lpthread -ldl -lrt -I /home/zxd/rs_driver/src -I /home/zxd/dora-rs/dora/apis/c -l dora_operator_api_c -L /home/zxd/dora-rs/dora/target/debug

extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <vector>
#include <cmath>

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <pcl/point_types.h>

static Vec_uint8_t point_data;

namespace pointcloud_filter
{

    template <class T>
    bool float_eq(const T a, const T b, const T eps = 10e-6)
    {
        return std::fabs(a - b) < eps;
    }

    enum class PointIndex
    {
        X,
        Y,
        Z,
        Intensity,
        Ring,
        Azimuth,
        Distance,
        ReturnType,
        TimeStamp
    };

    struct PointXYZI
    {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
        float intensity{0.0F};
        PointXYZI() = default;
        PointXYZI(float x, float y, float z, float intensity) : x(x), y(y), z(z), intensity(intensity) {}
        friend bool operator==(const PointXYZI &p1, const PointXYZI &p2) noexcept
        {
            return float_eq<float>(p1.x, p2.x) && float_eq<float>(p1.y, p2.y) &&
                   float_eq<float>(p1.z, p2.z) && float_eq<float>(p1.intensity, p2.intensity);
        }
    };

    class RingOutlierFilter
    {
    public:
        RingOutlierFilter(double distanceRatio, double objectLengthThreshold, int numPointsThreshold)
            : distanceRatio_(distanceRatio),
              objectLengthThreshold_(objectLengthThreshold),
              numPointsThreshold_(numPointsThreshold) {}

        void filterPointCloud(const std::vector<PointXYZI> &inputCloud, std::vector<PointXYZI> &outputCloud);

    private:
        double distanceRatio_;
        double objectLengthThreshold_;
        int numPointsThreshold_;

        bool isCluster(const std::vector<PointXYZI> &cloud, const std::vector<size_t> &indices);
    };

    void RingOutlierFilter::filterPointCloud(const std::vector<PointXYZI> &inputCloud, std::vector<PointXYZI> &outputCloud)
    {
        outputCloud.clear();
        std::vector<std::vector<size_t>> inputRingMap(128);

        for (size_t idx = 0; idx < inputCloud.size(); ++idx)
        {
            uint16_t ring = static_cast<uint16_t>(inputCloud[idx].intensity);
            inputRingMap[ring].push_back(idx);
        }

        for (const auto &ringIndices : inputRingMap)
        {
            if (ringIndices.size() < 2)
            {

                continue;
            }

            std::vector<size_t> tmpIndices;
            for (size_t idx = 0; idx < ringIndices.size() - 1; ++idx)
            {
                const auto &currentIdx = ringIndices[idx];
                const auto &nextIdx = ringIndices[idx + 1];
                tmpIndices.push_back(currentIdx);

                float azimuthDiff = inputCloud[nextIdx].intensity - inputCloud[currentIdx].intensity;
                azimuthDiff = azimuthDiff < 0.f ? azimuthDiff + 36000.f : azimuthDiff;

                if (std::max(inputCloud[currentIdx].z, inputCloud[nextIdx].z) <
                        std::min(inputCloud[currentIdx].z, inputCloud[nextIdx].z) * distanceRatio_ &&
                    azimuthDiff < 100.f)
                {
                    continue;
                }

                if (isCluster(inputCloud, tmpIndices))
                {
                    for (const auto &tmpIdx : tmpIndices)
                    {
                        outputCloud.push_back(inputCloud[tmpIdx]);
                    }
                }
                tmpIndices.clear();
            }

            if (!tmpIndices.empty())
            {
                if (isCluster(inputCloud, tmpIndices))
                {
                    for (const auto &tmpIdx : tmpIndices)
                    {
                        outputCloud.push_back(inputCloud[tmpIdx]);
                    }
                }
                tmpIndices.clear();
            }
        }
    }

    bool RingOutlierFilter::isCluster(const std::vector<PointXYZI> &cloud, const std::vector<size_t> &indices)
    {
        return indices.size() > numPointsThreshold_;
    }

}

int run(void *dora_context)
{
    for (int i = 0; ; i++)
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

            pointcloud_filter::PointXYZI *points = reinterpret_cast<pointcloud_filter::PointXYZI *>(data + 16);
            std::vector<pointcloud_filter::PointXYZI> inputcloud;
            for (size_t i = 0; i < data_len / 16 - 1; i++)
            {
                // std::cout << "x: " << points[i].x << "y: " << points[i].y << "z: " << points[i].z << "i: " << points[i].intensity << std::endl;
                pointcloud_filter::PointXYZI point_xyzi(points[i].x, points[i].y, points[i].z, points[i].intensity);
                inputcloud.push_back(point_xyzi);
            }

            std::cout << "input cloud size: " << inputcloud.size() << std::endl;

            pointcloud_filter::RingOutlierFilter ringOutlierFilter(25.5, 10.0, 1.0);

            std::vector<pointcloud_filter::PointXYZI> outputcloud;

            ringOutlierFilter.filterPointCloud(inputcloud, outputcloud);

            std::cout << "output size: " << outputcloud.size() << std::endl;
            pointcloud_filter::PointXYZI *points_out;
            points_out = new pointcloud_filter::PointXYZI[outputcloud.size()];
            for (size_t i = 0; i < outputcloud.size(); i++)
            {
                points_out[i].x = outputcloud[i].x;
                points_out[i].y = outputcloud[i].y;
                points_out[i].z = outputcloud[i].z;
                points_out[i].intensity = outputcloud[i].intensity;
                // std::cout << "x: " << points_out[i].x << "y: " << points_out[i].y << "z: " << points_out[i].intensity << std::endl;
            }

            size_t all_size = 16 + outputcloud.size() * 16;
            point_data.ptr = new uint8_t[all_size];

            memcpy(point_data.ptr, data, 16);
            memcpy(point_data.ptr + 16, &points_out[0], outputcloud.size() * 16);

            char *output_data = (char *)point_data.ptr;
            size_t output_data_len = ((outputcloud.size() + 1) * 16);
            std::string out_id = "pointcloud";
            int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
            delete[] point_data.ptr;

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
    std::cout << "ring_outlier_filter node" << std::endl;
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    return ret;
}
