#include "dora-node-api.h"
#include "dora-ros2-bindings.h"

#include <iostream>
#include <vector>
#include <random>

int main()
{
    std::cout << "HELLO dora_node_sub_rosmsg  C++" << std::endl;

    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto qos = qos_default();
    qos.durability = Ros2Durability::Volatile;
    qos.liveliness = Ros2Liveliness::Automatic;
    qos.reliable = true;
    qos.max_blocking_time = 0.1;

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/ros2_demo", "turtle_teleop_sub");
    auto vel_topic = node->create_topic_geometry_msgs_Twist("/turtle1", "cmd_vel", qos);
    auto pose_subscription = node->create_subscription(vel_topic, qos, merged_events);

    std::random_device dev;
    std::default_random_engine gen(dev());
    std::uniform_real_distribution<> dist(0., 1.);
 
    auto received_ticks = 0;
    auto responses_received = 0;

    while(1)
    {
        auto event = merged_events.next();

        if (event.is_dora())
        {
            auto dora_event = downcast_dora(std::move(event));

            auto ty = event_type(dora_event);

            if (ty == DoraEventType::AllInputsClosed)
            {
                break;
            }
            else if (ty == DoraEventType::Input)
            {
                auto input = event_as_input(std::move(dora_event));
                received_ticks += 1;

                std::cout << "Received input " << std::string(input.id) << std::endl;
 
            }
            else
            {
                std::cerr << "Unknown event type " << static_cast<int>(ty) << std::endl;
            }

            if (received_ticks > 20)
            {
                break;
            }
        }
        else if (pose_subscription->matches(event))
        {
            auto cmd = pose_subscription->downcast(std::move(event));
            std::cout << "Received cmd x:" << cmd.linear.x << ", y:" << cmd.linear.y <<", z:"<< cmd.angular.z << std::endl;
        }
        else
        {
            std::cout << "received unexpected event" << std::endl;
        }
    }

    std::cout << "Received " << responses_received << " service responses" << std::endl;
    assert(responses_received > 0);

    // try to access a constant for testing
    assert((sensor_msgs::const_NavSatStatus_STATUS_NO_FIX() == -1));

    std::cout << "GOODBYE FROM C++ node (using Rust API)" << std::endl;

    return 0;
}
