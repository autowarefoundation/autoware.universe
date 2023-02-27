# Timing violation monitor framework

A lightweight framework for timing violation detection

## Brief Description

This package serves the framework to detect timing violations. The timing violation detection framework measures the response time on selected paths and checks if it is less than expected. Here, a path means a series processing across nodes.

The following figure illustrates what the framework measures. The response time is the elapsed time from the start of `Node S` to the end of `Node E`, represented by the blue dotted lines `<---->`.

![Response time](./docs/images/response_time.png "Response time")

## Design

### Design overview

The initial design of the timing violation framework is intended to be deployed as a small start, and is designed with the following three points.

- Minimize changes to Autoware user code as much as possible
- Have the flexibility to add or delete paths
- Run in a different Linux process than the monitored entity in order to keep the monitored entity stable
  - Avoid delays in the execution of the monitored entity

To fulfill the policy, the framework utilizes existing timestamp in header of topic messages.

![Basic idea](./docs/images/timing_violation_detection_basic_design.png)

The framework assumes that the timestamp, represented `ts: t0` in the figure, is not changed while it goes through the path.

The timing violation monitor receives the topic message, `/topic_e`, sent from the end node, `Node E`. The monitor check occurrence of timing violation with the topic message.

Autoware has several paths that do not change timestamps, such as Sensing, Perception, or Localization. If you apply the framework to such paths, you only need to add the timing violation monitor and don't need to change any user code.

### Limitation

As mentioned above, the framework assumes that paths does not change timestamps while the timestamp goes through the path. If the value of timestamp is replaced by another in the path, the framework cannot be applied to the path.

That limitation allow users to apply the framework to not all paths. If you want to apply any paths, [TILDE](https://github.com/tier4/TILDE) might be a good candidate.

### Other materials

The fundamental of the timing violation framework is described in [the page](./docs/design_timing_violation_detection.md). That page describes the requirements and high-level design policy of the timing violation framework.

[The other page](./docs/internal_design.md) shows the design which is the basis of the implemented timing violation monitor. This might be helpful if you want to apply the framework to another node.

## Usage

If you want to monitor a new path with the framework, you need to execute the following steps.

1. Define the path and its corresponding topic
2. Add `MessageTrackingNotifier` to the end of path if necessary
3. Add new path definition to the configuration file

### Define the path

The first step is to decide which path to monitor. As mentioned in [limitation](#limitation) section, you have to choose the path where the input timestamp is not changed at all.

In the step, you have to check if the framework is applied to target path.

### Add `MessageTrackingNotifier` to the end of path

You can skip this step if the end node publishes message topic and this topic is specified as the end of the path. If so, skip this step and specify the information for the last message topic to be output in the configuration file in the next step.

The second step is to add `MessageTrackingNotifier` which is mentioned as the add-on in [the design page](./docs/internal_design.md). `MessageTrackingNotifier` notifies when the topic data is consumed. It is needed if you apply the framework to the path which does not publish any message topic.

If you want to add `MessageTrackingNotifier` to a path, you need to change files as below.

1. Add `timing_violation_monitor` to your package.xml

   ```xml
     <depend>timing_violation_monitor</depend>
   ```

2. Change the header file of end node as below

   ```cpp
   // Statements which include header files.
   #include <rclcpp/rclcpp.hpp>
   #include <timing_violation_monitor_utils/message_consumption_notifier.hpp> // *** Add this statements ***/

   class EndNode : public rclcpp::Node {

     public:
       EndNode(); // constructor

     private:
       // callback
       void main_method();
       void receive_method();
       //...

       // subscribers and publishers
       rclcpp::Subscription<PointCloud2>::SharedPtr sub_;
       rclcpp::Publisher<PointCloud2>::SharedPtr pub_;

       std::unique_ptr<timing_violation_monitor_utils::MessageConsumptionNotifier> notifier_; // *** Add this statements ***/
   };
   ```

3. Add notifier execution on the end node

   ```cpp
   // Constructor.
   EndNode::EndNode() {
       sub_ = this->create_subscription<PointCloud2>(...);
       pub_ = this->create_publisher<PointCloud2>(...);

       notifier_ = std::make_unique<timing_violation_monitor_utils::MessageConsumptionNotifier>(this, "notifier_topic_message_name", 10); // *** Add this statements ***/
   }
   // Other statements ....

   // Definition of main_method().
   void EndNode::main_method() {
       // user code.
       // ...

       // when target topic is consumed.
       consume_message(message);
       notifier->notify(message.header.stamp);  // *** Add this statements ***/

       // user code.
       // ...
   }
   ```

### Add new path definition to the configuration file

The timing violation monitor has a configuration file to know which path and topic to monitor. In this step, users have to write configuration file.

The sample configuration file is shown as below. If you add a new path, append the items in `target_paths`.

```yaml
ros__parameters:
  diag_period_sec: 5.0 # frequency of report
  target_paths:
    ekf-to-ndt: # path name. Can be set to any name.
      topic: /localization/pose_estimator/for_tilde_interpolator_mtt # topic name published by MessageConsumptionNotifier.
      message_type: tilde_msg/msg/MessageTrackingTag # message type
      severity: warn # severity
      period: 100.0 # execution frequency of path
      deadline: 200.0 # deadline of response time
      violation_count_threshold: 5 # threshold to judge warn or not.

    pointcloudPreprocessor-to-ndt: # path name
      topic: /localization/pose_estimator/pose_with_covariance # topic name
      message_type: geometry_msgs/msg/PoseWithCovarianceStamped # message type
      severity: error # severity
      period: 100.0 # execution frequency of path
      deadline: 150.0 # deadline of response time
      violation_count_threshold: 1 # threshold to judge error or not.
```

In this sample, path `ekf-to-ndt` uses MessageConsumptionNotifier to notify when the topic data is consumed. On the other hand, path `pointcloudPreprocessor-to-ndt` need no code changes.

## Output Message

The timing violation monitor transmits the topic message whose name is `/diagnostics`. `/diagnostics` is the common topic message served by ROS 2 [`diagnostic_updater`](https://github.com/ros/diagnostics). The format of `/diagnostics` is defined by `diagnostic_updater` also.

<!-- prettier-ignore-start -->

!!! Note
What this section describes is tentative.

<!-- prettier-ignore-start -->
