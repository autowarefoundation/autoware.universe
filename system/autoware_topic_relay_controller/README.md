# topic_relay_controller

## Purpose

The node subscribes to a specified topic, remaps it, and republishes it. Additionally, it has the capability to continue publishing the last received value if the subscription stops.

## Inputs / Outputs

### Input

| Name         | Type                       | Description                                                          |
| ------------ | -------------------------- | -------------------------------------------------------------------- |
| `/<topic>`   | `<specified message type>` | Topic to be subscribed, as defined by the `topic` parameter.         |
| `/tf`        | `tf2_msgs::msg::TFMessage` | (Optional) If the topic is `/tf`, used for transform message relay.  |
| `/tf_static` | `tf2_msgs::msg::TFMessage` | (Optional) If the topic is `/tf_static`, used for static transforms. |

### Output

| Name             | Type                       | Description                                                                   |
| ---------------- | -------------------------- | ----------------------------------------------------------------------------- |
| `/<remap_topic>` | `<specified message type>` | Republished topic after remapping, as defined by the `remap_topic` parameter. |

## Parameters

- **`topic`** (string): The name of the input topic to subscribe to.
- **`remap_topic`** (string): The name of the output topic to publish to.
- **`topic_type`** (string): The type of messages being relayed.
- **`qos`** (integer, default: `1`): QoS profile to use for subscriptions and publications.
- **`transient_local`** (boolean, default: `false`): Enables transient local QoS for subscribers.
- **`best_effort`** (boolean, default: `false`): Enables best-effort QoS for subscribers.
- **`enable_relay_control`** (boolean, default: `true`): Allows dynamic relay control via a service.
- **`srv_name`** (string): The service name for relay control when `enable_relay_control` is `true`.
- **`enable_keep_publishing`** (boolean, default: `false`): Keeps publishing the last received topic value when not subscribed.
- **`update_rate`** (integer, default: `10`): The rate (Hz) for publishing the last topic value when `enable_keep_publishing` is `true`.
- **`frame_id`** (string, optional): Frame ID for transform messages when subscribing to `/tf` or `/tf_static`.
- **`child_frame_id`** (string, optional): Child frame ID for transform messages when subscribing to `/tf` or `/tf_static`.

## Assumptions / Known limits

- The node assumes that the specified `topic` and `remap_topic` are valid and accessible within the ROS 2 environment.
- If `enable_keep_publishing` is `true`, the node continuously republishes the last received value even if no new messages are being received.
- For `/tf` and `/tf_static`, additional parameters like `frame_id` and `child_frame_id` are required for selective transformation relays.
- QoS settings must be carefully chosen to match the requirements of the subscribed and published topics.
