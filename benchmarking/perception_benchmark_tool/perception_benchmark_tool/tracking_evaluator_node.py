from autoware_auto_perception_msgs.msg import TrackedObjects
import rclpy
from rclpy.node import Node


class TrackingEvaluator(Node):
    def __init__(self):
        super().__init__("tracking_evaluation_node")

        self.sub_tracking = self.create_subscription(
            TrackedObjects,
            "/perception/object_recognition/tracking/objects",
            self.tracked_objects_callback,
            10,
        )

    def tracked_objects_callback(self, tracked_objects):
        print("Tracked objects coming.")


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = TrackingEvaluator()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
