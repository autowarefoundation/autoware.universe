import gc
import glob
import signal
from subprocess import DEVNULL
from subprocess import Popen
import sys
import threading

from perception_benchmark_tool.benchmark_tools.datasets_ros_nodes.waymo_ros_node import WaymoRosNode
import rclpy


def main(args=None):
    tf_records = glob.glob(sys.argv[1] + "/*.tfrecord")
    launch_file_path = sys.argv[2]

    for tf_record in tf_records:

        rclpy.init(args=args)
        node = WaymoRosNode(tf_record)
        try:
            launch_process = Popen(["ros2", "launch", launch_file_path], text=False, stdout=DEVNULL)

        except Exception as ex:
            print("Parent program has exited with the below error:\n{0}".format(ex))
            if launch_process:
                launch_process.send_signal(signal.SIGINT)
                launch_process.wait(timeout=30)

        thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        thread.start()

        rate = node.create_rate(10, node.get_clock())

        while rclpy.ok() and not node.is_dataset_finished():
            if node.check_subscriber_ready() and not node.scene_processed():
                node.publish_scene()
                rate.sleep()

        launch_process.send_signal(signal.SIGINT)
        launch_process.wait(timeout=30)

        node.destroy_node()
        rclpy.shutdown()
        thread.join()

        gc.collect()

        print("Scene processed : " + tf_record)


if __name__ == "__main__":
    main()
