# 基本整合測試（不進 CI），驗證 ros2_control_node 能起、spawner 能啟動 controller
import os
import pytest
import launch
import launch_ros
import launch_testing
import rclpy
from rclpy.node import Node

def generate_test_description():
    bringup = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(os.path.dirname(__file__), "..", "config", "ros2_control_mock.yaml")],
        output="screen",
    )
    spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["adaptive_controller"],
        output="screen",
    )
    return launch.LaunchDescription([bringup, spawner, launch_testing.actions.ReadyToTest()]), {"bringup": bringup}

@pytest.mark.launch_test
def test_topics_exist(launch_service, bringup, proc_output):
    rclpy.init()
    node = Node("probe")
    try:
      # 等待 topic 出現
      topics = ["/controller_manager/adaptive_controller/debug",
                "/controller_manager/adaptive_controller/reference"]
      ok = False
      for _ in range(100):
          rclpy.spin_once(node, timeout_sec=0.1)
          avail = [t for t,_ in node.get_topic_names_and_types()]
          if all(t in avail for t in topics):
              ok = True; break
      assert ok, f"required topics missing; got={node.get_topic_names_and_types()}"
    finally:
      node.destroy_node()
      rclpy.shutdown()
