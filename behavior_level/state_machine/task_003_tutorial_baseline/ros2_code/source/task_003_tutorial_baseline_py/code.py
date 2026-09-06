#!/usr/bin/env python
import rclpy
from task_003_tutorial_baseline_py import smach_lite as smach
from task_003_tutorial_baseline.srv import SetModelState
from std_msgs.msg import ColorRGBA


class SimMonitorState(smach.State):
    def __init__(self, node, model_name):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'preempted', 'aborted'],
                             input_keys=['time_threshold', 'target_color'])
        self.node = node
        self._model_name = model_name

        # ROS 2 Service Client
        self._set_state_client = self.node.create_client(
            SetModelState, '/gazebo/set_model_state')
        while not self._set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /gazebo/set_model_state service...')

    def execute(self, ud):
        self.node.get_logger().info("Executing SimMonitorState for model: %s" % self._model_name)

        start_time = self.node.get_clock().now()

        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            current_time = self.node.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9

            if elapsed >= ud.time_threshold:
                break

        # Build the service request
        request = SetModelState.Request()
        request.model_state.model_name = self._model_name

        self.node.get_logger().info("Threshold reached. Calling set_model_state service.")

        future = self._set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        result = future.result()
        if result is not None and result.success:
            self.node.get_logger().info("Service call succeeded for model: %s" % self._model_name)
            return 'succeeded'
        else:
            self.node.get_logger().info("Service call failed for model: %s" % self._model_name)
            return 'aborted'


def main():
    rclpy.init()
    node = rclpy.create_node('sim_monitor_node')
    state = SimMonitorState(node, 'test_model')
    node.get_logger().info("SimMonitorState node created.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()