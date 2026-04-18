#!/usr/bin/env python3
import rclpy
import smach
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import ColorRGBA


class SimMonitorState(smach.State):
    def __init__(self, node, model_name):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted', 'aborted'],
            input_keys=['time_threshold', 'target_color']
        )
        self.node = node
        self._model_name = model_name

        self._set_state_client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        while not self._set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /gazebo/set_model_state service...")

    def execute(self, ud):
        self.node.get_logger().info("Executing SimMonitorState for model: %s" % self._model_name)

        start_time = self.node.get_clock().now()
        current_time = self.node.get_clock().now()

        while ((current_time - start_time).nanoseconds / 1e9) <= ud.time_threshold:
            if self.preempt_requested():
                self.service_preempt()
                self.node.get_logger().info("Preempt requested in SimMonitorState.")
                return 'preempted'
            rclpy.spin_once(self.node, timeout_sec=0.1)
            current_time = self.node.get_clock().now()

        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = self._model_name

        future = self._set_state_client.call_async(request)
        while rclpy.ok() and not future.done():
            if self.preempt_requested():
                self.service_preempt()
                self.node.get_logger().info("Preempt requested while waiting for service response.")
                return 'preempted'
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not future.done() or future.result() is None:
            self.node.get_logger().info("Failed to receive response from /gazebo/set_model_state.")
            return 'aborted'

        response = future.result()
        if response.success:
            self.node.get_logger().info("Successfully updated model state.")
            return 'succeeded'

        self.node.get_logger().info("Service call failed: %s" % response.status_message)
        return 'aborted'