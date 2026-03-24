import rclpy
from rclpy.duration import Duration
from rclpy.task import Future
import smach
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import ColorRGBA

class SimMonitorState(smach.State):
    def __init__(self, node, model_name):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['time_threshold', 'target_color'])
        self.node = node
        self._model_name = model_name

        self._client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for service /gazebo/set_model_state...')

    def execute(self, ud):
        self.node.get_logger().info("Executing SimMonitorState for model: %s" % self._model_name)

        start_time = self.node.get_clock().now()
        while True:
            current_time = self.node.get_clock().now()
            elapsed_sec = (current_time - start_time).nanoseconds / 1e9
            if elapsed_sec > ud.time_threshold:
                break

            rclpy.spin_once(self.node, timeout_sec=0.1)

            if self.preempt_requested():
                self.node.get_logger().info('Preempt requested, preempting...')
                self.service_preempt()
                return 'preempted'

        req = SetModelState.Request()
        req.model_state.model_name = self._model_name
        # Optionally set other fields of model_state here if needed

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info('SetModelState service call succeeded')
            return 'succeeded'
        else:
            self.node.get_logger().error('SetModelState service call failed')
            return 'aborted'