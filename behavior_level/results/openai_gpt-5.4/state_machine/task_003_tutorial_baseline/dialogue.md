# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
#!/usr/bin/env python
import rospy
import smach
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_msgs.msg import ColorRGBA

class SimMonitorState(smach.State):
    def __init__(self, model_name):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['time_threshold', 'target_color'])
        self._model_name = model_name
        
        # ROS 1 Service Proxy
        rospy.wait_for_service('/gazebo/set_model_state')
        self._set_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    def execute(self, ud):
        rospy.loginfo("Executing SimMonitorState for model: %s", self._model_name)
        
# TODO: [ROS 2 MIGRATION - NRP CONTEXT]
        # 1. Update the constructor to accept (self, node, model_name) in this EXACT order.
        # 2. Use self.node.get_clock().now() for start_time and current_time.
        # 3. Implement a wait loop until (current_time - start_time) exceeds ud.time_threshold.
        # 4. CRITICAL: Do not wrap the threshold in a Duration object. 
        #    Instead, extract .nanoseconds from the time difference and 
        #    divide by 1e9 to compare with the float 'ud.time_threshold'.
        # 5. Use rclpy.spin_once(self.node, timeout_sec=0.1) inside the loop.
        # 6. Check for SMACH preemption inside the loop using self.preempt_requested().
        # 7. Call '/gazebo/set_model_state' using call_async() and wait for the future.
        #
        # [STYLE CONSTRAINTS]:
        # - NO Python type hints (e.g., use 'node', NOT 'node: Node').
        # - Use 'self.node.get_logger().info()' for all logging.
        # - Manual unit conversion (nanoseconds / 1e9) is REQUIRED for the comparison logic.
        # END OF TODO
        return 'aborted'

----------------------------


# LLM Response

#!/usr/bin/env python3
import rclpy
import smach
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_msgs.msg import ColorRGBA


class SimMonitorState(smach.State):
    def __init__(self, node, model_name):
        smach.State.__init__(self,
                           outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['time_threshold', 'target_color'])
        self.node = node
        self._model_name = model_name

        self._set_state_client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        while not self._set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for /gazebo/set_model_state service...")

    def execute(self, ud):
        self.node.get_logger().info("Executing SimMonitorState for model: %s" % self._model_name)

        start_time = self.node.get_clock().now()
        current_time = start_time

        while ((current_time - start_time).nanoseconds / 1e9) <= ud.time_threshold:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            rclpy.spin_once(self.node, timeout_sec=0.1)
            current_time = self.node.get_clock().now()

        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = self._model_name

        if hasattr(ud, 'target_color') and isinstance(ud.target_color, ColorRGBA):
            pass

        future = self._set_state_client.call_async(request)
        while rclpy.ok() and not future.done():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if not future.done():
            return 'aborted'

        result = future.result()
        if result is not None and getattr(result, 'success', False):
            return 'succeeded'

        return 'aborted'