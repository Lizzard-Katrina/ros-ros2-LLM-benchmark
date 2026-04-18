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

```python
#!/usr/bin/env python3
import rclpy
import smach
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_msgs.msg import ColorRGBA

class SimMonitorState(smach.State):
    def __init__(self, node, model_name):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['time_threshold', 'target_color'])
        self._model_name = model_name
        self.node = node
        
        # ROS 2 Service Client
        self._set_state_client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        while not self._set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /gazebo/set_model_state service...')

    def execute(self, ud):
        self.node.get_logger().info(f"Executing SimMonitorState for model: {self._model_name}")
        
        start_time = self.node.get_clock().now()
        
        while True:
            current_time = self.node.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            
            if elapsed_time >= ud.time_threshold:
                break
                
            if self.preempt_requested():
                return 'preempted'
                
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Create and send service request
        request = SetModelStateRequest()
        request.model_state.model_name = self._model_name
        
        future = self._set_state_client.call_async(request)
        
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        return 'succeeded'
                    else:
                        self.node.get_logger().error('Failed to set model state')
                        return 'aborted'
                except Exception as e:
                    self.node.get_logger().error(f'Service call failed: {e}')
                    return 'aborted'
                    
            if self.preempt_requested():
                return 'preempted'
                
        return 'aborted'
```