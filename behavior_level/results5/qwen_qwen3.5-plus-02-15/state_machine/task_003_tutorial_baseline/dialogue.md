# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: code.py
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

[FILENAME: code.py]
#!/usr/bin/env python3
import rclpy
import smach
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import ColorRGBA
from rclpy.duration import Duration

class SimMonitorState(smach.State):
    def __init__(self, node, model_name):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'preempted', 'aborted'],
                           input_keys=['time_threshold', 'target_color'])
        self.node = node
        self._model_name = model_name
        
        # ROS 2 Service Client
        self._set_state_client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        while not self._set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for service /gazebo/set_model_state...')

    def execute(self, ud):
        self.node.get_logger().info("Executing SimMonitorState for model: %s", self._model_name)
        
        start_time = self.node.get_clock().now()
        
        while True:
            current_time = self.node.get_clock().now()
            time_diff = current_time - start_time
            
            # Manual unit conversion: nanoseconds / 1e9 to get seconds as float
            elapsed_seconds = time_diff.nanoseconds / 1e9
            
            if elapsed_seconds >= ud.time_threshold:
                break
            
            if self.preempt_requested():
                self.node.get_logger().info("Preempt requested in SimMonitorState")
                return 'preempted'
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Call service asynchronously
        req = SetModelState.Request()
        req.model_name = self._model_name
        # Example: Reset pose or maintain state, adapting to specific needs
        req.pose.position.x = 0.0
        req.pose.position.y = 0.0
        req.pose.position.z = 0.0
        req.pose.orientation.w = 1.0
        
        future = self._set_state_client.call_async(req)
        
        # Wait for the future with a small timeout loop to allow preemption checks if needed
        # For simplicity in this migration context, we spin until done or timeout
        timeout = 5.0 # seconds
        start_wait = self.node.get_clock().now()
        while rclpy.ok() and not future.done():
            now = self.node.get_clock().now()
            if (now - start_wait).nanoseconds / 1e9 > timeout:
                self.node.get_logger().error("Service call timed out")
                return 'aborted'
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.node.get_logger().info("SetModelState succeeded")
                    return 'succeeded'
                else:
                    self.node.get_logger().error("SetModelState failed: %s", response.status_message)
                    return 'aborted'
            except Exception as e:
                self.node.get_logger().error("Service call failed: %s", str(e))
                return 'aborted'
        
        return 'aborted'