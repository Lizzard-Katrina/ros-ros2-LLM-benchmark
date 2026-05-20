#!/usr/bin/env python
import rclpy
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
        
        self._set_state_client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        while not self._set_state_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for /gazebo/set_model_state service...')

    def execute(self, ud):
        self.node.get_logger().info("Executing SimMonitorState for model: %s" % self._model_name)
        
        start_time = self.node.get_clock().now()
        
        while rclpy.ok():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
                
            current_time = self.node.get_clock().now()
            time_diff = current_time - start_time
            
            if (time_diff.nanoseconds / 1e9) > ud.time_threshold:
                break
                
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        req = SetModelState.Request()
        req.model_state.model_name = self._model_name
        
        future = self._set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            self.node.get_logger().info("Successfully set model state.")
            return 'succeeded'
            
        return 'aborted'