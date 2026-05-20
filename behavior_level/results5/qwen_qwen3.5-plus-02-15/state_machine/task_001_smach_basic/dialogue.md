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

FILE_PATH: service_state.py
----------------------------
#!/usr/bin/env python3
import rclpy

import threading

import smach
from .ros_state import RosState

__all__ = ['ServiceState']

class ServiceState(RosState):
    """State for calling a service."""
    def __init__(self,
            node,
            # Service info
            service_name,
            service_spec,
            # Request Policy
            request = None,
            request_cb = None,
            request_cb_args = [],
            request_cb_kwargs = {},
            request_key = None,
            request_slots = [],
            # Response Policy
            response_cb = None,
            response_cb_args = [],
            response_cb_kwargs = {},
            response_key = None,
            response_slots = [],
            # Keys
            input_keys = [],
            output_keys = [],
            outcomes = [],
            ):

        RosState.__init__(self, node, outcomes=['succeeded', 'aborted', 'preempted'])

        # Store Service info
        self._service_name = service_name
        self._service_spec = service_spec

        self._proxy = None

        # Store request policy
        if request is None:
            self._request = service_spec.Request()
        else:
            self._request = request


        if request_cb is not None and not hasattr(request_cb, '__call__'):
            raise smach.InvalidStateError("Request callback object given to ServiceState that IS NOT a function object")

        self._request_cb = request_cb
        self._request_cb_args = request_cb_args
        self._request_cb_kwargs = request_cb_kwargs
        if smach.has_smach_interface(request_cb):
            self._request_cb_input_keys = request_cb.get_registered_input_keys()
            self._request_cb_output_keys = request_cb.get_registered_output_keys()

            self.register_input_keys(self._request_cb_input_keys)
            self.register_output_keys(self._request_cb_output_keys)
        else:
            self._request_cb_input_keys = input_keys
            self._request_cb_output_keys = output_keys

        self._request_key = request_key
        if request_key is not None:
            self.register_input_keys([request_key])

        self._request_slots = request_slots
        self.register_input_keys(request_slots)

        # Store response policy
        if response_cb is not None and not hasattr(response_cb, '__call__'):
            raise smach.InvalidStateError("Response callback object given to ServiceState that IS NOT a function object")

        self._response_cb = response_cb
        self._response_cb_args = response_cb_args
        self._response_cb_kwargs = response_cb_kwargs
        if smach.has_smach_interface(response_cb):
            self._response_cb_input_keys = response_cb.get_registered_input_keys()
            self._response_cb_output_keys = response_cb.get_registered_output_keys()
            self._response_cb_outcomes = response_cb.get_registered_outcomes()

            self.register_input_keys(self._response_cb_input_keys)
            self.register_output_keys(self._response_cb_output_keys)
            self.register_outcomes(self._response_cb_outcomes)
        else:
            self._response_cb_input_keys = input_keys
            self._response_cb_output_keys = output_keys
            self._response_cb_outcomes = outcomes

        # Register additional input and output keys
        self.register_input_keys(input_keys)
        self.register_output_keys(output_keys)
        self.register_outcomes(outcomes)

        self._response_key = response_key
        if response_key is not None:
            self.register_output_keys([response_key])

        self._response_slots = response_slots
        self.register_output_keys(response_slots)

        self._proxy = self.node.create_client(self._service_spec, self._service_name)

    def execute(self, ud):
        """Execute service"""
        # Check for preemption before executing
        if self.preempt_requested():
            self.node.get_logger().info("Preempting %s before sending request." % self._service_name)
            self.service_preempt()
            return 'preempted'

        # Make sure we're connected to the service
        try:
        # TODO: [ROS2 MIGRATION TASK]
# 1. Wait for the service to be ready using self._proxy (handle preemption and rclpy shutdown).
# 2. Prepare the service request using self._request, self._request_key, and self._request_slots.
# 3. Call the user-supplied request_cb if it exists to update the request.
# 4. Perform a SYNCHRONOUS-LIKE service call compatible with SMACH's execute loop.
# 5. Handle the response using response_cb, response_key, and response_slots.
# NOTE: Use self.node.get_logger() for logging. Ensure the call doesn't deadlock the executor.
        #end of TODO
        if self._response_cb is not None:
            try:
                response_cb_outcome = self._response_cb(
                        smach.Remapper(
                                ud,
                                self._response_cb_input_keys,
                                self._response_cb_output_keys,
                                []),
                        self._response,
                        *self._response_cb_args,
                        **self._response_cb_kwargs)
                if response_cb_outcome is not None and response_cb_outcome not in self.get_registered_outcomes():
                    self.node.get_logger().error("Result callback for service "+self._service_name+", "+str(self._response_cb)+" was not registered with the response_cb_outcomes argument. The response callback returned '"+str(response_cb_outcome)+"' but the only registered outcomes are: "+str(self.get_registered_outcomes()))
                    return 'aborted'
            except:
                self.node.get_logger().error("Could not execute response callback: "+traceback.format_exc())
                return 'aborted'

        if self._response_key is not None:
            ud[self._response_key] = self._response

        for key in self._response_slots:
            ud[key] = getattr(self._response,key)

        if response_cb_outcome is not None:
            return response_cb_outcome

        return 'succeeded'

----------------------------


# LLM Response

[FILENAME: service_state.py]
#!/usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor

import threading
import traceback

import smach
from .ros_state import RosState

__all__ = ['ServiceState']

class ServiceState(RosState):
    """State for calling a service."""
    def __init__(self,
            node,
            # Service info
            service_name,
            service_spec,
            # Request Policy
            request = None,
            request_cb = None,
            request_cb_args = [],
            request_cb_kwargs = {},
            request_key = None,
            request_slots = [],
            # Response Policy
            response_cb = None,
            response_cb_args = [],
            response_cb_kwargs = {},
            response_key = None,
            response_slots = [],
            # Keys
            input_keys = [],
            output_keys = [],
            outcomes = [],
            ):

        RosState.__init__(self, node, outcomes=['succeeded', 'aborted', 'preempted'])

        # Store Service info
        self._service_name = service_name
        self._service_spec = service_spec

        self._proxy = None

        # Store request policy
        if request is None:
            self._request = service_spec.Request()
        else:
            self._request = request


        if request_cb is not None and not hasattr(request_cb, '__call__'):
            raise smach.InvalidStateError("Request callback object given to ServiceState that IS NOT a function object")

        self._request_cb = request_cb
        self._request_cb_args = request_cb_args
        self._request_cb_kwargs = request_cb_kwargs
        if smach.has_smach_interface(request_cb):
            self._request_cb_input_keys = request_cb.get_registered_input_keys()
            self._request_cb_output_keys = request_cb.get_registered_output_keys()

            self.register_input_keys(self._request_cb_input_keys)
            self.register_output_keys(self._request_cb_output_keys)
        else:
            self._request_cb_input_keys = input_keys
            self._request_cb_output_keys = output_keys

        self._request_key = request_key
        if request_key is not None:
            self.register_input_keys([request_key])

        self._request_slots = request_slots
        self.register_input_keys(request_slots)

        # Store response policy
        if response_cb is not None and not hasattr(response_cb, '__call__'):
            raise smach.InvalidStateError("Response callback object given to ServiceState that IS NOT a function object")

        self._response_cb = response_cb
        self._response_cb_args = response_cb_args
        self._response_cb_kwargs = response_cb_kwargs
        if smach.has_smach_interface(response_cb):
            self._response_cb_input_keys = response_cb.get_registered_input_keys()
            self._response_cb_output_keys = response_cb.get_registered_output_keys()
            self._response_cb_outcomes = response_cb.get_registered_outcomes()

            self.register_input_keys(self._response_cb_input_keys)
            self.register_output_keys(self._response_cb_output_keys)
            self.register_outcomes(self._response_cb_outcomes)
        else:
            self._response_cb_input_keys = input_keys
            self._response_cb_output_keys = output_keys
            self._response_cb_outcomes = outcomes

        # Register additional input and output keys
        self.register_input_keys(input_keys)
        self.register_output_keys(output_keys)
        self.register_outcomes(outcomes)

        self._response_key = response_key
        if response_key is not None:
            self.register_output_keys([response_key])

        self._response_slots = response_slots
        self.register_output_keys(response_slots)

        self._proxy = self.node.create_client(self._service_spec, self._service_name)

    def execute(self, ud):
        """Execute service"""
        # Check for preemption before executing
        if self.preempt_requested():
            self.node.get_logger().info("Preempting %s before sending request." % self._service_name)
            self.service_preempt()
            return 'preempted'

        # Make sure we're connected to the service
        try:
            # 1. Wait for the service to be ready
            while not self._proxy.wait_for_service(timeout_sec=1.0):
                if self.preempt_requested():
                    self.node.get_logger().info("Preempting %s while waiting for service." % self._service_name)
                    self.service_preempt()
                    return 'preempted'
                if not rclpy.ok():
                    self.node.get_logger().error("ROS2 shutdown while waiting for service %s." % self._service_name)
                    return 'aborted'
                self.node.get_logger().info("Waiting for service %s..." % self._service_name)

            # 2. Prepare the service request
            request_msg = self._request
            if isinstance(request_msg, type):
                request_msg = request_msg()
            
            if self._request_key is not None:
                if self._request_key in ud:
                    request_msg = ud[self._request_key]
            
            for key in self._request_slots:
                if key in ud:
                    setattr(request_msg, key, ud[key])

            # 3. Call the user-supplied request_cb if it exists
            if self._request_cb is not None:
                try:
                    req_cb_result = self._request_cb(
                        smach.Remapper(
                            ud,
                            self._request_cb_input_keys,
                            self._request_cb_output_keys,
                            []),
                        request_msg,
                        *self._request_cb_args,
                        **self._request_cb_kwargs
                    )
                    if req_cb_result is not None:
                        request_msg = req_cb_result
                except Exception as e:
                    self.node.get_logger().error("Could not execute request callback: " + traceback.format_exc())
                    return 'aborted'

            # 4. Perform a SYNCHRONOUS-LIKE service call
            self._response = None
            call_future = self._proxy.call_async(request_msg)
            
            # Create a temporary executor to spin until future is done or preempted
            temp_executor = SingleThreadedExecutor()
            temp_executor.add_node(self.node)
            
            while rclpy.ok() and not call_future.done():
                if self.preempt_requested():
                    self.node.get_logger().info("Preempting %s during service call." % self._service_name)
                    self.service_preempt()
                    temp_executor.shutdown()
                    return 'preempted'
                temp_executor.spin_once(timeout_sec=0.1)
            
            temp_executor.shutdown()

            if call_future.done():
                try:
                    self._response = call_future.result()
                except Exception as e:
                    self.node.get_logger().error("Service call failed: " + str(e))
                    return 'aborted'
            else:
                self.node.get_logger().error("Service call did not complete successfully.")
                return 'aborted'

            # 5. Handle the response (Logic continues below with response_cb)
            
        except Exception as e:
            self.node.get_logger().error("Service execution failed: " + traceback.format_exc())
            return 'aborted'

        response_cb_outcome = None
        if self._response_cb is not None:
            try:
                response_cb_outcome = self._response_cb(
                        smach.Remapper(
                                ud,
                                self._response_cb_input_keys,
                                self._response_cb_output_keys,
                                []),
                        self._response,
                        *self._response_cb_args,
                        **self._response_cb_kwargs)
                if response_cb_outcome is not None and response_cb_outcome not in self.get_registered_outcomes():
                    self.node.get_logger().error("Result callback for service "+self._service_name+", "+str(self._response_cb)+" was not registered with the response_cb_outcomes argument. The response callback returned '"+str(response_cb_outcome)+"' but the only registered outcomes are: "+str(self.get_registered_outcomes()))
                    return 'aborted'
            except:
                self.node.get_logger().error("Could not execute response callback: "+traceback.format_exc())
                return 'aborted'

        if self._response_key is not None:
            ud[self._response_key] = self._response

        for key in self._response_slots:
            ud[key] = getattr(self._response,key)

        if response_cb_outcome is not None:
            return response_cb_outcome

        return 'succeeded'