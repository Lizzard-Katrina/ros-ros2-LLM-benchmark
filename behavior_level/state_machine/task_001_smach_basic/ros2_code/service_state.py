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
            # Wait for the service to be ready, handle preemption and shutdown
            while not self._proxy.wait_for_service(timeout_sec=0.1):
                if self.preempt_requested():
                    self.node.get_logger().info("Preempting %s while waiting for service." % self._service_name)
                    self.service_preempt()
                    return 'preempted'
                if not rclpy.ok():
                    self.node.get_logger().error("ROS shutdown while waiting for service %s" % self._service_name)
                    return 'aborted'

            # Prepare request
            if self._request_key is not None:
                self._request = ud[self._request_key]
            for slot in self._request_slots:
                setattr(self._request, slot, ud[slot])

            # Call request callback if provided
            if self._request_cb is not None:
                remapped_ud = smach.Remapper(ud, self._request_cb_input_keys, self._request_cb_output_keys, [])
                self._request_cb(remapped_ud, self._request, *self._request_cb_args, **self._request_cb_kwargs)

            # Perform synchronous-like service call
            future = self._proxy.call_async(self._request)
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if future.done():
                    break
                if self.preempt_requested():
                    self.node.get_logger().info("Preempting %s during service call." % self._service_name)
                    self.service_preempt()
                    return 'preempted'
            if not future.done():
                self.node.get_logger().error("Service call to %s did not complete." % self._service_name)
                return 'aborted'

            self._response = future.result()
            if self._response is None:
                self.node.get_logger().error("Service call to %s failed." % self._service_name)
                return 'aborted'

        except Exception as e:
            self.node.get_logger().error("Service call to %s failed: %s" % (self._service_name, str(e)))
            return 'aborted'

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
                import traceback
                self.node.get_logger().error("Could not execute response callback: "+traceback.format_exc())
                return 'aborted'
        else:
            response_cb_outcome = None

        if self._response_key is not None:
            ud[self._response_key] = self._response

        for key in self._response_slots:
            ud[key] = getattr(self._response,key)

        if response_cb_outcome is not None:
            return response_cb_outcome

        return 'succeeded'