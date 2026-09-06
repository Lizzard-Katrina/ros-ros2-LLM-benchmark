#!/usr/bin/env python
#
# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Rosbridge class:

Class that handle communication between CARLA and ROS
"""

import os
import pkg_resources
try:
    import queue
except ImportError:
    import Queue as queue
import sys
from distutils.version import LooseVersion
from threading import Thread, Lock, Event

try:
    import carla
except ImportError:
    pass

try:
    import ros_compatibility as roscomp
    from ros_compatibility.node import CompatibleNode
except ImportError:
    import rclpy as roscomp
    from rclpy.node import Node as CompatibleNode

try:
    from carla_ros_bridge.actor import Actor
    from carla_ros_bridge.actor_factory import ActorFactory
    from carla_ros_bridge.carla_status_publisher import CarlaStatusPublisher
    from carla_ros_bridge.debug_helper import DebugHelper
    from carla_ros_bridge.ego_vehicle import EgoVehicle
    from carla_ros_bridge.world_info import WorldInfo
except ImportError:
    pass

try:
    from carla_msgs.msg import CarlaControl, CarlaWeatherParameters
    from carla_msgs.srv import SpawnObject, DestroyObject, GetBlueprints
except ImportError:
    pass

from rosgraph_msgs.msg import Clock


class CarlaRosBridge(object):

    """
    Carla Ros bridge
    """

    # in synchronous mode, if synchronous_mode_wait_for_vehicle_control_command is True,
    # wait for this time until a next tick is triggered.
    VEHICLE_CONTROL_TIMEOUT = 1.

    def __init__(self):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param params: dict of parameters, see settings.yaml
        :type params: dict
        """
        super(CarlaRosBridge, self).__init__()

    # pylint: disable=attribute-defined-outside-init
    def initialize_bridge(self, carla_world, params):
        """
        Initialize the bridge
        """
        self.parameters = params
        self.carla_world = carla_world

        self.ros_timestamp = None
        self.callback_group = None

        self.synchronous_mode_update_thread = None
        self.shutdown = Event()

        self.carla_settings = carla_world.get_settings()
        if not self.parameters["passive"]:
            if self.carla_settings.synchronous_mode:
                self.carla_settings.synchronous_mode = False
                carla_world.apply_settings(self.carla_settings)

            self.carla_settings.synchronous_mode = self.parameters["synchronous_mode"]
            self.carla_settings.fixed_delta_seconds = self.parameters["fixed_delta_seconds"]
            carla_world.apply_settings(self.carla_settings)

        self.sync_mode = self.carla_settings.synchronous_mode and not self.parameters["passive"]

        self.carla_control_queue = queue.Queue()

        # actor factory
        self.actor_factory = ActorFactory(self, carla_world, self.sync_mode)

        # world info
        self.world_info = WorldInfo(carla_world=self.carla_world, node=self)
        # debug helper
        self.debug_helper = DebugHelper(carla_world.debug, self)

        self.status_publisher = CarlaStatusPublisher(
            self.carla_settings.synchronous_mode,
            self.carla_settings.fixed_delta_seconds,
            self)

        self._all_vehicle_control_commands_received = Event()
        self._expected_ego_vehicle_control_command_ids = []
        self._expected_ego_vehicle_control_command_ids_lock = Lock()

        if self.sync_mode:
            self.carla_run_state = CarlaControl.PLAY

            self.synchronous_mode_update_thread = Thread(
                target=self._synchronous_mode_update)
            self.synchronous_mode_update_thread.start()
        else:
            self.timestamp_last_run = 0.0
            self.actor_factory.start()
            self.on_tick_id = self.carla_world.on_tick(self._carla_time_tick)

        self._registered_actors = []

    def process_run_state(self):
        """
        process state changes
        """
        command = None

        while not self.carla_control_queue.empty():
            command = self.carla_control_queue.get()

        while command is not None:
            self.carla_run_state = command

            if self.carla_run_state == CarlaControl.PAUSE:
                self.status_publisher.set_synchronous_mode_running(False)
                command = self.carla_control_queue.get()
            elif self.carla_run_state == CarlaControl.PLAY:
                self.status_publisher.set_synchronous_mode_running(True)
                return
            elif self.carla_run_state == CarlaControl.STEP_ONCE:
                self.status_publisher.set_synchronous_mode_running(True)
                self.carla_control_queue.put(CarlaControl.PAUSE)
                return

    def _synchronous_mode_update(self):
        """
        Implement the Lockstep Simulation Barrier.
        This block is responsible for the atomic stepping of the simulation.
        Synchronize the internal ActorFactory, the CARLA world tick,
        and the ROS system clock to ensure that all published sensor data
        and TF transforms are temporally aligned with the simulation frame.
        """
        while not self.shutdown.is_set():
            self.process_run_state()

            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                self._expected_ego_vehicle_control_command_ids = []
                with self._expected_ego_vehicle_control_command_ids_lock:
                    for actor_id in self.actor_factory.actors:
                        if isinstance(self.actor_factory.actors[actor_id], EgoVehicle):
                            self._expected_ego_vehicle_control_command_ids.append(actor_id)

            self.actor_factory.update_available_objects()

            frame = self.carla_world.tick()

            world_snapshot = self.carla_world.get_snapshot()

            self.update_clock(world_snapshot.timestamp)
            self.status_publisher.set_frame(frame)
            self._update(frame, world_snapshot.timestamp.elapsed_seconds)

            if self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
                if self._expected_ego_vehicle_control_command_ids:
                    if not self._all_vehicle_control_commands_received.wait(CarlaRosBridge.VEHICLE_CONTROL_TIMEOUT):
                        self.logwarn("Timeout ({}s) while waiting for ego vehicle control commands. "
                                     "Missing command from {}".format(
                                         CarlaRosBridge.VEHICLE_CONTROL_TIMEOUT,
                                         self._expected_ego_vehicle_control_command_ids))
                    self._all_vehicle_control_commands_received.clear()

    def _carla_time_tick(self, carla_snapshot):
        """
        Private callback registered at carla.World.on_tick()
        to trigger cyclic updates.

        After successful locking the update mutex
        (only perform trylock to respect bridge processing time)
        the clock and the children are updated.
        Finally the ROS messages collected to be published are sent out.

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        if not self.shutdown.is_set():
            if self.timestamp_last_run < carla_snapshot.timestamp.elapsed_seconds:
                self.timestamp_last_run = carla_snapshot.timestamp.elapsed_seconds
                self.update_clock(carla_snapshot.timestamp)
                self.status_publisher.set_frame(carla_snapshot.frame)
                self._update(carla_snapshot.frame,
                             carla_snapshot.timestamp.elapsed_seconds)

    def _update(self, frame_id, timestamp):
        """
        update all actors
        :return:
        """
        self.world_info.update(frame_id, timestamp)
        self.actor_factory.update_actor_states(frame_id, timestamp)

    def _ego_vehicle_control_applied_callback(self, ego_vehicle_id):
        if not self.sync_mode or \
                not self.parameters['synchronous_mode_wait_for_vehicle_control_command']:
            return
        with self._expected_ego_vehicle_control_command_ids_lock:
            if ego_vehicle_id in self._expected_ego_vehicle_control_command_ids:
                self._expected_ego_vehicle_control_command_ids.remove(
                    ego_vehicle_id)
            else:
                self.logwarn(
                    "Unexpected vehicle control command received from {}".format(ego_vehicle_id))
            if not self._expected_ego_vehicle_control_command_ids:
                self._all_vehicle_control_commands_received.set()

    def update_clock(self, carla_timestamp):
        """
        perform the update of the clock

        :param carla_timestamp: the current carla time
        :type carla_timestamp: carla.Timestamp
        :return:
        """
        self.ros_timestamp = carla_timestamp.elapsed_seconds

    def loginfo(self, msg):
        print("[INFO] {}".format(msg))

    def logwarn(self, msg):
        print("[WARN] {}".format(msg))

    def logerr(self, msg):
        print("[ERROR] {}".format(msg))

    def logfatal(self, msg):
        print("[FATAL] {}".format(msg))

    def destroy(self):
        """
        Function to destroy this object.

        :return:
        """
        self.shutdown.set()
        if not self.sync_mode:
            if self.on_tick_id:
                self.carla_world.remove_on_tick(self.on_tick_id)
            self.actor_factory.thread.join()
        else:
            self.synchronous_mode_update_thread.join()
        self.debug_helper.destroy()
        self.status_publisher.destroy()
        for uid in self._registered_actors:
            self.actor_factory.destroy_actor(uid)
        self.actor_factory.update_available_objects()
        self.actor_factory.clear()


def main(args=None):
    """
    main function for carla simulator ROS bridge
    maintaining the communication client and the CarlaBridge object
    """
    pass


if __name__ == "__main__":
    main()