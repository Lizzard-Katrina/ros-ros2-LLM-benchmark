#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.

import rclpy
from vda5050_connector_py.mqtt_bridge import MQTTBridge


def main(args=None):
    rclpy.init(args=args)

    mqtt_bridge = MQTTBridge()

    try:
        rclpy.spin(mqtt_bridge)
    except KeyboardInterrupt:
        pass
    except BaseException:
        raise
    finally:
        mqtt_bridge.on_shutdown()
        mqtt_bridge.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()