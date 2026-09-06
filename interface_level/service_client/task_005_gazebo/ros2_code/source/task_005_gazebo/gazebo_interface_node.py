import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class GazeboInterfaceNode(Node):
    """
    A ROS2 node that provides a simple service to demonstrate
    the gazebo interface migration pattern.
    """

    def __init__(self):
        super().__init__('gazebo_interface_node')
        self.get_logger().info('Gazebo interface node started')

        # Provide a simple service for testing
        self.srv = self.create_service(
            SetBool,
            'spawn_entity',
            self.spawn_entity_callback,
        )
        self.get_logger().info('Service spawn_entity is ready')

    def spawn_entity_callback(self, request, response):
        """Handle spawn entity requests."""
        if request.data:
            response.success = True
            response.message = 'Entity spawned successfully'
        else:
            response.success = False
            response.message = 'Spawn request declined'
        self.get_logger().info('spawn_entity called: success=%s' % response.success)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GazeboInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()