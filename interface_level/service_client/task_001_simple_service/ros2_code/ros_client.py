import rclpy
from beginner_tutorials.srv import AddTwoInts


def client_node():
    rclpy.init()
    node = rclpy.create_node('add_two_ints_client')
    client = node.create_client(AddTwoInts, 'add_two_ints')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting...')

    req = AddTwoInts.Request()
    req.a = 1
    req.b = 2

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f"Result: {req.a} + {req.b} = {response.sum}")
    else:
        node.get_logger().error("Failed to call service add_two_ints")

    node.get_logger().info("Client node running")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    client_node()