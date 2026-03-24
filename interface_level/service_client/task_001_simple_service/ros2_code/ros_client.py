import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        self.future = self.cli.call_async(req)

def client_node():
    rclpy.init()
    node = AddTwoIntsClient()
    node.get_logger().info("Client node running")
    # TODO: call send_request with desired values
    node.send_request(1, 2)  # TODO: replace with desired values
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info('Result of add_two_ints: for %d + %d = %d' %
                                       (1, 2, response.sum))  # TODO: replace with desired values
            break

if __name__ == "__main__":
    client_node()