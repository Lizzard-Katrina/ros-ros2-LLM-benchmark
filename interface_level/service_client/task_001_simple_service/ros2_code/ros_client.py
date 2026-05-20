import sys
import rclpy
from rclpy.node import Node
from beginner_tutorials.srv import AddTwoInts

def client_node():
    rclpy.init()
    node = rclpy.create_node('add_two_ints_client')
    
    client = node.create_client(AddTwoInts, 'add_two_ints')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')
        
    req = AddTwoInts.Request()
    req.a = 5
    req.b = 3
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is not None:
        node.get_logger().info('Result: %d' % future.result().sum)
    else:
        node.get_logger().error('Exception while calling service: %r' % future.exception())
        
    node.get_logger().info("Client node running")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    client_node()