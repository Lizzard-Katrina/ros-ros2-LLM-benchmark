# ros2_code/mocks/rclpy.py
class Node:
    def __init__(self, name):
        self.name = name

    def create_publisher(self, msg_type, topic, qos):
        return PublisherMock()

    def create_subscription(self, msg_type, topic, callback, qos):
        return SubscriptionMock()

    def destroy_node(self):
        pass

class PublisherMock:
    def publish(self, msg):
        pass

class SubscriptionMock:
    pass

def init():
    pass

def spin(node):
    pass

def shutdown():
    pass
