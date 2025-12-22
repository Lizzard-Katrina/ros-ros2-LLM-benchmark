import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

TEST_MESSAGES = ["hello1", "hello2", "hello3"]
RECEIVED_MESSAGES = []

class TestListener(Node):
    def __init__(self):
        super().__init__('test_listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        print(f"[listener] Received: {msg.data}")
        RECEIVED_MESSAGES.append(msg.data)

class TestTalker(Node):
    def __init__(self):
        super().__init__('test_talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)

    def publish_messages(self):
        for msg_text in TEST_MESSAGES:
            msg = String()
            msg.data = msg_text
            self.publisher.publish(msg)
            print(f"[talker] Published: {msg_text}")
            time.sleep(0.5)

def main():
    rclpy.init()
    listener = TestListener()
    talker = TestTalker()

    # 发布消息 + spin_once 循环接收
    for _ in range(10):
        talker.publish_messages()
        rclpy.spin_once(listener, timeout_sec=0.5)

    listener.destroy_node()
    talker.destroy_node()
    rclpy.shutdown()

    # 验证消息
    if all(msg in RECEIVED_MESSAGES for msg in TEST_MESSAGES):
        print("\n[RESULT] PASS: All messages received!")
    else:
        print("\n[RESULT] FAIL: Missing messages!")
        print("Expected:", TEST_MESSAGES)
        print("Received:", RECEIVED_MESSAGES)

if __name__ == "__main__":
    main()
