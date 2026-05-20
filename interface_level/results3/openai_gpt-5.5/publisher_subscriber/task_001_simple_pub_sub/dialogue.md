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

FILE_PATH: listener.py
----------------------------
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    # Keep logging logic
    rospy.loginfo("I heard %s", data.data)

def listener():
    # TODO: initialize node 'listener'
    # and subscribe to topic 'chatter'
    # and keep spin
    # END OF TODO
if __name__ == '__main__':
    listener()



----------------------------

FILE_PATH: talker.py
----------------------------
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    # TODO: create a ROS publisher for topic 'chatter'
    # initialize node 'talker'
    # END OF TODO
    rate = rospy.Rate(1)  # Keep this line

    while not rospy.is_shutdown():
        msg = "Hello world %s" % rospy.get_time()   # Keep message logic
        rospy.loginfo(msg)

        # TODO: publish the message
        #END OF TODO
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

----------------------------


# LLM Response

[FILENAME: listener.py]
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

_node = None

def callback(data):
    # Keep logging logic
    _node.get_logger().info("I heard %s" % data.data)

def listener():
    # TODO: initialize node 'listener'
    # and subscribe to topic 'chatter'
    # and keep spin
    # END OF TODO
    global _node
    rclpy.init()
    _node = rclpy.create_node('listener')
    _node.create_subscription(String, 'chatter', callback, 10)
    try:
        rclpy.spin(_node)
    finally:
        _node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    listener()


[FILENAME: talker.py]
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

class RosPyCompatRate:
    def __init__(self, node, hz):
        self._node = node
        self._period = 1.0 / float(hz)

    def sleep(self):
        rclpy.spin_once(self._node, timeout_sec=self._period)

class RosPyCompat:
    class ROSInterruptException(Exception):
        pass

    def __init__(self):
        self._node = None

    def init_node(self, name):
        rclpy.init()
        self._node = rclpy.create_node(name)

    def Publisher(self, topic, msg_type, queue_size=10):
        return self._node.create_publisher(msg_type, topic, queue_size)

    def Rate(self, hz):
        return RosPyCompatRate(self._node, hz)

    def is_shutdown(self):
        return not rclpy.ok()

    def get_time(self):
        return self._node.get_clock().now().nanoseconds / 1e9

    def loginfo(self, msg, *args):
        if args:
            msg = msg % args
        self._node.get_logger().info(msg)

    def shutdown(self):
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

rospy = RosPyCompat()

def talker():
    # TODO: create a ROS publisher for topic 'chatter'
    # initialize node 'talker'
    # END OF TODO
    rospy.init_node('talker')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)  # Keep this line

    try:
        while not rospy.is_shutdown():
            msg = "Hello world %s" % rospy.get_time()   # Keep message logic
            rospy.loginfo(msg)

            # TODO: publish the message
            #END OF TODO
            ros_msg = String()
            ros_msg.data = msg
            pub.publish(ros_msg)
            rate.sleep()
    finally:
        rospy.shutdown()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass