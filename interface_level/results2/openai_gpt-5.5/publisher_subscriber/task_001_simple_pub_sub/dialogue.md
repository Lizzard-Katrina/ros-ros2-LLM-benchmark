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
    global _node
    # TODO: initialize node 'listener'
    # and subscribe to topic 'chatter'
    # and keep spin
    # END OF TODO
    rclpy.init()
    _node = rclpy.create_node('listener')
    _node.create_subscription(String, 'chatter', callback, 10)
    rclpy.spin(_node)
    _node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    listener()


[FILENAME: talker.py]
#!/usr/bin/env python3
import rclpy
from std_msgs.msg import String

_node = None

class _RospyCompat:
    class ROSInterruptException(Exception):
        pass

    @staticmethod
    def Rate(hz):
        return _node.create_rate(hz)

    @staticmethod
    def is_shutdown():
        return not rclpy.ok()

    @staticmethod
    def get_time():
        return _node.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def loginfo(msg):
        _node.get_logger().info(str(msg))

rospy = _RospyCompat

def talker():
    global _node
    rclpy.init()
    _node = rclpy.create_node('talker')
    # TODO: create a ROS publisher for topic 'chatter'
    # initialize node 'talker'
    # END OF TODO
    pub = _node.create_publisher(String, 'chatter', 10)
    rate = rospy.Rate(1)  # Keep this line

    while not rospy.is_shutdown():
        msg = "Hello world %s" % rospy.get_time()   # Keep message logic
        rospy.loginfo(msg)

        # TODO: publish the message
        #END OF TODO
        pub.publish(String(data=msg))
        rate.sleep()

    _node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass