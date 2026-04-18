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


# LLM Response

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.key = None
        self.settings = termios.tcgetattr(sys.stdin)
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termIOS.TCSADRAIN, self.settings)
        return self.key
        
    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s" % (target_linear_vel, target_angular_vel)

def run_teleop():
    rclpy.init(args=sys.argv)
    teleop = TeleopNode()
    
    linear_vel = 0.0
    angular_vel = 0.0
    linear_step = 0.1
    angular_step = 0.1
    
    status = 0
    
    try:
        while rclpy.ok():
            key = teleop.get_key()
            if key == 'w':
                linear_vel = min(linear_vel + linear_step, 1.0)
                status += 1
            elif key == 'x':
                linear_vel = max(linear_vel - linear_step, -1.0)
                status += 1
            elif key == 'a':
                angular_vel = min(angular_vel + angular_step, 1.0)
                status += 1
            elif key == 'd':
                angular_vel = max(angular_vel - angular_step, -1.0)
                status += 1
            elif key == ' ':
                linear_vel = 0.0
                angular_vel = 0.0
            elif key == '\x03':
                break
            
            if status == 20:
                print(teleop.vels(linear_vel, angular_vel))
                status = 0
            
            twist = Twist()
            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            teleop.pub.publish(twist)
            
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        teleop.pub.publish(twist)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    run_teleop()
```