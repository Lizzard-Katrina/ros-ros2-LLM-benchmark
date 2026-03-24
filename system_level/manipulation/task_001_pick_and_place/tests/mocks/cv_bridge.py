# ros2_code/mocks/cv_bridge.py
class CvBridge:
    def imgmsg_to_cv2(self, msg, desired_encoding='passthrough'):
        return None

    def cv2_to_imgmsg(self, cv_image, encoding='passthrough'):
        return None
