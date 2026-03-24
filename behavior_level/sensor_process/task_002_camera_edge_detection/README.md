# Task 002: Camera Edge Detection - Vision Processing Pipeline

## 1. Brief Description
Task 002 is a core **Behavior Level** task focused on visual perception. It requires implementing a Python-based ROS 2 node that transforms a raw BGR camera stream into a structural edge-map using the Canny algorithm. The node must handle the complete data lifecycle: subscribing to `sensor_msgs/Image`, converting it for OpenCV processing, applying feature extraction, and re-publishing the result with synchronized metadata.
--
source tutorial and code:
```https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython```
## 2. Excavation Strategy: Perception Standards & Constraints
The excavation removes the entire internal logic of the `image_callback`. Unlike a simple "translation" task, this excavation forces the developer (or AI) to adhere to **Industrial Perception Standards**:
* **Explicit Pre-processing**: Mandates a manual conversion to Grayscale to ensure Canny algorithm stability, rather than relying on implicit library behavior.
* **Style Guardrails**: Enforces the use of **positional arguments** and **direct header assignment**. This is designed to test "Instruction Following" and ensure the code remains compatible with strict automated verification systems.
* **Temporal Synchronization**: Requires the manual preservation of the `header` object to prevent "sensor lag" in downstream fusion nodes.

## 3. Oracle Test Design & Expected Outcomes

The Oracle suite uses specific regex patterns to validate the **architectural intent** and **style compliance** of the implementation.

| Test Case | Design / Intent | Expected Outcome (To Pass) |
| :--- | :--- | :--- |
| **Ingestion Style** | Validates the use of positional arguments for `imgmsg_to_cv2`. | Must match `imgmsg_to_cv2(data, 'bgr8')`; keywords like `desired_encoding=` are prohibited. |
| **Explicit Grayscale** | Ensures the "Perception Expert" step of converting BGR to Gray is present. | Must contain `COLOR_BGR2GRAY` or `cvtColor` call. |
| **Canny Algorithm** | Verifies the core processing behavior. | Presence of `cv2.Canny` or equivalent functional call. |
| **Egress Style** | Validates positional arguments and correct output encoding. | Must match `cv2_to_imgmsg(edges, 'mono8')`. |
| **Metadata Sync** | Ensures the output message is temporally aligned with the sensor. | Must use full assignment: `output_msg.header = data.header`. |
| **Error Handling** | Checks for node robustness against corrupted frames. | Conversion logic must be wrapped in `try-except CvBridgeError`. |
| **No Legacy API** | Prevents "ROS 1 Hallucination." | Total absence of `rospy` or ROS 1 style global publishers. |

## 4. Engineering Impact
Passing Task 002 demonstrates that the model can handle the **Visual Pipeline** with high precision. It proves the ability to follow complex style constraints while maintaining the physical integrity of the data (correct encodings and timestamps), which is the bedrock of reliable autonomous navigation.
