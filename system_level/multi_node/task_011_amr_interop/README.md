# VDA5050 ROS 2 MQTT Connector

## 1. Brief Description
This project implements a **VDA5050 Protocol Bridge** designed to enable communication between industrial mobile robots (AGVs/AMRs) in a ROS 2 environment and a central Master Control system. It acts as middleware that translates **MQTT messages** (JSON format) conforming to the VDA5050 standard into **ROS 2 topics**, while simultaneously gathering robot status (State) and pose (Visualization) data to be published back to the MQTT Broker.

The core components include:
* **MQTTBridge (Python)**: Manages low-level MQTT connections, TLS security, topic subscriptions/publishing, and bidirectional JSON-to-ROS message serialization.
* **StateHandler (C++)**: Provides an abstract interface for state management, ensuring standardized logic for extracting robot status data.

---
source:

```https://github.com/inorbit-ai/ros_amr_interop/blob/humble-devel/vda5050_connector/vda5050_connector_py```


## 2. Design Logic for Fill-in-the-Blanks

### Task_011_A: StateHandler Interface Design (C++)
* **Abstract Encapsulation**: To ensure compatibility across different robot models with varying state-extraction logic, a `StateHandler` base class was designed inheriting from a generic `Handler`.
* **State Persistence**: A protected member variable `current_state_msg_` was introduced. This allows subclasses to maintain and update the state persistently across execution cycles without redundant memory allocations.
* **Lifecycle Hooks**: Pure virtual methods `configure()` (for resource/subscriber initialization) and `execute()` (for periodic logic processing) were defined to align with the ROS 2 component lifecycle.

### Task_011_C: ROS 2 Subscriptions & Message Flow (Python)
* **Decoupled Orchestration**: The `on_configure` method serves as the central hub for ROS 2 subscribers. It utilizes the `get_vda5050_ros2_topic` helper to ensure internal ROS topics follow the standard naming convention: `/{manufacturer}/{serial_number}/{interface_name}/{topic}`.
* **Directional Data Flow**: 
    * **MQTT to ROS**: Subscribes to MQTT `order` and `instantActions` topics, converting JSON payloads into ROS messages for local robot consumption.
    * **ROS to MQTT**: Subscribes to local `state`, `connection`, and `visualization` topics, using callbacks like `_publish_state` to push telemetry to the remote Master Control.
* **Error Resilience**: Implemented `try-except` blocks within `on_message_mqtt` to catch JSON decoding errors or missing keys, preventing the bridge from crashing due to malformed external messages.

---

## 3. Oracle Testcases & Expected Outcomes

### Case 1: Connection & Online Notification
* **Logic**: Verify that the node correctly configures the MQTT client upon startup and sends the initial "ONLINE" handshake.
* **Expected Outcome**:
    1.  MQTT Broker receives a message on `uagv/v2/robots/robot_1/connection`.
    2.  JSON payload contains `"connectionState": "ONLINE"`.
    3.  ROS 2 logs display: `Connected to MQTT Broker!`.

### Case 2: Order Message Forwarding
* **Logic**: Simulate the Master Control issuing a VDA5050 Order.
* **Input**: Publish a JSON string containing `orderId` and `nodes` to the MQTT order topic.
* **Expected Outcome**:
    1.  The `MQTTBridge` node captures the MQTT message.
    2.  A corresponding `vda5050_msgs/msg/Order` message is observed on the local ROS 2 topic via `ros2 topic echo`.

### Case 3: Abnormal Disconnection (Last Will)
* **Logic**: Verify that the Broker publishes the "Last Will" message if the bridge process is killed or loses network.
* **Action**: Forcefully terminate the process (`kill -9`).
* **Expected Outcome**:
    1.  The MQTT Broker publishes the pre-configured `will_payload` after the heartbeat timeout.
    2.  `connectionState` is set to `"CONNECTIONBROKEN"`.
    3.  `timestamp` shows the epoch default `"1970-01-01T12:00:00.00Z"`, signaling an involuntary exit to the supervisor.

### Case 4: Graceful Shutdown
* **Logic**: Verify the node updates its status before a clean exit.
* **Action**: Send a `SIGINT` (Ctrl+C) to the node.
* **Expected Outcome**:
    1.  Node publishes a final message to the MQTT `connection` topic.
    2.  `connectionState` is set to `"OFFLINE"`.
    3.  The `headerId` is incremented by exactly 1 compared to the previous successful transmission.
