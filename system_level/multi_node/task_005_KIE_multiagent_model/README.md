# ROS 2 Migration Benchmark: Task 005 KIE Multiagent Model

## 1. Brief Description
This benchmark task evaluates a Large Language Model's (LLM) ability to migrate core discovery and network-probing components from ROS 1 to ROS 2. The focus is on the `fkie_mas_discovery` package, specifically handling the transition from a centralized Master-based discovery system (XML-RPC) to a decentralized, node-centric discovery architecture. 

The task requires the model to maintain **Static Architectural Consistency** across two interdependent files while transitioning from `rospy` to `rclpy`.

---
source code file:
```https://github.com/fkie/fkie-multi-agent-suite/blob/master/fkie_mas_discovery/src/fkie_mas_discovery```


## 2. Hollowing Design & System Coupling

### File A: `master_discovery.py` (The Heart)
* **Holes:** * `Discoverer.__init__` (Publisher definitions).
    * `Discoverer._publish_current_state` (Logic for heartbeat and status broadcasting).
* **Design Intent:** This file represents the high-level application logic. The LLM must correctly instantiate ROS 2 publishers and map the internal state to the outgoing communication layer.
* **System Coupling:** It is tightly coupled with `interface_finder.py`. The data retrieved by the finder must be processed here. If the LLm changes the internal data structure in the finder but fails to update the processing logic here, the system fails to remain consistent.

### File B: `interface_finder.py` (The Utility)
* **Hole:** The entire `_get_topic` function.
* **Design Intent:** This is a low-level discovery tool. In ROS 1, it queries the Master via XML-RPC. In ROS 2, it must be rewritten to use the Node's Graph API (`get_topic_names_and_types`).
* **System Coupling:** It provides the "source of truth" regarding active topics. Any architectural mismatch in how topics are filtered or returned will break the status updates in `master_discovery.py`.

---

## 3. Oracle Test Cases & Expected Outcomes

### master_discovery.py Tests
| Test Case | Design Intent | Expected Outcome |
| :--- | :--- | :--- |
| `test_md_ros2_publisher_definition` | Verifies the use of `create_publisher` with ROS 2 types. | Must find `create_publisher` calls using `MasterState` and `LinkStatesStamped`. |
| `test_md_no_rospy_time_leakage` | Ensures no legacy ROS 1 time artifacts remain. | Absence of `rospy.Time`. |
| `test_md_udp_protocol_preservation` | Ensures the underlying binary UDP protocol is intact. | Presence of `struct.pack` using `HEARTBEAT_FMT`. |
| `test_md_variable_consistency` | Checks if variable names match between init and use. | Defined `self.pubchanges` must be used to `.publish()`. |

### interface_finder.py Tests
| Test Case | Design Intent | Expected Outcome |
| :--- | :--- | :--- |
| `test_if_no_xmlrpc_usage` | Ensures removal of Master-dependent XML-RPC libraries. | Absence of `xmlrpc` and `ServerProxy`. |
| `test_if_graph_api_usage` | Checks for the correct ROS 2 discovery API. | Presence of `get_topic_names_and_types`. |
| `test_if_host_filtering_logic` | Ensures business logic (filtering by host) is preserved. | Usage of `get_hostname` within the filtering loop. |
| `test_if_return_type_contract` | Maintains API compatibility with the rest of the system. | The function must return a `list` of strings. |
