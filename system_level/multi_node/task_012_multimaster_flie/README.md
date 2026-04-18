# Task 012: ROS Multi-Master Synchronization Logic Migration

## 1. Brief Description
This task involves migrating core components of the `fkie_multimaster` stack from a legacy environment to a system-level correctness evaluation framework. The primary objective is to implement the bridge logic that allows disjoint ROS Masters to discover each other and synchronize topics/services across different network nodes. The migration focuses on two critical functional pillars: 
- **State Monitoring**: Observing the local ROS Master topology.
- **Remote Synchronization**: Mirroring remote states onto the local Master while preventing infinite loops and maintaining peer-to-peer transparency.
--

source:
```https://github.com/fkie/multimaster_fkie/blob/master/fkie_master_sync```
---
## 2. Design Patterns for Code "Holes" (TODOs)

### A. MasterMonitor.updateState()
* **Design Intent**: This hole represents the "Sensor" of the multi-master system. The goal is to test the model's ability to interface with the ROS Master's XML-RPC API and transform low-level nested lists into a structured `MasterInfo` object.
* **Constraint Logic**: 
    - **Strict Interface Adherence**: The model is forced to use `self._succeed()` for error wrapping, mimicking the specific architectural pattern of the FKIE stack.
    - **Data Mapping**: It requires mapping `topicTypes` to the `systemState` results, ensuring the model understands the relational nature of ROS Master data.
    - **Atomicity**: The implementation must update `self.__new_master_state` to ensure the internal state remains consistent during the polling cycle.

### B. SyncThread._apply_remote_state()
* **Design Intent**: This hole represents the "Actuator". It tests the logic required to synchronize remote data into the local environment without causing a system collapse (e.g., infinite feedback loops).
* **Constraint Logic**:
    - **Loop Prevention**: The model must explicitly check if a remote node name matches its own identity (`self.ros_node_name`), a critical requirement for multi-node system correctness.
    - **Performance Optimization**: It mandates the use of `own_master_multi()` (XML-RPC MultiCall) to batch registration requests, reducing network overhead.
    - **Transparency**: It requires the preservation of the original remote Node URI to ensure the ROS peer-to-peer (P2P) model is maintained.

## 3. Oracle Testcases & Expected Outcomes

### Test 1: XML-RPC Success Wrapper Check (`test_monitor_uses_succeed_helper`)
* **Strategy**: Static pattern matching for the call to `self._succeed()`.
* **Expected Outcome**: The code must pass all raw XML-RPC returns through the helper. Failure to do so indicates a violation of the system's error-handling contract.

### Test 2: Local Topology Mapping (`test_monitor_populates_master_info`)
* **Strategy**: Regex validation for the specific loop structure `for topic, nodes in publishers:`.
* **Expected Outcome**: The model must use the exact variable names and structure defined in the TODO requirements to ensure integration with the state analysis tools.

### Test 3: Multi-Master Loop Prevention (`test_sync_loop_prevention`)
* **Strategy**: Identification of conditional logic comparing remote node names against `rospy.get_name()` or `self.ros_node_name`.
* **Expected Outcome**: Presence of a "Guard Clause" that skips synchronization if the node is local. This prevents a "topic storm" where masters continuously re-sync their own mirrored topics.

### Test 4: Batched Registration (`test_sync_uses_multicall`)
* **Strategy**: Searching for the `own_master_multi()` method call.
* **Expected Outcome**: The synchronization must be batched. Individual `registerPublisher` calls are considered a failure in system-level performance requirements.

### Test 5: P2P Transparency (`test_sync_preserves_remote_uri`)
* **Strategy**: Verification that the `registerPublisher` arguments include the remote Node's URI.
* **Expected Outcome**: The local Master should point directly to the remote IP for the topic. If the bridge's own URI is used instead, the test fails as it breaks the ROS distributed architecture.

### Test 6: Hardcoded Name Absence (`test_absence_of_hardcoded_names`)
* **Strategy**: Negative matching for strings like `'/master_discovery'` or `'/master_sync'`.
* **Expected Outcome**: The code must use dynamic name retrieval. Hardcoded strings will cause failures in environments with custom namespaces or multiple instances.
