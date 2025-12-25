# 006 – Swarm ROS Bridge (Integration-Level Multinode)

## Task Overview

This task evaluates integration-level correctness for a distributed multi-robot
communication bridge based on ROS and ZeroMQ.

Each robot runs its own ROS master. A bridge node forwards selected ROS topics
through a ZeroMQ PUB/SUB network to enable swarm-level coordination.

The implementation must correctly handle:
- Multiple ROS topics
- Multiple robots and namespaces
- Serialization and deserialization
- Concurrent send/receive pipelines

---

## Source

- Repository: https://github.com/shupx/swarm_ros_bridge
- ROS Version: ROS 1
- Communication Middleware: ZeroMQ (zmqpp)

---

## Scaffolded Files and Expected Outcomes

### 1. `src/bridge_node.cpp`

**Scaffolded Logic**
- ROS subscriber callbacks
- Message frequency control
- ROS message serialization
- ZeroMQ send/receive logic
- Deserialization and ROS publishing
- Receive thread management

**Expected Outcome**
- Messages published on configured ROS topics are forwarded through ZeroMQ
- Incoming ZeroMQ messages are deserialized and republished to ROS topics
- No topic mismatch across robots or namespaces
- Stable behavior under multiple topics and concurrent threads

---

### 2. `include/ros_sub_pub.hpp`

**Scaffolded Logic**
- ROS message type dispatch based on string identifiers
- Subscriber and publisher creation
- Type-safe deserialization dispatch

**Expected Outcome**
- Correct ROS subscriber and publisher creation for each configured topic
- Message types match configuration strings exactly
- Send-side and receive-side message handling remains consistent

---

## Evaluation Focus

This task focuses on:
- Multinode integration correctness
- Distributed ROS communication
- Middleware bridging (ROS ↔ ZeroMQ)
- End-to-end data flow consistency

