# Task 009 – Fleet-Level Multinode Integration (Robofleet)

## Source
https://github.com/ut-amrl/robofleet

This task is based on the Robofleet system, which enables distributed coordination
across multiple ROS robots via a fleet-level communication layer.

---

## Files with Removed Logic

### 1. Fleet Subscription Management (Server Side)

**Source directory**
```robofleet_server/src/subscriptions.ts```

**Removed logic**
- Subscription state update and removal
- Subscription matching logic for topic-based message routing

**Expected outcome**
- Correctly track topic subscription patterns per connected client
- Support dynamic subscribe and unsubscribe actions
- Determine whether a message on a given topic should be forwarded
  to a specific client in a multi-robot fleet

---

### 2. Robot Client Multinode Integration (Client Side)

**Source directory**

```robofleet_client/robofleet_client/RosClientNode.hpp```

**Removed logic**
- Local ROS topic registration and encoding
- Remote message decoding and local publishing
- Remote subscription request emission

**Expected outcome**
- Integrate multiple local ROS topics into a single client node
- Bridge local and remote topics without feedback loops
- Correctly manage bidirectional message flow between
  robot-local nodes and fleet-level communication

