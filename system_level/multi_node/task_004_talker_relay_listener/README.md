# Task 004: Multinode Talker–Relay–Listener (ROS1)

## Description
This task evaluates integration-level correctness in a ROS1 multi-node system.

Three ROS nodes communicate through a chained topic pipeline:

talker → relay → listener

The relay node must correctly subscribe to an upstream topic and republish the message downstream without modification.

## Source
https://github.com/ros/ros_tutorials/tree/noetic-devel/roscpp_tutorials

## Files with Missing Logic

### ros1_code/relay.cpp

Missing implementations include:
- NodeHandle initialization
- Subscriber creation
- Publisher creation
- Message forwarding callback
- Node spinning logic

## Expected Outcome
When all nodes are running:
- Messages published by `talker` are received by `listener`
- The `relay` node transparently forwards messages
- No message type, topic name, or payload is altered
