# Task 006 — Multi-topic Synchronization with message_filters

## Goal
Evaluate whether the model can translate a ROS1 multi-topic synchronization node (using message_filters) into a ROS2 equivalent structure.

## Description
Two sensor streams are published to:
- `/left/image`
- `/right/image`

A synchronization node must:
- subscribe to both topics
- use `message_filters::Synchronizer`
- register a sync callback handling matched pairs

## Requirements
You must complete the missing sections in:


The ROS2 version is optional but provided as:


## Notes
This task is medium-high difficulty because it requires:
- message_filters policies
- multi-topic callback patterns
- stereo/multi-sensor synchronization logic
