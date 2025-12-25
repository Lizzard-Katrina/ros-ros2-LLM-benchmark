# 007 – AWS Lex Integration (ROS1)

## Task Description
This task evaluates integration-level reasoning in a ROS1 system that bridges
robot-side communication with a cloud-based natural language service (AWS Lex).

A ROS service node receives audio/text requests, forwards them to AWS Lex,
and returns structured intent and slot information to ROS clients.

## Source
https://github.com/aws-robotics/lex-ros1

## Files with Blanks

### 1. lex_node/src/lex_node.cpp
**Removed Logic:**
- Complete ROS service handling loop for Lex conversations

**Expected Outcome:**
- Correct translation between ROS requests and AWS Lex requests
- Proper handling of Lex responses and failure cases
- End-to-end semantic integrity of the request-response cycle

---

### 2. lex_node/include/lex_node/lex_node.h
**Removed Logic:**
- Semantic contract defining the role of the LexNode
- Intent and slot interpretation responsibilities

**Expected Outcome:**
- Clear definition of node responsibilities in the ROS graph
- Well-defined semantic contract for Lex request handling
- Extensible design for future intents and commands

## Evaluation Focus
- Multi-system integration reasoning
- Cloud service interaction via ROS
- Semantic consistency across system boundaries
- Robust handling of asynchronous request-response workflows
