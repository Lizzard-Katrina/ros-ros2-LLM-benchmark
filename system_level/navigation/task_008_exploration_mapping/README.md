# Task 008: Unknown Environment Exploration + Mapping

## Level
Integration Level Correctness

## Description
This task evaluates whether an LLM can correctly reconstruct an
integration-level exploration and navigation behavior in ROS.

The robot must:
1. Process laser scan data
2. Identify frontier points
3. Send navigation goals via move_base
4. Operate in an unknown environment using SLAM

## Source
Based on the open-source project:
bandasaikrishna/Autonomous_Mobile_Robot  
File: navigation/scripts/explore.py

## TODO Description

### scan_callback()
Expected outcome:
- Correct extraction of frontier points from LaserScan
- Valid Cartesian conversion
- Meaningful exploration candidates

### send_goal()
Expected outcome:
- Proper MoveBaseGoal construction
- Goal sent in map frame
- Navigation stack receives valid goals

## Validation

Single TODO validation:
- Frontier points are generated from scan
- Goals are published and accepted

Full pipeline validation:
- Run SLAM (gmapping)
- Robot explores unknown map
- Navigation goals update dynamically
- Robot avoids obstacles and expands map
