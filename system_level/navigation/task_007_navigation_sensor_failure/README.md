# Task 007: Navigation Failure Recovery

## Level
Integration Level Correctness

## Source
ros-planning/navigation  
File: move_base/src/move_base.cpp

## Description
This task evaluates whether an LLM can correctly restore integration-level
navigation behavior when controller failures occur.

The task focuses on:
- Detecting controller failure
- Transitioning to recovery mode
- Executing recovery behaviors in correct order
- Deciding when to abort navigation

## TODO Description

### executeCycle() – CONTROLLING state
Expected Outcome:
- Detect controller failure
- Switch to CLEARING state
- Reset recovery index

### executeCycle() – CLEARING state
Expected Outcome:
- Execute recovery behaviors sequentially
- Abort navigation when all recoveries fail

## Validation
- Navigation should recover from temporary failures
- System should not deadlock or skip recovery steps
