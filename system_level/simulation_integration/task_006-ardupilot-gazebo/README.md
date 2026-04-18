# Benchmark Task: ArduPilot Gazebo Plugin Logic Synchronization

## 1. Brief Description
This task evaluates a developer's ability to handle **asynchronous communication synchronization** and **coordinate transformations** between a complex robotics simulation system (Gazebo Sim) and a flight control stack (ArduPilot). 

The developer is required to refine the plugin logic to support ArduPilot's lock-step simulation, ensure correct PWM normalization, and implement rigorous coordinate frame transformations (converting from Gazebo's ENU convention to ArduPilot's NED convention). The core challenge involves managing redundant control protocols (Binary UDP vs. JSON) and ensuring robust handshake logic during the simulation's initialization phase.
---
source code file:
```https://github.com/ArduPilot/ardupilot_gazebo/blob/main```

## 2. Design Reasoning for Holes (Fill-in-the-blanks)

* **Hole 1: `receivedFirstCmd` State Assignment**
    * **Reasoning**: The essence of lock-step simulation is "sim-waits-for-autopilot." However, the plugin must send the first state packet to initiate the handshake. This hole tests whether the developer understands that the simulation should only block *after* the initial connection is confirmed via the first received command.
* **Hole 2: PWM-to-Command Normalization Formula**
    * **Reasoning**: ArduPilot plugins follow a strict mathematical model: `cmd = (normalized_pwm + offset) * multiplier`. Developers often confuse the order of operations between the multiplier and the offset. This hole verifies the precision of the signal processing chain.
* **Hole 3: Socket Address Initialization**
    * **Reasoning**: This validates the "Configuration over Hardcoding" principle. By removing the default IP initialization, the developer is forced to retrieve the `fdm_addr` from SDF elements, ensuring network environment compatibility.
* **Hole 4: Coordinate Transformation Composition**
    * **Reasoning**: Transforming from Gazebo to ArduPilot requires a chain of `wldAToWldG`, `wldGToBdyG`, and `bdyAToBdyG`. This hole tests the developer's ability to derive complex 3D pose chains correctly.

## 3. Oracle Testcases & Expected Outcomes

### Testcase 1: `test_lockstep_logic`
* **Design Goal**: Verify the startup sequence under lock-step mode.
* **Expected Outcome**: 
    1. The simulation should not hang indefinitely before the first ArduPilot packet is received; it must send an initial state.
    2. `receivedFirstCmd` must be set to `true` upon the first successful packet reception.
    3. Post-handshake, the simulation must correctly block if no new packet is received.

### Testcase 2: `test_pwm_normalization_logic`
* **Design Goal**: Ensure actuators (motors/servos) receive physically accurate commands.
* **Expected Outcome**: The source code must match the specific regex pattern. The output `cmd` must be correctly scaled by the `multiplier` *after* applying the `offset` to the `[0, 1]` normalized value.

### Testcase 3: `test_no_hardcoded_addresses`
* **Design Goal**: Check for parameterization to prevent failure in containerized or CI/CD environments.
* **Expected Outcome**: No instances of `"127.0.0.1"` should exist in the `ArduPilotPluginPrivate` constructor or member initializers. All addresses must be sourced via `_sdf->Get<std::string>("fdm_addr", ...)`.

### Testcase 4: `test_pose_transform_consistency`
* **Design Goal**: Autopilots are extremely sensitive to orientation; a single axis flip leads to a crash.
* **Expected Outcome**: Verify that `wldAToBdyA` is derived using the correct `Inverse()` composition, ensuring the pose is accurately reflected in the FRD (Front-Right-Down) frame.
