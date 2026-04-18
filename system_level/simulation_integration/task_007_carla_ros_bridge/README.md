# Task: CARLA ROS Bridge Migration - Synchronous Mode & Transforms

## 1. Brief Description
This task involves migrating the core synchronization logic and coordinate transformation utilities from a CARLA-ROS 1 environment to a ROS 2 compatible structure using the `ros_compatibility` framework. The goal is to ensure that the bridge maintains a strict lockstep with the simulator and correctly maps physics data between different coordinate conventions.

---
source code file:
```https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge```

## 2. Abstraction Strategy (Hole-Punching Logic)
The holes are strategically placed to test architectural understanding rather than syntax:

* **Logic Coupling (`transforms.py`)**: The entire body of `carla_velocity_to_ros_twist` is removed. The developer must correctly combine linear and angular transformations, ensuring they re-use existing helper functions like `carla_vector_to_ros_vector_rotated` to maintain system-wide consistency.
* **Sequential Barrier (`bridge.py`)**: The `_synchronous_mode_update` loop is hollowed out. This requires the developer to implement the "Update-Tick-Broadcast" sequence in the specific order required for temporal alignment in synchronous simulation.

## 3. Testcase Design & Expected Outcomes

| Test Case | Design Logic / Concept | Expected Outcome |
| :--- | :--- | :--- |
| `test_twist_linear_rotation_logic` | **Functional Re-use**: Checks if the rotation helper is called. | Presence of `carla_vector_to_ros_vector_rotated`. |
| `test_twist_angular_unit_conversion` | **Physics Conversion**: Validates degrees to radians. | Usage of `math.radians()` for all axes. |
| `test_twist_angular_handedness_inversion` | **Coordinate Handedness**: Checks LH (Carla) to RH (ROS) mapping. | Negative signs (`-`) on Angular Y and Z components. |
| `test_bridge_sync_tick_order` | **Temporal Sequence**: tick() must precede snapshot processing. | `carla_world.tick()` index < `get_snapshot()` index. |
| `test_bridge_clock_synchronization_call` | **Clock Coupling**: Ensures the ROS clock is driven by Carla. | Call to `self.update_clock()` with snapshot timestamp. |
| `test_actor_factory_pre_tick_update` | **Lifecycle Sync**: Factory must update before physics step. | `actor_factory.update_available_objects()` called before `tick()`. |
| `test_no_hardcoded_ego_id` | **Decoupling**: Validates dynamic actor identification. | No hardcoded integer lists for ego IDs. |
