# Task 003: TurtleBot3 Multi-Variant System Integration

## 1. Brief Description
This task challenges developers to complete the physical robot description (URDF) for two distinct TurtleBot3 variants (**Burger** and **Waffle**) and integrate them into a modern **ROS 2 + Gazebo Sim (Ignition)** simulation environment. The task focuses on maintaining physical consistency between robot models and correctly utilizing the `ros_gz_sim` Launch API, which has significantly different parameter structures compared to the legacy `gazebo_ros` (Classic) implementation.

---

source file:
```https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/main/turtlebot3_gazebo```

## 2. Hollowing Logic

### A. URDF Variant Differentiation
* **Burger `base_link` (Cylindrical vs. Box):** The `base_link` of the Burger model is hollowed out. Developers must provide the correct mesh path (`burger_base.stl`) and, crucially, the correct `collision` box dimensions (approx. $0.14 \times 0.14 \times 0.143\text{m}$) and `mass` ($0.825\text{kg}$). A common mistake is "hallucinating" the larger Waffle parameters for the smaller Burger chassis.
* **Waffle `caster_back_left` (Dual Caster Setup):** Unlike the single-caster Burger, the Waffle uses a dual-caster setup. The left caster link and joint are hollowed out. This tests if the developer understands the structural asymmetry and specific component naming conventions (`caster_back_left_link`).

### B. Launch System Coupling
* **Modern Gazebo API (`ros_gz_sim`):**
    The `gzserver` and `gzclient` include blocks are hollowed. 
    * **Old Hallucination:** Using `gzserver.launch.py` (deprecated/wrong package).
    * **Correct Logic:** Using `gz_sim.launch.py` with specific `gz_args`.
* **Resource Path Mapping:** The environment variable setup for `GZ_SIM_RESOURCE_PATH` is a critical coupling point. Without this, Gazebo cannot locate the `.stl` or `.dae` meshes defined in the URDFs.

---

## 3. Oracle Testcase Design

| Testcase | Concept Validated | Expected Outcome (Pass Criteria) |
| :--- | :--- | :--- |
| `test_burger_base_link_mesh` | Variant-specific mesh usage. | Must match `burger_base.stl`. Fails if it defaults to `waffle` or generic boxes. |
| `test_burger_collision_geometry` | Physics/Collision accuracy. | Must contain `<box size="0.14 0.14 0.143"/>` (allowing minor float variance). |
| `test_burger_inertial_properties` | Dynamic fidelity. | Mass must be approximately `0.825kg` (scientific notation: `8.257...e-01`). |
| `test_waffle_dual_caster_structure` | Structural completeness. | Strings `caster_back_right_link` AND `caster_back_left_link` must both exist. |
| `test_launch_uses_modern_gz_sim` | ROS 2 Migration compliance. | Must use `ros_gz_sim` package and reference `gz_sim.launch.py`. |
| `test_launch_server_client_separation` | Execution CLI arguments. | Server command must pass `'-r -s'` (run + server) flags within `gz_args`. |
| `test_launch_env_resource_path` | Resource discovery. | Must use `AppendEnvironmentVariable` to map the `models` directory. |
| `test_no_legacy_gazebo_nodes` | Anti-pattern detection. | String `gazebo_ros` must NOT appear (prevents mixing old/new simulation logic). |

---

