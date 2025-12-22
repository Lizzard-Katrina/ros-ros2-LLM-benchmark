# Task 004 – Simplified Pick & Place  
**Integration-Level Manipulation Benchmark**

---

## 1. Task Scope and Purpose

This task evaluates an AI system’s ability to complete an **end-to-end robotic manipulation pipeline** in ROS1 by filling in missing logic inside an existing codebase.

The benchmark focuses on **integration correctness**, rather than algorithmic novelty or low-level control performance.

Specifically, the task requires coordinating:

- TF transformations  
- Inverse kinematics (IK) feasibility  
- Motion planning via MoveIt  
- Gripper control  
- Pick-and-place sequencing  

The reference implementation is adapted from:

> https://github.com/gstavrinos/ez_pick_and_place

---

## 2. Task Boundary

- **ROS version**: ROS1 Noetic  
- **Language**: Python  
- **Evaluation level**: Integration  
- **Simulation / visualization**: Not required for correctness  

Only one file contains missing logic:

ros1_code/src/ez_tools.py

The file `ez_pnp2.py` is provided **unchanged** and should be treated as a black-box orchestration layer.

---

## 3. Subtask Decomposition

The task is decomposed into **four subtasks**, each corresponding to a semantically meaningful stage in a manipulation pipeline.

Each subtask must satisfy **explicit functional conditions** described below.

---

## Subtask 004-1: Grasp Feasibility Filtering (`discard`)

### Objective

Filter out grasp poses that are **kinematically infeasible** for the robot arm.

### Required Operation

For each candidate grasp pose \( g_i \):

1. Transform the grasp pose into the robot base frame  
2. Call the inverse kinematics solver  
3. Discard the grasp if no valid joint configuration exists  

### Expected Outcome (Formal Condition)

Let:

- \( G = \{ g_1, g_2, \dots, g_n \} \) be the set of candidate grasps  
- \( IK(g_i) \) return a valid joint vector or fail  

Then the output set \( G' \) must satisfy:

\[
G' = \{ g_i \in G \mid IK(g_i) \text{ exists} \}
\]

### Evaluation Criterion

- All remaining grasps must admit at least one valid IK solution  
- No infeasible grasp is passed to downstream planning  

---

## Subtask 004-2: Pick Execution (`pick`)


#### Objective

Ensure the gripper is in a valid open state before approaching the object.

#### Expected Outcome

The gripper joint values satisfy:

\[
q_{\text{gripper}} = q_{\text{open}}
\]

prior to any grasp motion.


#### Objective

Refine grasp candidates by removing those that fail during planning or execution checks.

#### Expected Outcome

Let \( G' \) be the output of Subtask 004-1.  
The selected grasp \( g^* \) must satisfy:

\[
g^* \in G' \quad \text{and} \quad \text{planning}(g^*) = \text{success}
\]


#### Objective

Move the robot end-effector to the grasp pose and attach the object.

#### Expected Outcome

After execution:

- The robot end-effector pose equals the grasp pose within tolerance  
- The object is attached to the gripper link  

Formally:

\[
T_{\text{ee}} \approx T_{g^*}, \quad \text{object} \in \text{attached\_collision\_objects}
\]

---

## Subtask 004-3: Place Execution (`place`)

This subtask evaluates the **placement phase**, including pose computation, motion execution, and object release.


#### Objective

Compute a valid end-effector pose for placing the object.

#### Expected Outcome

There exists a placement pose \( p \) such that:

\[
IK(p) \text{ exists}
\]

and the pose is collision-free.

#### Objective

Move the robot arm to the placement pose.

#### Expected Outcome

The motion planner successfully computes and executes a trajectory:

\[
\text{execute}(\text{plan}(p)) = \text{success}
\]

#### Objective

Detach the object and open the gripper.

#### Expected Outcome

After execution:

\[
\text{object} \notin \text{attached\_collision\_objects}
\]

and

\[
q_{\text{gripper}} = q_{\text{open}}
\]


## 4. Overall Task Success Criteria

The task is considered **successfully solved** if:

1. All subtasks complete without runtime errors  
2. A valid object pick occurs  
3. A valid object placement occurs  
4. The system terminates in a consistent, non-attached state  

No requirements are imposed on:

- Trajectory optimality  
- Execution speed  
- Visual output  

---

## 5. Evaluation Philosophy

This benchmark evaluates:

- Cross-module reasoning  
- State consistency across subsystems  
- Correct handling of failure branches  
- Understanding of manipulation pipeline semantics  

It does **not** evaluate:

- Low-level control performance  
- Perception quality  
- Learning-based grasp synthesis  

---

## 6. Summary

Task 004 represents a realistic manipulation integration challenge, requiring the AI system to reason across:

\[
\text{TF} \rightarrow \text{IK} \rightarrow \text{Planning} \rightarrow \text{Execution}
\]

Correctness is defined by **functional completion**, not by implementation style.

