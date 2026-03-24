# Task 011: Translate PR2 Doors ROS1 Demo to ROS2

## Brief Description
This task focuses on translating a ROS1 door-handling demo to ROS2 while preserving the key action client semantics.  
The goal is to ensure that the ROS2 code correctly:

- Creates ActionClients for door manipulation (`move_through_door`) and base navigation (`move_base_local`)  
- Waits for the action servers before sending goals  
- Initializes goal messages (`DoorGoal` and `MoveBaseGoal`)  
- Subscribes to `/test_output` for logging or testing feedback  

The translation is verified **statically** using pattern-matching oracle tests (regex + string search) to check semantic equivalence, not runtime correctness.

## Where to Insert TODO / Blank
The benchmark uses a "fill-in-the-blank" approach to test LLM translation:

- **Target section to blank:**  
  The initialization of ROS2 ActionClients and sending goals, including:
  ```python
  self.ac_door = SimpleActionClient('move_through_door', DoorAction)
  self.ac_door.wait_for_server()
  self.ac_move = SimpleActionClient('move_base_local', MoveBaseAction)
  self.ac_move.wait_for_server()
  ```

### Test Case1: Door ActionClient Created

**Concept**
Check that an ActionClient for the door (`move_through_door`) action exists in ROS2.

** Semantic**
- The code must initialize a `SimpleActionClient` (or ROS2 equivalent) for `DoorAction`.
- Ensures LLM correctly translates ROS1 ActionClient semantics to ROS2.

** ROS1 Reference**
```python
self.ac_door = SimpleActionClient('move_through_door', DoorAction)
```

**Expected Outcome **
Regex matches ActionClient('move_through_door', DoorAction)

Failure message: "Door ActionClient creation missing"


---

### Test Case2: MoveBase ActionClient Created

**Concept**  
Check that an ActionClient for the move base (`move_base_local`) action exists in ROS2.

**Semantic**  
- Ensures a navigation client exists for sending MoveBase goals.  
- Validates correct ROS2 translation of ROS1 `SimpleActionClient`.

**ROS1 Reference**  
```python
self.ac_move = SimpleActionClient('move_base_local', MoveBaseAction)
```

**expected outcome**
Regex matches ActionClient('move_base_local', MoveBaseAction)

Failure message: "MoveBase ActionClient creation missing"


---

### Test Case3: Wait for Server Called

**Concept**  
Verify that the code waits for both action servers before sending goals.


**Semantic**  
- `.wait_for_server()` must be called for `ac_door` and `ac_move`.  
- Ensures the ROS2 client properly waits before sending goals.

**ROS1 Reference**  
```python
self.ac_door.wait_for_server()
self.ac_move.wait_for_server()
```

** Expected Outcome**

Regex finds two calls to .wait_for_server()

Failure message: "wait_for_server() not called for both action clients"



---

### Test Case: Subscriber Exists

**Concept**  
Check that a ROS2 subscriber exists for logging or testing output (`/test_output`).

**Semantic**  
- The code must subscribe to `/test_output` topic to receive test/log messages.  
- Ensures semantic match for ROS2 subscriber translation from ROS1.

**ROS1 Reference**  
```python
rospy.Subscriber("/test_output", String, self.stringOutput)
```

**Expected Outcome**
Regex matches a Subscriber('/test_output', ...) call

Failure message: "Subscriber to /test_output missing"

---

### Test Case: Goal Initialization

**Concept**  
Verify that `DoorGoal` and `MoveBaseGoal` messages are initialized.

**Semantic**  
- The code must create goal messages for door and navigation actions.  
- Ensures semantic correctness for goal initialization in ROS2.

**ROS1 Reference**  
```python
self.door = DoorGoal()
self.move = MoveBaseGoal()
```

**Expected Outcome **
Regex matches DoorGoal() and MoveBaseGoal()

Failure message: "DoorGoal or MoveBaseGoal not initialized"


---

### Test Case: Door and Move Commands Sent

**Concept**  
Ensure the action goals are sent using `send_goal_and_wait` semantics.

**Semantic**  
- The code should send both door and move goals using ROS2 equivalent of `send_goal_and_wait`.  
- Validates the action execution pipeline in ROS2.

**ROS1 Reference**  
```python
self.ac_door.send_goal_and_wait(self.door, rospy.Duration(TEST_DURATION), rospy.Duration(5.0))
self.ac_move.send_goal_and_wait(self.move, rospy.Duration(TEST_DURATION), rospy.Duration(5.0))
```

** Expected Outcome **

Regex finds .send_goal_and_wait calls for both clients

Failure message: "Action goals not sent correctly"
