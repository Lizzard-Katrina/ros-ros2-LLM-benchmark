## Task 006: Multi-topic Synchronization (Stereo Image)

### reference source:
```https://github.com/ros/ros_comm```


## Overview

This task requires students to translate ROS1 stereo synchronization code to ROS2, maintaining the same functional structure while adapting to ROS2's API changes.

**Input:** ROS1 code template (with TODO sections)  
**Task:** Fill in TODO sections and translate ROS1 → ROS2 APIs  
**Validation:** 7 Oracle tests check key translation milestones

---

## Design Philosophy: Why Static Code Analysis?

### Problem We're Solving
- **Avoid Runtime Dependencies:** No need to spin up ROS2 master, middleware, or DDS infrastructure
- **Fast Feedback:** Tests run in <1 second (vs 30+ seconds with launch tests)
- **Deterministic Results:** No flaky network/timing issues
- **Easy Debugging:** Students can run tests locally without Docker
- **Scalability:** Tests work on any machine with Python

### How We Avoid C++ Code Conflicts
1. **Target Semantic Patterns, Not Exact Syntax**
   - Use regex patterns that match common C++ idioms
   - Allow whitespace/formatting flexibility
   - Test for presence of required concepts, not exact implementation

2. **No Compilation Required**
   - Read source file directly as text
   - Check for includes, function signatures, class definitions
   - Avoid false positives from comments (mostly)

3. **Layered Validation**
   - Oracle 1-7 test different aspects
   - Passing all 7 means the translation is structurally complete
   - Each oracle is independent (one failure doesn't break others)

---

## Oracle Tests: What and Why

### Oracle 1: ROS2 Headers (NOT ROS1)
**Checks:** 
- ❌ Should NOT contain `#include <ros/ros.h>`
- ✅ Should contain `#include <rclcpp/...>`

**Why this design:**
- Most obvious indicator of ROS1 vs ROS2
- Regex: Simple string search (no false positives)
- **Expected outcome:** `assert '#include <ros/ros.h>' not in source`

**How to pass:**
```cpp
// ❌ WRONG
#include <ros/ros.h>

// ✅ RIGHT
#include <rclcpp/rclcpp.hpp>
```

---

### Oracle 2: StereoSync Class Structure
**Checks:**
- StereoSync class exists (any scope style)
- syncCallback method exists

**Why this design:**
- Ensures core class structure is preserved
- Checks that callback is named correctly (interface contract)
- Regex allows flexibility in whitespace: `class\s+StereoSync\s*\{`
- **Expected outcome:** Class with public method `void syncCallback(...)`

**How to pass:**
```cpp
class StereoSync {
public:
    void syncCallback(
        const sensor_msgs::msg::Image::SharedPtr& left,
        const sensor_msgs::msg::Image::SharedPtr& right) {
        // Implementation
    }
};
```

**Why NOT a regex like `syncCallback\s*\(.*const.*Image.*const.*Image`:**
- Too fragile (breaks if student reformats parameters)
- Just checking for method existence is sufficient

---

### Oracle 3: Message Filters with ApproximateTime
**Checks:**
- `#include <message_filters/...>`  exists
- `message_filters::Synchronizer` used with `ApproximateTime` policy

**Why this design:**
- Core functionality: synchronizing messages from multiple topics
- ApproximateTime is the specified synchronization policy
- Regex pattern: `message_filters::Synchronizer<\s*message_filters::sync_policies::ApproximateTime`
  - Allows whitespace/newlines between `<` and policy name
  - Matches the template syntax precisely
- **Expected outcome:** 
```cpp
message_filters::Synchronizer<
    message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, 
        sensor_msgs::msg::Image
    >
>
```

**Why NOT check the template parameters?**
- Student might write `ApproximateTime<Image, Image>` vs full qualified names
- Checking for "ApproximateTime" is enough to validate the policy choice

---

### Oracle 4: Two Image Subscribers
**Checks:**
- At least 2 instances of `message_filters::Subscriber<sensor_msgs::Image>`
- Member variables `left_sub_` and `right_sub_` exist

**Why this design:**
- Validates that student understands multi-topic subscription pattern
- Checks for correct message type (Image)
- Member variable names show understanding of semantic roles
- Regex: `message_filters::Subscriber<sensor_msgs::Image>` (exact, no false positives)
- **Expected outcome:** 
```cpp
private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
```

**Why count occurrences?**
- If only 1 subscriber exists, synchronization is impossible
- Finding exactly 2 proves multi-topic handling

---

### Oracle 5: ROS2 Node Creation
**Checks:**
- ❌ Should NOT use `ros::NodeHandle` (ROS1)
- ❌ Should NOT use `ros::init()` (ROS1)
- ✅ Should use `rclcpp::Node` or `std::make_shared<rclcpp::Node>`

**Why this design:**
- Key API difference between ROS1 and ROS2
- NodeHandle is ROS1 specific; Node is ROS2
- Regex: `(?:rclcpp::Node|make_shared.*Node|std::make_shared)`
  - Non-capturing group for flexibility
  - Matches various constructor styles
- **Expected outcome:**
```cpp
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("stereo_sync_node");
    StereoSync sync(node);
    rclcpp::spin(node);
    return 0;
}
```

**Why allow different constructor styles?**
- `std::make_shared<rclcpp::Node>`
- `rclcpp::create_node("name")`
- Both are valid ROS2 patterns

---

### Oracle 6: Proper main() Function
**Checks:**
- `int main()` or `int main(int argc, char* argv[])` signature
- Uses `rclcpp::init()` and `rclcpp::spin()` (not `ros::init`)

**Why this design:**
- Ensures program is executable
- Validates ROS2 node lifecycle management
- `rclcpp::spin()` blocks and processes messages (core ROS2 pattern)
- Regex: `int\s+main\s*\(\s*(?:int\s+argc\s*,\s*char\s*\*.*argv|)\)`
  - Flexible parameter list
  - Allows `char *argv[]` or `char **argv`
- **Expected outcome:**
```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // Create and use StereoSync
    rclcpp::spin(...);
    rclcpp::shutdown();
    return 0;
}
```

---

### Oracle 7: No ROS1 Remnants
**Checks:**
- ❌ No `ros::NodeHandle`
- ❌ No `ros::Subscriber` 
- ❌ No `boost::shared_ptr` (ROS2 uses `std::shared_ptr`)
- ❌ No `ROS_INFO` (ROS2 uses `RCLCPP_INFO`)

**Why this design:**
- Ensures complete migration (not hybrid ROS1/ROS2 code)
- Catches common mistakes students make
- Warnings instead of failures (some might be in comments/disabled)
- **Expected outcome:** Zero ROS1 API calls in active code

**Why separate from other tests?**
- Allows partial credit if other oracles pass
- Encourages clean code even if functional

---

## Test Execution Flow

```
stereo_sync.cpp
    ↓
Read source code as text
    ↓
Run 7 oracle regex/string checks in parallel
    ↓
✅ All 7 pass → Student successfully translated ROS1→ROS2
⚠️  Some fail → Specific feedback on what's missing
```

---

## Expected Outcomes by Scenario

### Scenario A: Perfect Translation
```
✅ Oracle 1: Using ROS2 headers (rclcpp)
✅ Oracle 2: StereoSync class with syncCallback exists
✅ Oracle 3: Using message_filters::Synchronizer with ApproximateTime
✅ Oracle 4: Two Image subscribers (left_sub_, right_sub_) exist
✅ Oracle 5: Using ROS2 node creation (not ros::init)
✅ Oracle 6: main() properly initializes and spins ROS2 node
✅ Oracle 7: Minimal ROS1 remnants (or none)

Result: ✅ 7/7 PASSED - Translation complete
```

### Scenario B: Incomplete Translation
```
✅ Oracle 1: Using ROS2 headers (rclcpp)
❌ Oracle 2: StereoSync class not found
❌ Oracle 3: Missing Synchronizer with ApproximateTime
❌ Oracle 4: Missing two Image subscribers
✅ Oracle 5: Using ROS2 node creation (not ros::init)
✅ Oracle 6: main() properly initializes and spins ROS2 node
⚠️  Oracle 7: Found ROS1 patterns: [ROS1 NodeHandle]

Result: ❌ 3/7 FAILED - Class structure needs work
Feedback: "Focus on implementing StereoSync class with proper subscribers"
```

---

## Why This Design is Robust

| Aspect | How Handled |
|--------|------------|
| **Whitespace variation** | Regex `\s+` allows any whitespace |
| **Line breaks** | No reliance on single-line patterns |
| **Comment noise** | Simple string/pattern matching ignores context |
| **Formatting styles** | Multiple regex alternatives (non-capturing groups) |
| **Code completeness** | Each oracle tests one concept independently |
| **Failure clarity** | Error messages tell student exactly what's missing |

---

## How to Adapt Oracles for Your Codebase

If tests fail due to syntax differences:

1. **Enable debug mode:**
   ```python
   print("Source code length:", len(cls.source_code))
   print("First 500 chars:", cls.source_code[:500])
   ```

2. **Adjust regex patterns:**
   - If `class StereoSync` uses a different style, update the pattern
   - Example: `struct StereoSync` needs pattern `(?:class|struct)\s+StereoSync`

3. **Test locally:**
   ```bash
   python3 -m pytest test_oracle_ros2.py -v -s
   ```

---

## Summary

**This oracle test suite validates:**
- ✅ ROS1 → ROS2 API migration
- ✅ Multi-topic synchronization implementation
- ✅ Correct use of message_filters
- ✅ Proper ROS2 node lifecycle
- ✅ Code cleanliness (no hybrid ROS1/2)

**It avoids:**
- ❌ Runtime dependencies (no need for ROS master)
- ❌ Flaky timing/network issues
- ❌ Complex launch configurations
- ❌ Compilation failures breaking tests
