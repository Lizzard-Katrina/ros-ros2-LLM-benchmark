# Oracle Test: ROS1 Nodelet → ROS2 Interface-Level Translation

## Resource reference:

github: ```https://github.com/ros/nodelet_core/blob/noetic-devel/test_nodelet/src/plus.cpp```

used document: test_nodelet/src/plus.cpp

This document contains the full circuit of publisher_server.

## Task Goal

This oracle test evaluates whether an LLM can correctly translate **ROS1 nodelet-based publisher/subscriber code** into **ROS2-compliant node or component code**, focusing strictly on the **interface level** rather than runtime behavior.

The test is designed to validate migration correctness for:
- Publisher / Subscriber APIs
- Parameter handling
- Message types
- Node / component architecture
- Namespace semantics

No compilation or execution is required.

---

## What This Oracle Tests

The oracle validates **semantic concepts**, not exact syntax.

### Core Migration Concepts

1. **ROS2 API Usage**
   - Uses `rclcpp` headers
   - Does NOT use `ros/ros.h`, `nodelet`, or `PLUGINLIB_EXPORT_CLASS`

2. **Nodelet → ROS2 Architecture**
   - Nodelet class is migrated to a `rclcpp::Node` or ROS2 component

3. **Parameter System Migration**
   - `getParam()` → `declare_parameter()` + `get_parameter()`

4. **Publisher Interface**
   - `advertise()` → `create_publisher()`

5. **Subscriber Interface**
   - `subscribe()` → `create_subscription()`

6. **Message Type Migration**
   - `std_msgs::Float64` → `std_msgs::msg::Float64`
   - Same for `Bool`, `Byte`, `Time`

7. **Removal of ROS1 Artifacts**
   - No `ROS_INFO`, `NODELET_DEBUG`
   - No `boost::shared_ptr`

8. **Executable Semantics**
   - Either `rclcpp::init + rclcpp::spin`
   - OR ROS2 component registration macro

9. **Interface-Level Topic Semantics**
   - Topic names such as `/global`, `namespaced`, `private`, `in`, `out` must be preserved

---

## Design Philosophy

- ✅ Pattern matching only (regex + string search)
- ✅ No compilation, no runtime execution
- ✅ Each test checks exactly one concept
- ✅ Tests are independent
- ❌ No exact formatting or variable-name assumptions
- ❌ No dependence on line numbers or indentation

This ensures:
- Fast execution (<1s)
- Robustness against stylistic variation
- Fair evaluation of LLM translation capability
