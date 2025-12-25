# README – 005 FKIE Multi-Agent Daemon Benchmark

**Task ID:** 005  
**Task Name:** FKIE Multi-Agent Daemon ROS1  
**Source:** https://github.com/fkie/fkie-multi-agent-suite  
**Description:** Multi-agent orchestration suite for ROS1 multi-master: daemon per host → node discovery → synchronization → cross-host coordination. This benchmark tests node initialization, monitoring, shutdown, and diagnostic functionalities.  
**Difficulty Level:** Hard  

---

## Source Code Directory

| File | Description |
|------|-------------|
| `fkie_mas_daemon/ros_node.py` | Core ROS node startup and server registration logic |
| `fkie_mas_daemon/monitor.py` | Monitoring services including diagnostics, warning updates, and node shutdown |

---

## Code to Implement (挖空部分)

| File | Function / Method | TODO Description | Expected Outcome |
|------|-----------------|-----------------|-----------------|
| `ros_node.py` | `__init__` | Initialize ROS node, parse arguments, create server, set logging, and set `success_start`. | - Node successfully initialized with parsed arguments.<br>- `ros_node` object created with correct namespace and logging.<br>- Server object initialized and ready for communication.<br>- `success_start` reflects node startup success.<br>- No exceptions during startup. |
| `monitor.py` | `rosShutdown` | Properly terminate all child processes and ROS nodes. | - All child processes managed by the monitor are terminated.<br>- Cleanup hooks or websocket notifications executed.<br>- No lingering processes remain.<br>- Returns JSON `{"result": True/False, "message": ...}`. |
| `monitor.py` | `update_warning_groups` | Update local warning groups and publish new warnings to websocket. | - Local warning groups updated correctly.<br>- New warnings added; removed warnings within lifetime preserved.<br>- Websocket publishes updated warnings.<br>- No duplicate or stale warnings remain. |
| `monitor.py` | `getDiagnostics` | Collect diagnostics from local monitor and system, convert to JSON, and publish. | - Collects diagnostics from all monitored nodes and system components.<br>- Converts to structured JSON format.<br>- Returns diagnostics ready for websocket publishing.<br>- No exceptions during collection. |

---

## Benchmark Expected Outcome Summary

The benchmark evaluates:

1. **Node Startup & Registration** (`ros_node.py::__init__`) – Node starts correctly with all arguments, logging, and server initialized.  
2. **Node Shutdown** (`monitor.py::rosShutdown`) – All child processes and monitored nodes terminated cleanly, JSON status returned.  
3. **Warning Group Update** (`monitor.py::update_warning_groups`) – Updates propagated to websocket; warnings are correctly maintained.  
4. **Diagnostics Retrieval** (`monitor.py::getDiagnostics`) – Diagnostic messages collected, converted, and returned without error.
