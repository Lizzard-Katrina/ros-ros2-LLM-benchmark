# ROS1 Parameter Server – Multinode Integration Task

## Task Description

This task evaluates **multinode integration correctness** of the ROS1 parameter
server by removing two core functions that jointly define parameter
write/read behavior across nodes.

Source: ROS Noetic (`ros_comm`)

---
## Modified Source Files

### 1️ Parameter Write Function

**Function (removed)**
```cpp
void set(const std::string& key, const XmlRpc::XmlRpcValue& v)
```
### Expected Outcome

A parameter set in one node becomes visible to other nodes.

The parameter exists on the ROS master.

Local parameter cache remains consistent.

Parent namespace cache entries are invalidated.
##2. Parameter Read Function

### File
```clients/roscpp/src/libros/param.cpp```
** Function (removed)**
bool getImpl(const std::string& key,
             XmlRpc::XmlRpcValue& v,
             bool use_cache)

### Expected Outcome

A parameter set in one node becomes visible to other nodes.

The parameter exists on the ROS master.

Local parameter cache remains consistent.

Parent namespace cache entries are invalidated.
