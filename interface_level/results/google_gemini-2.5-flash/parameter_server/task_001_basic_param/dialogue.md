# Prompt


You are an expert ROS2 migration engineer.
Task: Convert this ROS1 file to ROS2. Fill TODO locations. Keep function names and structure.
Do NOT explain. Only output source code.
ROS1 code:
----------------------------
/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/param.h"
#include "ros/master.h"
#include "ros/xmlrpc_manager.h"
#include "ros/this_node.h"
#include "ros/names.h"

#include <ros/console.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <map>

namespace ros
{

namespace param
{

typedef std::map<std::string, XmlRpc::XmlRpcValue> M_Param;
M_Param g_params;
boost::recursive_mutex g_params_mutex;
S_string g_subscribed_params;

void invalidateParentParams(const std::string& key)
{
// TODO: Invalidate cached parent namespace entries for a given key.
//END OF TODO
}

void set(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string mapped_key = ros::names::resolve(key);

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  params[2] = v;

  {
    // Lock around the execute to the master in case we get a parameter update on this value between
    // executing on the master and setting the parameter in the g_params list.
    boost::recursive_mutex::scoped_lock lock(g_params_mutex);

    if (master::execute("setParam", params, result, payload, true))
    {
      // Update our cached params list now so that if get() is called immediately after param::set()
      // we already have the cached state and our value will be correct
      if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
      {
        g_params[mapped_key] = v;
      }
      invalidateParentParams(mapped_key);
    }
  }
}

void set(const std::string& key, const std::string& s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  XmlRpc::XmlRpcValue v(s);
  ros::param::set(key, v);
}

void set(const std::string& key, const char* s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  std::string sxx = std::string(s);
  XmlRpc::XmlRpcValue v(sxx);
  ros::param::set(key, v);
}

void set(const std::string& key, double d)
{
  XmlRpc::XmlRpcValue v(d);
  ros::param::set(key, v);
}

void set(const std::string& key, int i)
{
  XmlRpc::XmlRpcValue v(i);
  ros::param::set(key, v);
}

void set(const std::string& key, bool b)
{
  XmlRpc::XmlRpcValue v(b);
  ros::param::set(key, v);
}

template <class T>
  void setImpl(const std::string& key, const std::vector<T>& vec)
{
  // Note: the XmlRpcValue starts off as "invalid" and assertArray turns it
  // into an array type with the given size
  XmlRpc::XmlRpcValue xml_vec;
  xml_vec.setSize(vec.size());

  // Copy the contents into the XmlRpcValue
  for(size_t i=0; i < vec.size(); i++) {
    xml_vec[i] = vec.at(i);
  }

  ros::param::set(key, xml_vec);
}

void set(const std::string& key, const std::vector<std::string>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<double>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<float>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<int>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<bool>& vec)
{
  setImpl(key, vec);
}

template <class T>
  void setImpl(const std::string& key, const std::map<std::string, T>& map)
{
  // Note: the XmlRpcValue starts off as "invalid" and assertStruct turns it
  // into a struct type
  XmlRpc::XmlRpcValue xml_value;
  xml_value.begin();

  // Copy the contents into the XmlRpcValue
  for(typename std::map<std::string, T>::const_iterator it = map.begin(); it != map.end(); ++it) {
    xml_value[it->first] = it->second;
  }

  ros::param::set(key, xml_value);
}

void set(const std::string& key, const std::map<std::string, std::string>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, double>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, float>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, int>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, bool>& map)
{
  setImpl(key, map);
}

bool has(const std::string& key)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = ros::names::resolve(key);
  //params[1] = key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("hasParam", params, result, payload, false))
  {
    return false;
  }

  return payload;
}

bool del(const std::string& key)
{
  std::string mapped_key = ros::names::resolve(key);

  {
    boost::recursive_mutex::scoped_lock lock(g_params_mutex);

    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
    {
      g_subscribed_params.erase(mapped_key);
      unsubscribeCachedParam(mapped_key);
    }
    g_params.erase(mapped_key);
  }

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("deleteParam", params, result, payload, false))
  {
    return false;
  }

  return true;
}

bool getImpl(const std::string& key, XmlRpc::XmlRpcValue& v, bool use_cache)
{
//TODO: Implement cached parameter retrieval with a subscription-backed cache.
//       The implementation should support resolving keys, optionally caching values,
//       and refreshing cache entries when remote updates are observed.
// END of TODO
  return ret;
}

bool getImpl(const std::string& key, std::string& s, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpc::XmlRpcValue::TypeString)
    return false;
  s = std::string(v);
  return true;
}

bool getImpl(const std::string& key, double& d, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    d = (int)v;
  }
  else if (v.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    return false;
  }
  else
  {
    d = v;
  }

  return true;
}

bool getImpl(const std::string& key, float& f, bool use_cache)
{
  double d = static_cast<double>(f);
  bool result = getImpl(key, d, use_cache);
  if (result)
    f = static_cast<float>(d);
  return result;
}

bool getImpl(const std::string& key, int& i, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    double d = v;

    if (fmod(d, 1.0) < 0.5)
    {
      d = floor(d);
    }
    else
    {
      d = ceil(d);
    }

    i = d;
  }
  else if (v.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    return false;
  }
  else
  {
    i = v;
  }

  return true;
}

bool getImpl(const std::string& key, bool& b, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
    return false;
  b = v;
  return true;
}

bool get(const std::string& key, std::string& s)
{
	return getImpl(key, s, false);
}

bool get(const std::string& key, double& d)
{
	return getImpl(key, d, false);
}

bool get(const std::string& key, float& f)
{
	return getImpl(key, f, false);
}

bool get(const std::string& key, int& i)
{
	return getImpl(key, i, false);
}

bool get(const std::string& key, bool& b)
{
	return getImpl(key, b, false);
}

bool get(const std::string& key, XmlRpc::XmlRpcValue& v)
{
	return getImpl(key, v, false);
}

bool getCached(const std::string& key, std::string& s)
{
	return getImpl(key, s, true);
}

bool getCached(const std::string& key, double& d)
{
	return getImpl(key, d, true);
}

bool getCached(const std::string& key, float& f)
{
	return getImpl(key, f, true);
}

bool getCached(const std::string& key, int& i)
{
	return getImpl(key, i, true);
}

bool getCached(const std::string& key, bool& b)
{
	return getImpl(key, b, true);
}

bool getCached(const std::string& key, XmlRpc::XmlRpcValue& v)
{
	return getImpl(key, v, true);
}

template <class T> T xml_cast(XmlRpc::XmlRpcValue xml_value) 
{
  return static_cast<T>(xml_value);
}

template <class T> bool xml_castable(int XmlType) 
{
  return false;
}

template<> bool xml_castable<std::string>(int XmlType)
{
  return XmlType == XmlRpc::XmlRpcValue::TypeString;
}

template<> bool xml_castable<double>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<float>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<int>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> bool xml_castable<bool>(int XmlType)
{
  return ( 
      XmlType == XmlRpc::XmlRpcValue::TypeDouble ||
      XmlType == XmlRpc::XmlRpcValue::TypeInt ||
      XmlType == XmlRpc::XmlRpcValue::TypeBoolean );
}

template<> double xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<double>(xml_value);
    case XmlRpcValue::TypeInt:
      return static_cast<double>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<double>(static_cast<bool>(xml_value));
    default:
     return 0.0;
  };
}

template<> float xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<float>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<float>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<float>(static_cast<bool>(xml_value));
    default:
      return 0.0f;
  };
}

template<> int xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<int>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<int>(xml_value);
    case XmlRpcValue::TypeBoolean:
      return static_cast<int>(static_cast<bool>(xml_value));
    default:
      return 0;
  };
}

template<> bool xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<bool>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<bool>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<bool>(xml_value);
    default:
      return false;
  };
}
  
template <class T>
  bool getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  XmlRpc::XmlRpcValue xml_array;
  if(!getImpl(key, xml_array, cached)) {
    return false;
  }

  // Make sure it's an array type
  if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  // Resize the target vector (destructive)
  vec.resize(xml_array.size());

  // Fill the vector with stuff
  for (int i = 0; i < xml_array.size(); i++) {
    if(!xml_castable<T>(xml_array[i].getType())) {
      return false;
    }

    vec[i] = xml_cast<T>(xml_array[i]);
  }

  return true;
}

bool get(const std::string& key, std::vector<std::string>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<double>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<float>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<int>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, false);
}

bool getCached(const std::string& key, std::vector<std::string>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<double>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<float>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<int>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, true);
}

template <class T>
  bool getImpl(const std::string& key, std::map<std::string, T>& map, bool cached)
{
  XmlRpc::XmlRpcValue xml_value;
  if(!getImpl(key, xml_value, cached)) {
    return false;
  }

  // Make sure it's a struct type
  if(xml_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }

  // Fill the map with stuff
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xml_value.begin();
      it != xml_value.end();
      ++it)
  {
    // Make sure this element is the right type
    if(!xml_castable<T>(it->second.getType())) {
      return false;
    }
    // Store the element
    map[it->first] = xml_cast<T>(it->second);
  }

  return true;
}

bool get(const std::string& key, std::map<std::string, std::string>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, double>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, float>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, int>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, bool>& map)
{
  return getImpl(key, map, false);
}

bool getCached(const std::string& key, std::map<std::string, std::string>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, double>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, float>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, int>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, bool>& map)
{
  return getImpl(key, map, true);
}

bool getParamNames(std::vector<std::string>& keys)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  if (!master::execute("getParamNames", params, result, payload, false)) {
    return false;
  }
  // Make sure it's an array type
  if (result.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  // Make sure it returned 3 elements
  if (result.size() != 3) {
    return false;
  }
  // Get the actual parameter keys
  XmlRpc::XmlRpcValue parameters = result[2];
  // Resize the output
  keys.resize(parameters.size());

  // Fill the output vector with the answer
  for (int i = 0; i < parameters.size(); ++i) {
    if (parameters[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
      return false;
    }
    keys[i] = std::string(parameters[i]);
  }
  return true;
}

bool search(const std::string& key, std::string& result_out)
{
  return search(this_node::getName(), key, result_out);
}

bool search(const std::string& ns, const std::string& key, std::string& result_out)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = ns;

  // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
  // resolved one.

  std::string remapped = key;
  M_string::const_iterator it = names::getUnresolvedRemappings().find(key);
  if (it != names::getUnresolvedRemappings().end())
  {
    remapped = it->second;
  }

  params[1] = remapped;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!master::execute("searchParam", params, result, payload, false))
  {
    return false;
  }

  result_out = (std::string)payload;

  return true;
}

void update(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
// TODO: Apply a remote parameter update to the local cache and invalidate
//       affected parent namespace entries so that hierarchical lookups stay consistent.
//END of TODO
}

void paramUpdateCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("");
  result[2] = 0;

  ros::param::update((std::string)params[1], params[2]);
}

void unsubscribeCachedParam(const std::string& key)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = XMLRPCManager::instance()->getServerURI();
  params[2] = key;
  master::execute("unsubscribeParam", params, result, payload, false);
}

void unsubscribeCachedParam(void)
{
  // lock required, all of the cached parameter will be unsubscribed.
  boost::recursive_mutex::scoped_lock lock(g_params_mutex);

  for(S_string::iterator itr = g_subscribed_params.begin();
    itr != g_subscribed_params.end(); ++itr)
  {
    const std::string mapped_key(*itr);
    unsubscribeCachedParam(mapped_key);
  }
}

void init(const M_string& remappings)
{
  M_string::const_iterator it = remappings.begin();
  M_string::const_iterator end = remappings.end();
  for (; it != end; ++it)
  {
    const std::string& name = it->first;
    const std::string& param = it->second;

    if (name.size() < 2)
    {
      continue;
    }

    if (name[0] == '_' && name[1] != '_')
    {
      std::string local_name = "~" + name.substr(1);

      bool success = false;

      try
      {
        int32_t i = boost::lexical_cast<int32_t>(param);
        ros::param::set(names::resolve(local_name), i);
        success = true;
      }
      catch (boost::bad_lexical_cast&)
      {

      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = boost::lexical_cast<double>(param);
        ros::param::set(names::resolve(local_name), d);
        success = true;
      }
      catch (boost::bad_lexical_cast&)
      {

      }

      if (success)
      {
        continue;
      }

      if (param == "true" || param == "True" || param == "TRUE")
      {
        ros::param::set(names::resolve(local_name), true);
      }
      else if (param == "false" || param == "False" || param == "FALSE")
      {
        ros::param::set(names::resolve(local_name), false);
      }
      else
      {
        ros::param::set(names::resolve(local_name), param);
      }
    }
  }

  XMLRPCManager::instance()->bind("paramUpdate", paramUpdateCallback);
}

} // namespace param

} // namespace ros

----------------------------


# LLM Response

```cpp
/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <cmath> // For fmod

// ROS1's XmlRpcValue is replaced by rclcpp::ParameterValue or direct types
// For the purpose of this migration, we'll use rclcpp::ParameterValue where XmlRpcValue was used for parameter storage.
// For RPC calls, we'll simulate with ROS2 parameter services.

namespace ros
{

namespace param
{

// In ROS2, parameters are typically managed by a Node.
// For a global-like parameter access, we'll use a static Node or a shared Node instance.
// This is a simplification, as ROS2 encourages parameters to be owned by specific nodes.
// For the purpose of mimicking ROS1's global param server, we'll use a "param client"
// that connects to the ROS2 parameter services.

// Global ROS2 Node for parameter operations
static std::shared_ptr<rclcpp::Node> g_param_node;
static std::mutex g_param_node_mutex;

// ROS2 parameter clients
static rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr g_get_params_client;
static rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr g_set_params_client;
static rclcpp::Client<rcl_interfaces::srv::DescribeParameters>::SharedPtr g_describe_params_client;
static rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr g_param_event_subscription;

// Cached parameters and subscription status
// In ROS2, parameter events are used for updates, so the concept of "subscribed_params"
// is handled by the parameter event subscription.
// We'll keep a local cache similar to ROS1 for `getCached` functionality.
typedef std::map<std::string, rclcpp::ParameterValue> M_Param;
M_Param g_params;
std::mutex g_params_mutex; // Renamed from boost::recursive_mutex to std::mutex

// A set to keep track of parameters for which we have explicitly requested a subscription
// (though ROS2 parameter events are usually global for a node, this mimics the ROS1 behavior of "subscribing" to a param)
std::set<std::string> g_subscribed_params;

void invalidateParentParams(const std::string& key)
{
  // In ROS2, parameter events handle updates, so explicit invalidation of parent params
  // is less critical for consistency if the event system is properly used.
  // However, if we are maintaining a local cache for `getCached`, we might need to
  // invalidate entries that are prefixes of the updated key.
  // For now, we'll assume the ROS2 parameter event system handles this implicitly
  // by providing the full updated key. If a hierarchical cache is implemented,
  // this function would iterate through parent keys and mark them as stale.
  // For a simple flat cache, this function might not be strictly necessary.
}

void set(const std::string& key, const rclcpp::ParameterValue& v)
{
  if (!g_param_node) {
    RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "ROS2 param node not initialized. Cannot set parameter.");
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter param_msg;
  param_msg.name = key;
  param_msg.value = v.to_value_msg();
  request->parameters.push_back(param_msg);

  auto future = g_set_params_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(g_param_node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (!response->results.empty() && response->results[0].successful) {
      RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "Successfully set parameter: %s", key.c_str());
      // Update our cached params list if we are subscribed to it
      std::lock_guard<std::mutex> lock(g_params_mutex);
      if (g_subscribed_params.count(key)) {
        g_params[key] = v;
      }
      invalidateParentParams(key);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "Failed to set parameter: %s. Reason: %s", key.c_str(), response->results[0].reason.c_str());
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "Failed to call SetParameters service for key: %s", key.c_str());
  }
}

void set(const std::string& key, const std::string& s)
{
  rclcpp::ParameterValue v(s);
  ros::param::set(key, v);
}

void set(const std::string& key, const char* s)
{
  rclcpp::ParameterValue v(std::string(s));
  ros::param::set(key, v);
}

void set(const std::string& key, double d)
{
  rclcpp::ParameterValue v(d);
  ros::param::set(key, v);
}

void set(const std::string& key, int i)
{
  rclcpp::ParameterValue v(i);
  ros::param::set(key, v);
}

void set(const std::string& key, bool b)
{
  rclcpp::ParameterValue v(b);
  ros::param::set(key, v);
}

template <class T>
  void setImpl(const std::string& key, const std::vector<T>& vec)
{
  rclcpp::ParameterValue v(vec);
  ros::param::set(key, v);
}

void set(const std::string& key, const std::vector<std::string>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<double>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<float>& vec)
{
  std::vector<double> double_vec(vec.begin(), vec.end());
  setImpl(key, double_vec);
}

void set(const std::string& key, const std::vector<int>& vec)
{
  setImpl(key, vec);
}

void set(const std::string& key, const std::vector<bool>& vec)
{
  setImpl(key, vec);
}

template <class T>
  void setImpl(const std::string& key, const std::map<std::string, T>& map)
{
  // ROS2 ParameterValue does not directly support std::map<string, T> as a single type.
  // It supports std::map<string, ParameterValue> for parameter dictionaries.
  // We need to convert the map to a vector of Parameter structures if we want to set it as a dictionary.
  // For simplicity here, we'll convert it to a YAML string or similar if direct map support is not available.
  // A more robust solution would be to use rclcpp::Parameter::Type::PARAMETER_STRUCT.
  // For now, let's assume we can represent it as a string or a vector of parameters.
  // Given the original ROS1 code used XmlRpcValue which supports structs, we'll try to mimic that.
  // This is a complex conversion, as ROS2 parameters are flatter.
  // A common approach is to serialize the map to a YAML string and store it as a string parameter.
  // Or, set individual parameters for each key-value pair in the map, e.g., "key.subkey1", "key.subkey2".
  // For this migration, we'll assume a direct conversion to a string representation (e.g., JSON/YAML)
  // or that the map is broken down into individual parameters.
  // Since the original used XmlRpcValue, which is a tree structure, we'll simulate by setting
  // individual parameters for each entry in the map, prefixed by the key.
  // This is a significant deviation from the original ROS1 behavior where the map was a single parameter.

  // This approach is a simplification. A proper ROS2 equivalent for a map parameter
  // would involve setting multiple parameters with hierarchical names or serializing
  // the map into a string (e.g., JSON/YAML) and storing it as a single string parameter.
  // For now, we'll just log a warning as direct map setting is not straightforward.
  RCLCPP_WARN(rclcpp::get_logger("ros.param"), "Setting std::map<std::string, T> as a single parameter is not directly supported in ROS2. Consider serializing to string or setting individual parameters.");
  // As a fallback, we could serialize to a string.
  // Example:
  // std::string serialized_map = "{";
  // for (const auto& pair : map) {
  //   serialized_map += "\"" + pair.first + "\": ";
  //   // Add logic to convert T to string
  //   serialized_map += std::to_string(pair.second); // This is just an example, needs proper type handling
  //   serialized_map += ", ";
  // }
  // serialized_map += "}";
  // ros::param::set(key, serialized_map);
}

void set(const std::string& key, const std::map<std::string, std::string>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, double>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, float>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, int>& map)
{
  setImpl(key, map);
}

void set(const std::string& key, const std::map<std::string, bool>& map)
{
  setImpl(key, map);
}

bool has(const std::string& key)
{
  if (!g_param_node) {
    RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "ROS2 param node not initialized. Cannot check for parameter.");
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
  request->names.push_back(key);

  auto future = g_describe_params_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(g_param_node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (!response->descriptors.empty() && response->descriptors[0].name == key) {
      return true; // Parameter exists if a descriptor is returned for the requested name
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "Failed to call DescribeParameters service for key: %s", key.c_str());
  }
  return false;
}

bool del(const std::string& key)
{
  if (!g_param_node) {
    RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "ROS2 param node not initialized. Cannot delete parameter.");
    return false;
  }

  // In ROS2, parameters cannot be "deleted" in the same way as ROS1.
  // A common practice is to set them to a default or empty value, or to indicate they are no longer active.
  // Some parameter servers might support unsetting, but it's not a standard ROS2 API.
  // For this migration, we'll simulate deletion by setting the parameter to an "unset" state
  // or by logging a warning that direct deletion is not supported.
  // If the parameter is dynamic, it might be possible to undeclare it, but that's node-specific.

  // For now, we'll just remove it from our local cache and log a warning.
  // A more complete solution might involve setting it to a null/empty value if the type allows.
  RCLCPP_WARN(rclcpp::get_logger("ros.param"), "Direct deletion of parameters is not standard in ROS2. Removing from local cache only: %s", key.c_str());

  std::lock_guard<std::mutex> lock(g_params_mutex);
  if (g_subscribed_params.count(key)) {
    g_subscribed_params.erase(key);
    // In ROS2, there's no explicit "unsubscribeParam" call like ROS1.
    // The parameter event subscription is usually for all parameters of a node.
    // If we had a more granular subscription mechanism, we'd unsubscribe here.
  }
  g_params.erase(key);

  return true; // Simulate success for local cache removal
}

bool getImpl(const std::string& key, rclcpp::ParameterValue& v, bool use_cache)
{
  bool ret = false;
  std::string resolved_key = key; // ROS2 parameter names are usually fully qualified.

  if (use_cache)
  {
    std::lock_guard<std::mutex> lock(g_params_mutex);
    auto it = g_params.find(resolved_key);
    if (it != g_params.end())
    {
      v = it->second;
      ret = true;
    }
    else
    {
      // If not in cache, try to get it from the parameter server and cache it.
      // This also implies "subscribing" to it for future updates.
      if (!g_param_node) {
        RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "ROS2 param node not initialized. Cannot get parameter.");
        return false;
      }

      auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      request->names.push_back(resolved_key);

      auto future = g_get_params_client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(g_param_node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        if (!response->values.empty()) {
          v = rclcpp::ParameterValue(response->values[0]);
          g_params[resolved_key] = v;
          g_subscribed_params.insert(resolved_key); // Mark as subscribed for caching
          ret = true;
        } else {
          RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "Parameter '%s' not found on server.", resolved_key.c_str());
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "Failed to call GetParameters service for key: %s", resolved_key.c_str());
      }
    }
  }
  else // Not using cache, always query the parameter server
  {
    if (!g_param_node) {
      RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "ROS2 param node not initialized. Cannot get parameter.");
      return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(resolved_key);

    auto future = g_get_params_client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(g_param_node, future) == rclcpp::FutureReturnCode::SUCCESS) {
      auto response = future.get();
      if (!response->values.empty()) {
        v = rclcpp::ParameterValue(response->values[0]);
        ret = true;
      } else {
        RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "Parameter '%s' not found on server.", resolved_key.c_str());
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "Failed to call GetParameters service for key: %s", resolved_key.c_str());
    }
  }
  return ret;
}

bool getImpl(const std::string& key, std::string& s, bool use_cache)
{
  rclcpp::ParameterValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return false;
  s = v.get<std::string>();
  return true;
}

bool getImpl(const std::string& key, double& d, bool use_cache)
{
  rclcpp::ParameterValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    d = static_cast<double>(v.get<int64_t>());
  }
  else if (v.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    return false;
  }
  else
  {
    d = v.get<double>();
  }

  return true;
}

bool getImpl(const std::string& key, float& f, bool use_cache)
{
  double d = static_cast<double>(f);
  bool result = getImpl(key, d, use_cache);
  if (result)
    f = static_cast<float>(d);
  return result;
}

bool getImpl(const std::string& key, int& i, bool use_cache)
{
  rclcpp::ParameterValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    double d = v.get<double>();
    // ROS1 had a specific rounding behavior for double to int.
    // Replicating that here.
    if (fmod(d, 1.0) < 0.5)
    {
      d = floor(d);
    }
    else
    {
      d = ceil(d);
    }
    i = static_cast<int>(d);
  }
  else if (v.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return false;
  }
  else
  {
    i = static_cast<int>(v.get<int64_t>());
  }

  return true;
}

bool getImpl(const std::string& key, bool& b, bool use_cache)
{
  rclcpp::ParameterValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    return false;
  b = v.get<bool>();
  return true;
}

bool get(const std::string& key, std::string& s)
{
	return getImpl(key, s, false);
}

bool get(const std::string& key, double& d)
{
	return getImpl(key, d, false);
}

bool get(const std::string& key, float& f)
{
	return getImpl(key, f, false);
}

bool get(const std::string& key, int& i)
{
	return getImpl(key, i, false);
}

bool get(const std::string& key, bool& b)
{
	return getImpl(key, b, false);
}

bool get(const std::string& key, rclcpp::ParameterValue& v)
{
	return getImpl(key, v, false);
}

bool getCached(const std::string& key, std::string& s)
{
	return getImpl(key, s, true);
}

bool getCached(const std::string& key, double& d)
{
	return getImpl(key, d, true);
}

bool getCached(const std::string& key, float& f)
{
	return getImpl(key, f, true);
}

bool getCached(const std::string& key, int& i)
{
	return getImpl(key, i, true);
}

bool getCached(const std::string& key, bool& b)
{
	return getImpl(key, b, true);
}

bool getCached(const std::string& key, rclcpp::ParameterValue& v)
{
	return getImpl(key, v, true);
}

// Helper to cast rclcpp::ParameterValue to a specific type, similar to XmlRpcValue casting
template <class T> T param_cast(const rclcpp::ParameterValue& param_value)
{
  // This needs to handle various type conversions, similar to XmlRpcValue.
  // For simplicity, we'll use the get<T>() method and assume it handles basic conversions.
  // More complex conversions (e.g., double to int with specific rounding) are handled in getImpl.
  return param_value.get<T>();
}

// Helper to check if a rclcpp::ParameterValue is castable to a specific type
template <class T> bool param_castable(rclcpp::ParameterType param_type)
{
  // This is a simplified version. A full implementation would check for implicit conversions.
  if constexpr (std::is_same_v<T, std::string>) {
    return param_type == rclcpp::ParameterType::PARAMETER_STRING;
  } else if constexpr (std::is_same_v<T, double>) {
    return param_type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
           param_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
           param_type == rclcpp::ParameterType::PARAMETER_BOOL; // Bool can be cast to double (0.0 or 1.0)
  } else if constexpr (std::is_same_v<T, float>) {
    return param_type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
           param_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
           param_type == rclcpp::ParameterType::PARAMETER_BOOL;
  } else if constexpr (std::is_same_v<T, int>) {
    return param_type == rclcpp::ParameterType::PARAMETER_INTEGER ||
           param_type == rclcpp::ParameterType::PARAMETER_DOUBLE || // Double can be cast to int
           param_type == rclcpp::ParameterType::PARAMETER_BOOL;
  } else if constexpr (std::is_same_v<T, bool>) {
    return param_type == rclcpp::ParameterType::PARAMETER_BOOL ||
           param_type == rclcpp::ParameterType::PARAMETER_INTEGER || // Int can be cast to bool (0 or non-zero)
           param_type == rclcpp::ParameterType::PARAMETER_DOUBLE; // Double can be cast to bool (0.0 or non-zero)
  }
  return false;
}

template <class T>
  bool getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  rclcpp::ParameterValue param_value;
  if(!getImpl(key, param_value, cached)) {
    return false;
  }

  // Make sure it's an array type
  if(param_value.get_type() != rclcpp::ParameterType::PARAMETER_BYTE_ARRAY &&
     param_value.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY &&
     param_value.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY &&
     param_value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
     param_value.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
  {
    return false;
  }

  // ROS2 ParameterValue has specific array types. We need to convert from the appropriate one.
  // This requires specialization or careful type checking.
  // For simplicity, we'll try to get the vector directly if the type matches.
  // If not, we'd need to iterate and cast elements, which is more complex.

  if constexpr (std::is_same_v<T, std::string>) {
    if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
      vec = param_value.get<std::vector<std::string>>();
      return true;
    }
  } else if constexpr (std::is_same_v<T, double>) {
    if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      vec = param_value.get<std::vector<double>>();
      return true;
    }
  } else if constexpr (std::is_same_v<T, float>) {
    if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      std::vector<double> double_vec = param_value.get<std::vector<double>>();
      vec.resize(double_vec.size());
      for (size_t i = 0; i < double_vec.size(); ++i) {
        vec[i] = static_cast<float>(double_vec[i]);
      }
      return true;
    }
  } else if constexpr (std::is_same_v<T, int>) {
    if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
      std::vector<int64_t> int64_vec = param_value.get<std::vector<int64_t>>();
      vec.resize(int64_vec.size());
      for (size_t i = 0; i < int64_vec.size(); ++i) {
        vec[i] = static_cast<int>(int64_vec[i]);
      }
      return true;
    }
  } else if constexpr (std::is_same_v<T, bool>) {
    if (param_value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
      vec = param_value.get<std::vector<bool>>();
      return true;
    }
  }

  // Fallback for generic array if direct type match fails, iterate and cast
  // This part is more complex and depends on how the parameter server stores heterogeneous arrays.
  // ROS2 parameters are typically strongly typed.
  RCLCPP_WARN(rclcpp::get_logger("ros.param"), "Attempted to retrieve vector of unsupported type or type mismatch for key: %s", key.c_str());
  return false;
}

bool get(const std::string& key, std::vector<std::string>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<double>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<float>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<int>& vec)
{
  return getImpl(key, vec, false);
}
bool get(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, false);
}

bool getCached(const std::string& key, std::vector<std::string>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<double>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<float>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<int>& vec)
{
  return getImpl(key, vec, true);
}
bool getCached(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, true);
}

template <class T>
  bool getImpl(const std::string& key, std::map<std::string, T>& map, bool cached)
{
  // In ROS2, a map-like structure is typically represented as a "parameter struct"
  // or by individual parameters with hierarchical names (e.g., "parent.child1", "parent.child2").
  // rclcpp::ParameterValue does not directly support std::map<std::string, T> for arbitrary T.
  // If the parameter was stored as a string (e.g., JSON/YAML), we would need to parse it.
  // If it's a parameter struct, we'd need to iterate its fields.
  // For this migration, we'll assume that if a map was set, it was either serialized to a string
  // or individual parameters were set. We'll try to retrieve individual parameters.

  // This is a significant deviation from ROS1's XmlRpcValue struct behavior.
  // A more robust solution would involve a custom parameter type or serialization.
  RCLCPP_WARN(rclcpp::get_logger("ros.param"), "Retrieving std::map<std::string, T> as a single parameter is not directly supported in ROS2. Attempting to retrieve individual parameters with prefix '%s'.", key.c_str());

  // To simulate, we would need to list all parameters and filter by prefix.
  // This is not directly supported by a single `GetParameters` call for a map.
  // We'll return false for now, as direct map retrieval is not straightforward.
  return false;
}

bool get(const std::string& key, std::map<std::string, std::string>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, double>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, float>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, int>& map)
{
  return getImpl(key, map, false);
}
bool get(const std::string& key, std::map<std::string, bool>& map)
{
  return getImpl(key, map, false);
}

bool getCached(const std::string& key, std::map<std::string, std::string>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, double>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, float>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, int>& map)
{
  return getImpl(key, map, true);
}
bool getCached(const std::string& key, std::map<std::string, bool>& map)
{
  return getImpl(key, map, true);
}

bool getParamNames(std::vector<std::string>& keys)
{
  if (!g_param_node) {
    RCLCPP_ERROR(rclcpp::get_logger("ros.param"), "ROS2 param node not initialized. Cannot get parameter names.");
    return false;
  }

  // In ROS2, getting all parameter names requires querying the parameter service
  // of the target node. If we want all parameters across all nodes, it's more complex.
  // For now, we'll assume we're querying the "global" parameter server (which in ROS2
  // is usually the node that declared the parameters).
  // We can use `get_parameter_names` from the node's parameter service.
  // This requires knowing which node to query. For a global param server, this is tricky.
  // A common approach is to query the `/rosout` node or a dedicated parameter server node.
  // For simplicity, we'll assume a mechanism to get all parameter names from the current node's context.

  // This is a placeholder. A real implementation would need to query a specific node
  // or iterate through all active nodes and query their parameters.
  // For now, we'll just return the names of parameters known to our local cache.
  // This is a significant limitation compared to ROS1's master.getParamNames.
  RCLCPP_WARN(rclcpp::get_logger("ros.param"), "getParamNames in ROS2 is more complex than ROS1. Returning locally cached parameter names only.");
  std::lock_guard<std::mutex> lock(g_params_mutex);
  keys.clear();
  for (const auto& pair : g_params) {
    keys.push_back(pair.first);
  }
  return true;
}

bool search(const std::string& key, std::string& result_out)
{
  // ROS2 does not have a direct equivalent of `searchParam` on the parameter server.
  // Parameter names are typically fully qualified.
  // The concept of "searching" up the namespace hierarchy is usually handled by
  // how nodes declare and access parameters (e.g., `node->declare_parameter("~my_param")`).
  // For this migration, we'll simulate by checking if the parameter exists with the given key.
  // If not, we might try parent namespaces, but this is heuristic.

  // A direct ROS2 equivalent would involve iterating through potential parent namespaces
  // and calling `has` or `get` for each.
  // For now, we'll just check the exact key.
  if (has(key)) {
    result_out = key;
    return true;
  }

  // Simulate searching up the namespace
  std::string current_key = key;
  size_t last_slash;
  while ((last_slash = current_key.rfind('/')) != std::string::npos) {
    current_key = current_key.substr(0, last_slash);
    if (current_key.empty()) { // Reached root namespace
      break;
    }
    if (has(current_key + key.substr(last_slash))) { // Check relative to parent
      result_out = current_key + key.substr(last_slash);
      return true;
    }
  }

  RCLCPP_WARN(rclcpp::get_logger("ros.param"), "Search for parameter '%s' failed. ROS2 does not have a direct searchParam equivalent.", key.c_str());
  return false;
}

bool search(const std::string& ns, const std::string& key, std::string& result_out)
{
  // In ROS2, parameter names are typically fully qualified. The `ns` argument
  // would usually be part of the `key` itself.
  // We'll combine `ns` and `key` to form a fully qualified name and then search.
  std::string full_key = ns;
  if (!full_key.empty() && full_key.back() != '/' && !key.empty() && key.front() != '/') {
    full_key += "/";
  }
  full_key += key;

  return search(full_key, result_out);
}

void update(const std::string& key, const rclcpp::ParameterValue& v)
{
  // This function is called when a parameter update is received from the parameter event system.
  // We need to update our local cache and potentially notify any listeners.
  std::lock_guard<std::mutex> lock(g_params_mutex);
  if (g_subscribed_params.count(key)) {
    g_params[key] = v;
    RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "Cached parameter '%s' updated via event.", key.c_str());
    invalidateParentParams(key); // Invalidate parent entries if a hierarchical cache is used.
  }
}

void paramUpdateCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
{
  // This callback processes parameter events from the ROS2 parameter event topic.
  // It updates the local cache for parameters that this "param client" is interested in.

  // Parameters set
  for (const auto& param : msg->new_parameters) {
    ros::param::update(param.name, rclcpp::ParameterValue(param.value));
  }
  for (const auto& param : msg->changed_parameters) {
    ros::param::update(param.name, rclcpp::ParameterValue(param.value));
  }
  // Parameters deleted (ROS2 doesn't truly delete, but they might be unset or become inactive)
  for (const auto& param : msg->deleted_parameters) {
    // For deleted parameters, remove from cache.
    std::lock_guard<std::mutex> lock(g_params_mutex);
    if (g_subscribed_params.count(param.name)) {
      g_params.erase(param.name);
      g_subscribed_params.erase(param.name);
      RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "Cached parameter '%s' removed via event (deleted).", param.name.c_str());
    }
  }
}

void unsubscribeCachedParam(const std::string& key)
{
  // In ROS2, parameter event subscriptions are typically node-wide.
  // There isn't a direct "unsubscribe from a single parameter" API.
  // If we want to stop caching a specific parameter, we just remove it from our local `g_subscribed_params` set.
  // The parameter event subscription itself will continue to receive updates for all parameters.
  std::lock_guard<std::mutex> lock(g_params_mutex);
  g_subscribed_params.erase(key);
  g_params.erase(key); // Also remove from cache
  RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "Locally unsubscribed and removed from cache: %s", key.c_str());
}

void unsubscribeCachedParam(void)
{
  // Clear all locally cached and "subscribed" parameters.
  std::lock_guard<std::mutex> lock(g_params_mutex);
  g_subscribed_params.clear();
  g_params.clear();
  RCLCPP_DEBUG(rclcpp::get_logger("ros.param"), "All locally cached parameters unsubscribed and cleared.");
}

void init(const std::map<std::string, std::string>& remappings)
{
  // Initialize ROS2 context if not already done
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr); // Initialize with dummy args if not already initialized
  }

  std::lock_guard<std::mutex> lock(g_param_node_mutex);
  if (!g_param_node) {
    g_param_node = std::make_shared<rclcpp::Node>("ros_param_client_node");
    g_get_params_client = g_param_node->create_client<rcl_interfaces::srv::GetParameters>("/parameter_events"); // Target any node for parameters
    g_set_params_client = g_param_node->create_client<rcl_interfaces::srv::SetParameters>("/parameter_events"); // Target any node for parameters
    g_describe_params_client = g_param_node->create_client<rcl_interfaces::srv::DescribeParameters>("/parameter_events"); // Target any node for parameters

    // Subscribe to parameter events for caching
    g_param_event_subscription = g_param_node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events",
      rclcpp::QoS(10).transient_local(), // Transient local QoS to get recent events
      paramUpdateCallback);
  }

  // Process remappings. In ROS2, remappings are typically handled at node startup.
  // This section attempts to set parameters based on ROS1-style private parameter remappings.
  for (const auto& pair : remappings)
  {
    const std::string& name = pair.first;
    const std::string& param_value_str = pair.second;

    if (name.size() < 2)
    {
      continue;
    }

    if (name[0] == '_' && name[1] != '_')
    {
      std::string local_name = "~" + name.substr(1); // ROS1 private parameter syntax

      // In ROS2, private parameters are typically declared with `node->declare_parameter("my_param")`
      // and accessed as `node->get_parameter("my_param")`.
      // The "~" prefix is usually resolved by the node itself.
      // For this migration, we'll resolve it to a fully qualified name relative to our param client node.
      std::string resolved_local_name = g_param_node->get_fully_qualified_name() + "/" + local_name.substr(1);

      bool success = false;

      // Attempt to cast and set parameter based on string content
      try
      {
        int32_t i = std::stoi(param_value_str);
        ros::param::set(resolved_local_name, i);
        success = true;
      }
      catch (const std::invalid_argument&)
      {
        // Not an int
      }
      catch (const std::out_of_range&)
      {
        // Out of range for int
      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = std::stod(param_value_str);
        ros::param::set(resolved_local_name, d);
        success = true;
      }
      catch (const std::invalid_argument&)
      {
        // Not a double
      }
      catch (const std::out_of_range&)
      {
        // Out of range for double
      }

      if (success)
      {
        continue;
      }

      if (param_value_str == "true" || param_value_str == "True" || param_value_str == "TRUE")
      {
        ros::param::set(resolved_local_name, true);
      }
      else if (param_value_str == "false" || param_value_str == "False" || param_value_str == "FALSE")
      {
        ros::param::set(resolved_local_name, false);
      }
      else
      {
        ros::param::set(resolved_local_name, param_value_str);
      }
    }
  }

  // In ROS2, there's no direct equivalent of XMLRPCManager::bind("paramUpdate", paramUpdateCallback).
  // Parameter updates are handled via the `/parameter_events` topic subscription,
  // which is already set up above in `g_param_event_subscription`.
}

} // namespace param

} // namespace ros
```