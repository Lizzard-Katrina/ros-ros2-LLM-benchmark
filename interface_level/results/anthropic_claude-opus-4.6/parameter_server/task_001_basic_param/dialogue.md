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

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include <mutex>
#include <vector>
#include <map>
#include <string>
#include <set>
#include <cmath>
#include <stdexcept>

namespace ros2_param
{

namespace param
{

typedef std::map<std::string, rclcpp::Parameter> M_Param;
static M_Param g_params;
static std::recursive_mutex g_params_mutex;
static std::set<std::string> g_subscribed_params;
static rclcpp::Node::SharedPtr g_node = nullptr;

void setNode(rclcpp::Node::SharedPtr node)
{
  g_node = node;
}

rclcpp::Node::SharedPtr getNode()
{
  if (!g_node)
  {
    throw std::runtime_error("Parameter node not initialized. Call setNode() first.");
  }
  return g_node;
}

void invalidateParentParams(const std::string& key)
{
  // Invalidate cached parent namespace entries for a given key.
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
  std::string ns = key;
  while (true)
  {
    size_t pos = ns.rfind('/');
    if (pos == std::string::npos || pos == 0)
    {
      break;
    }
    ns = ns.substr(0, pos);
    auto it = g_params.find(ns);
    if (it != g_params.end())
    {
      g_params.erase(it);
    }
  }
}

void set(const std::string& key, const rclcpp::ParameterValue& v)
{
  auto node = getNode();
  std::string resolved_key = key;

  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

    try
    {
      if (!node->has_parameter(resolved_key))
      {
        node->declare_parameter(resolved_key, v);
      }
      else
      {
        node->set_parameter(rclcpp::Parameter(resolved_key, v));
      }

      if (g_subscribed_params.find(resolved_key) != g_subscribed_params.end())
      {
        g_params[resolved_key] = rclcpp::Parameter(resolved_key, v);
      }
      invalidateParentParams(resolved_key);
    }
    catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &)
    {
      node->set_parameter(rclcpp::Parameter(resolved_key, v));
    }
  }
}

void set(const std::string& key, const std::string& s)
{
  rclcpp::ParameterValue v(s);
  param::set(key, v);
}

void set(const std::string& key, const char* s)
{
  std::string sxx = std::string(s);
  rclcpp::ParameterValue v(sxx);
  param::set(key, v);
}

void set(const std::string& key, double d)
{
  rclcpp::ParameterValue v(d);
  param::set(key, v);
}

void set(const std::string& key, int i)
{
  rclcpp::ParameterValue v(i);
  param::set(key, v);
}

void set(const std::string& key, bool b)
{
  rclcpp::ParameterValue v(b);
  param::set(key, v);
}

template <class T>
  void setImpl(const std::string& key, const std::vector<T>& vec)
{
  rclcpp::ParameterValue v(vec);
  param::set(key, v);
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
  // Convert float vector to double vector for ROS2 parameter compatibility
  std::vector<double> dvec(vec.begin(), vec.end());
  rclcpp::ParameterValue v(dvec);
  param::set(key, v);
}

void set(const std::string& key, const std::vector<int>& vec)
{
  // ROS2 parameters use int64_t
  std::vector<int64_t> lvec(vec.begin(), vec.end());
  rclcpp::ParameterValue v(lvec);
  param::set(key, v);
}

void set(const std::string& key, const std::vector<bool>& vec)
{
  setImpl(key, vec);
}

template <class T>
  void setImpl(const std::string& key, const std::map<std::string, T>& map)
{
  // ROS2 doesn't natively support map parameters, so we set each entry as a sub-parameter
  for (typename std::map<std::string, T>::const_iterator it = map.begin(); it != map.end(); ++it)
  {
    std::string sub_key = key + "." + it->first;
    param::set(sub_key, it->second);
  }
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
  for (auto it = map.begin(); it != map.end(); ++it)
  {
    std::string sub_key = key + "." + it->first;
    param::set(sub_key, static_cast<double>(it->second));
  }
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
  auto node = getNode();
  return node->has_parameter(key);
}

bool del(const std::string& key)
{
  auto node = getNode();
  std::string mapped_key = key;

  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
    {
      g_subscribed_params.erase(mapped_key);
    }
    g_params.erase(mapped_key);
  }

  try
  {
    node->undeclare_parameter(mapped_key);
  }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException &)
  {
    return false;
  }

  return true;
}

bool getImpl(const std::string& key, rclcpp::Parameter& p, bool use_cache)
{
  // Implement cached parameter retrieval with a subscription-backed cache.
  std::string mapped_key = key;

  if (use_cache)
  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

    // Check if we have a cached value
    auto it = g_params.find(mapped_key);
    if (it != g_params.end())
    {
      p = it->second;
      return true;
    }
  }

  auto node = getNode();
  bool ret = false;

  try
  {
    if (node->has_parameter(mapped_key))
    {
      p = node->get_parameter(mapped_key);
      ret = true;
    }
    else
    {
      ret = false;
    }
  }
  catch (const rclcpp::exceptions::ParameterNotDeclaredException &)
  {
    ret = false;
  }

  if (ret && use_cache)
  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
    g_subscribed_params.insert(mapped_key);
    g_params[mapped_key] = p;
  }

  return ret;
}

bool getImpl(const std::string& key, std::string& s, bool use_cache)
{
  rclcpp::Parameter p;
  if (!getImpl(key, p, use_cache))
    return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
    return false;
  s = p.as_string();
  return true;
}

bool getImpl(const std::string& key, double& d, bool use_cache)
{
  rclcpp::Parameter p;
  if (!getImpl(key, p, use_cache))
  {
    return false;
  }

  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    d = static_cast<double>(p.as_int());
  }
  else if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    return false;
  }
  else
  {
    d = p.as_double();
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
  rclcpp::Parameter p;
  if (!getImpl(key, p, use_cache))
  {
    return false;
  }

  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
  {
    double d = p.as_double();

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
  else if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
  {
    return false;
  }
  else
  {
    i = static_cast<int>(p.as_int());
  }

  return true;
}

bool getImpl(const std::string& key, bool& b, bool use_cache)
{
  rclcpp::Parameter p;
  if (!getImpl(key, p, use_cache))
    return false;
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
    return false;
  b = p.as_bool();
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

bool get(const std::string& key, rclcpp::Parameter& v)
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

bool getCached(const std::string& key, rclcpp::Parameter& v)
{
  return getImpl(key, v, true);
}

template <class T> T param_cast(const rclcpp::Parameter& param);

template <class T> bool param_castable(rclcpp::ParameterType type)
{
  return false;
}

template<> bool param_castable<std::string>(rclcpp::ParameterType type)
{
  return type == rclcpp::ParameterType::PARAMETER_STRING;
}

template<> bool param_castable<double>(rclcpp::ParameterType type)
{
  return (
      type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
      type == rclcpp::ParameterType::PARAMETER_INTEGER ||
      type == rclcpp::ParameterType::PARAMETER_BOOL);
}

template<> bool param_castable<float>(rclcpp::ParameterType type)
{
  return (
      type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
      type == rclcpp::ParameterType::PARAMETER_INTEGER ||
      type == rclcpp::ParameterType::PARAMETER_BOOL);
}

template<> bool param_castable<int>(rclcpp::ParameterType type)
{
  return (
      type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
      type == rclcpp::ParameterType::PARAMETER_INTEGER ||
      type == rclcpp::ParameterType::PARAMETER_BOOL);
}

template<> bool param_castable<bool>(rclcpp::ParameterType type)
{
  return (
      type == rclcpp::ParameterType::PARAMETER_DOUBLE ||
      type == rclcpp::ParameterType::PARAMETER_INTEGER ||
      type == rclcpp::ParameterType::PARAMETER_BOOL);
}

template<> std::string param_cast<std::string>(const rclcpp::Parameter& param)
{
  return param.as_string();
}

template<> double param_cast<double>(const rclcpp::Parameter& param)
{
  switch(param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param.as_double();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(param.as_int());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return static_cast<double>(param.as_bool());
    default:
      return 0.0;
  }
}

template<> float param_cast<float>(const rclcpp::Parameter& param)
{
  switch(param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<float>(param.as_double());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<float>(param.as_int());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return static_cast<float>(param.as_bool());
    default:
      return 0.0f;
  }
}

template<> int param_cast<int>(const rclcpp::Parameter& param)
{
  switch(param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<int>(param.as_double());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<int>(param.as_int());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return static_cast<int>(param.as_bool());
    default:
      return 0;
  }
}

template<> bool param_cast<bool>(const rclcpp::Parameter& param)
{
  switch(param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<bool>(param.as_double());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<bool>(param.as_int());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return param.as_bool();
    default:
      return false;
  }
}

template <class T>
  bool getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  rclcpp::Parameter p;
  if (!getImpl(key, p, cached)) {
    return false;
  }

  // For vector types, ROS2 has native array parameter types
  try
  {
    if constexpr (std::is_same_v<T, std::string>)
    {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        return false;
      vec = p.as_string_array();
    }
    else if constexpr (std::is_same_v<T, double>)
    {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
      {
        vec = p.as_double_array();
      }
      else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
      {
        auto ivec = p.as_integer_array();
        vec.resize(ivec.size());
        for (size_t i = 0; i < ivec.size(); i++)
          vec[i] = static_cast<double>(ivec[i]);
      }
      else
      {
        return false;
      }
    }
    else if constexpr (std::is_same_v<T, float>)
    {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
      {
        auto dvec = p.as_double_array();
        vec.resize(dvec.size());
        for (size_t i = 0; i < dvec.size(); i++)
          vec[i] = static_cast<float>(dvec[i]);
      }
      else if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
      {
        auto ivec = p.as_integer_array();
        vec.resize(ivec.size());
        for (size_t i = 0; i < ivec.size(); i++)
          vec[i] = static_cast<float>(ivec[i]);
      }
      else
      {
        return false;
      }
    }
    else if constexpr (std::is_same_v<T, int>)
    {
      if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
      {
        auto ivec = p.as_integer_array();
        vec.resize(ivec.size());
        for (size_t i = 0; i < ivec.size(); i++)
          vec[i] = static_cast<int>(ivec[i]);
      }
      else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
      {
        auto dvec = p.as_double_array();
        vec.resize(dvec.size());
        for (size_t i = 0; i < dvec.size(); i++)
        {
          double d = dvec[i];
          if (fmod(d, 1.0) < 0.5)
            d = floor(d);
          else
            d = ceil(d);
          vec[i] = static_cast<int>(d);
        }
      }
      else
      {
        return false;
      }
    }
    else if constexpr (std::is_same_v<T, bool>)
    {
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY)
        return false;
      vec = p.as_bool_array();
    }
    else
    {
      return false;
    }
  }
  catch (const rclcpp::ParameterTypeException &)
  {
    return false;
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
  // In ROS2, map parameters are stored as individual sub-parameters with "key.subkey" naming
  auto node = getNode();

  // List parameters with the given prefix
  auto list_result = node->list_parameters({key}, 2);

  if (list_result.names.empty())
  {
    return false;
  }

  map.clear();
  std::string prefix = key + ".";

  for (const auto& param_name : list_result.names)
  {
    if (param_name.find(prefix) == 0)
    {
      std::string sub_key = param_name.substr(prefix.size());
      // Only get direct children (no further dots)
      if (sub_key.find('.') != std::string::npos)
        continue;

      T value;
      if (getImpl(param_name, value, cached))
      {
        map[sub_key] = value;
      }
      else
      {
        return false;
      }
    }
  }

  return !map.empty();
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
  auto node = getNode();
  auto list_result = node->list_parameters({}, 0);
  keys = list_result.names;
  return true;
}

bool search(const std::string& key, std::string& result_out)
{
  auto node = getNode();
  return search(node->get_name(), key, result_out);
}

bool search(const std::string& ns, const std::string& key, std::string& result_out)
{
  auto node = getNode();

  // In ROS2, search upward through namespaces for the parameter
  std::string search_ns = ns;

  while (true)
  {
    std::string candidate = search_ns.empty() ? key : (search_ns + "." + key);

    if (node->has_parameter(candidate))
    {
      result_out = candidate;
      return true;
    }

    // Move up one namespace level
    size_t pos = search_ns.rfind('/');
    if (pos == std::string::npos || search_ns.empty())
    {
      break;
    }
    search_ns = search_ns.substr(0, pos);
  }

  // Try just the key itself
  if (node->has_parameter(key))
  {
    result_out = key;
    return true;
  }

  return false;
}

void update(const std::string& key, const rclcpp::ParameterValue& v)
{
  // Apply a remote parameter update to the local cache and invalidate
  // affected parent namespace entries so that hierarchical lookups stay consistent.
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

  if (g_subscribed_params.find(key) != g_subscribed_params.end())
  {
    g_params[key] = rclcpp::Parameter(key, v);
  }
  invalidateParentParams(key);
}

void paramUpdateCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (const auto& changed_param : event->changed_parameters)
  {
    rclcpp::Parameter p = rclcpp::Parameter::from_parameter_msg(changed_param);
    param::update(changed_param.name, p.get_parameter_value());
  }
  for (const auto& new_param : event->new_parameters)
  {
    rclcpp::Parameter p = rclcpp::Parameter::from_parameter_msg(new_param);
    param::update(new_param.name, p.get_parameter_value());
  }
}

void unsubscribeCachedParam(const std::string& key)
{
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
  g_subscribed_params.erase(key);
  g_params.erase(key);
}

void unsubscribeCachedParam(void)
{
  // lock required, all of the cached parameter will be unsubscribed.
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

  for (auto itr = g_subscribed_params.begin();
    itr != g_subscribed_params.end(); ++itr)
  {
    const std::string mapped_key(*itr);
    g_params.erase(mapped_key);
  }
  g_subscribed_params.clear();
}

void init(const std::map<std::string, std::string>& remappings)
{
  auto node = getNode();

  for (auto it = remappings.begin(); it != remappings.end(); ++it)
  {
    const std::string& name = it->first;
    const std::string& param_val = it->second;

    if (name.size() < 2)
    {
      continue;
    }

    if (name[0] == '_' && name[1] != '_')
    {
      std::string local_name = name.substr(1);

      bool success = false;

      try
      {
        int32_t i = std::stoi(param_val);
        param::set(local_name, i);
        success = true;
      }
      catch (const std::invalid_argument&)
      {
      }
      catch (const std::out_of_range&)
      {
      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = std::stod(param_val);
        param::set(local_name, d);
        success = true;
      }
      catch (const std::invalid_argument&)
      {
      }
      catch (const std::out_of_range&)
      {
      }

      if (success)
      {
        continue;
      }

      if (param_val == "true" || param_val == "True" || param_val == "TRUE")
      {
        param::set(local_name, true);
      }
      else if (param_val == "false" || param_val == "False" || param_val == "FALSE")
      {
        param::set(local_name, false);
      }
      else
      {
        param::set(local_name, param_val);
      }
    }
  }
}

} // namespace param

} // namespace ros2_param
```