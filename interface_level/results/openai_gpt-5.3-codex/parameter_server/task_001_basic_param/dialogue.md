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
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/thread/recursive_mutex.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <map>
#include <set>
#include <string>
#include <mutex>
#include <cmath>
#include <algorithm>

namespace ros
{

using M_string = std::map<std::string, std::string>;
using S_string = std::set<std::string>;

namespace
{
rclcpp::Node::SharedPtr g_node;
rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr g_event_sub;
std::recursive_mutex g_node_mutex;

std::map<std::string, XmlRpc::XmlRpcValue> g_parameter_store;
std::recursive_mutex g_parameter_store_mutex;

std::string to_ros2_param_name(const std::string& key)
{
  std::string out = key;
  if (!out.empty() && out[0] == '/')
  {
    out.erase(out.begin());
  }
  std::replace(out.begin(), out.end(), '/', '.');
  while (!out.empty() && out.front() == '.')
  {
    out.erase(out.begin());
  }
  return out;
}

std::string to_ros1_key(const std::string& ros2_name)
{
  std::string out = ros2_name;
  std::replace(out.begin(), out.end(), '.', '/');
  if (out.empty() || out[0] != '/')
  {
    out = "/" + out;
  }
  return out;
}

bool xmlrpc_to_parameter_value(const XmlRpc::XmlRpcValue& in, rclcpp::ParameterValue& out)
{
  using Type = XmlRpc::XmlRpcValue::Type;
  switch (in.getType())
  {
    case Type::TypeBoolean:
      out = rclcpp::ParameterValue(static_cast<bool>(in));
      return true;
    case Type::TypeInt:
      out = rclcpp::ParameterValue(static_cast<int64_t>(static_cast<int>(in)));
      return true;
    case Type::TypeDouble:
      out = rclcpp::ParameterValue(static_cast<double>(in));
      return true;
    case Type::TypeString:
      out = rclcpp::ParameterValue(static_cast<std::string>(in));
      return true;
    case Type::TypeArray:
    {
      if (in.size() == 0)
      {
        out = rclcpp::ParameterValue(std::vector<std::string>{});
        return true;
      }

      const auto t = in[0].getType();
      bool homogeneous = true;
      for (int i = 1; i < in.size(); ++i)
      {
        if (in[i].getType() != t)
        {
          homogeneous = false;
          break;
        }
      }
      if (!homogeneous)
      {
        return false;
      }

      if (t == Type::TypeBoolean)
      {
        std::vector<bool> v;
        v.reserve(in.size());
        for (int i = 0; i < in.size(); ++i) v.push_back(static_cast<bool>(in[i]));
        out = rclcpp::ParameterValue(v);
        return true;
      }
      if (t == Type::TypeInt)
      {
        std::vector<int64_t> v;
        v.reserve(in.size());
        for (int i = 0; i < in.size(); ++i) v.push_back(static_cast<int64_t>(static_cast<int>(in[i])));
        out = rclcpp::ParameterValue(v);
        return true;
      }
      if (t == Type::TypeDouble)
      {
        std::vector<double> v;
        v.reserve(in.size());
        for (int i = 0; i < in.size(); ++i) v.push_back(static_cast<double>(in[i]));
        out = rclcpp::ParameterValue(v);
        return true;
      }
      if (t == Type::TypeString)
      {
        std::vector<std::string> v;
        v.reserve(in.size());
        for (int i = 0; i < in.size(); ++i) v.push_back(static_cast<std::string>(in[i]));
        out = rclcpp::ParameterValue(v);
        return true;
      }
      return false;
    }
    default:
      return false;
  }
}

XmlRpc::XmlRpcValue parameter_value_to_xmlrpc(const rclcpp::ParameterValue& pv)
{
  XmlRpc::XmlRpcValue out;
  switch (pv.get_type())
  {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      out = pv.get<bool>();
      break;
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      out = static_cast<int>(pv.get<int64_t>());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      out = pv.get<double>();
      break;
    case rclcpp::ParameterType::PARAMETER_STRING:
      out = pv.get<std::string>();
      break;
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
    {
      auto a = pv.get<std::vector<bool>>();
      out.setSize(static_cast<int>(a.size()));
      for (size_t i = 0; i < a.size(); ++i) out[static_cast<int>(i)] = a[i];
      break;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
    {
      auto a = pv.get<std::vector<int64_t>>();
      out.setSize(static_cast<int>(a.size()));
      for (size_t i = 0; i < a.size(); ++i) out[static_cast<int>(i)] = static_cast<int>(a[i]);
      break;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
    {
      auto a = pv.get<std::vector<double>>();
      out.setSize(static_cast<int>(a.size()));
      for (size_t i = 0; i < a.size(); ++i) out[static_cast<int>(i)] = a[i];
      break;
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
    {
      auto a = pv.get<std::vector<std::string>>();
      out.setSize(static_cast<int>(a.size()));
      for (size_t i = 0; i < a.size(); ++i) out[static_cast<int>(i)] = a[i];
      break;
    }
    default:
      break;
  }
  return out;
}

rclcpp::Node::SharedPtr ensure_node()
{
  std::lock_guard<std::recursive_mutex> lock(g_node_mutex);
  if (!rclcpp::is_initialized())
  {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  if (!g_node)
  {
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    g_node = std::make_shared<rclcpp::Node>("ros1_param_compat", options);
  }

  return g_node;
}

void set_master_result(XmlRpc::XmlRpcValue& result, const XmlRpc::XmlRpcValue& payload, bool ok, const std::string& msg = "")
{
  result[0] = ok ? 1 : 0;
  result[1] = msg;
  result[2] = payload;
}
}  // namespace

namespace names
{
static M_string g_unresolved_remappings;

std::string parentNamespace(const std::string& ns)
{
  if (ns.empty() || ns == "/")
  {
    return "/";
  }

  std::string tmp = ns;
  if (tmp.size() > 1 && tmp.back() == '/')
  {
    tmp.pop_back();
  }

  auto pos = tmp.find_last_of('/');
  if (pos == std::string::npos || pos == 0)
  {
    return "/";
  }
  return tmp.substr(0, pos);
}

std::string resolve(const std::string& key)
{
  auto node = ensure_node();

  if (key.empty())
  {
    return node->get_fully_qualified_name();
  }

  if (key[0] == '/')
  {
    return key;
  }

  std::string ns = node->get_namespace();
  if (ns.empty()) ns = "/";
  if (ns.back() != '/') ns += '/';

  if (key[0] == '~')
  {
    std::string n = node->get_fully_qualified_name();
    if (n.empty() || n.back() != '/') n += '/';
    return n + key.substr(1);
  }

  return ns + key;
}

const M_string& getUnresolvedRemappings()
{
  return g_unresolved_remappings;
}

void setUnresolvedRemappings(const M_string& remappings)
{
  g_unresolved_remappings = remappings;
}
}  // namespace names

namespace this_node
{
std::string getName()
{
  return ensure_node()->get_fully_qualified_name();
}
}  // namespace this_node

class XMLRPCManager
{
public:
  static XMLRPCManager* instance()
  {
    static XMLRPCManager m;
    return &m;
  }

  std::string getServerURI() const
  {
    return "ros2://parameter_events";
  }

  void bind(const std::string&, void(*)(XmlRpc::XmlRpcValue&, XmlRpc::XmlRpcValue&))
  {
  }
};

namespace master
{
bool execute(const std::string& method, XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result, XmlRpc::XmlRpcValue& payload, bool)
{
  auto node = ensure_node();

  if (method == "setParam")
  {
    std::string key = static_cast<std::string>(params[1]);
    XmlRpc::XmlRpcValue value = params[2];

    {
      std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
      g_parameter_store[key] = value;
    }

    rclcpp::ParameterValue pv;
    if (xmlrpc_to_parameter_value(value, pv))
    {
      std::string p_name = to_ros2_param_name(key);
      if (!node->has_parameter(p_name))
      {
        node->declare_parameter(p_name, pv);
      }
      else
      {
        node->set_parameter(rclcpp::Parameter(p_name, pv));
      }
    }

    payload = 1;
    set_master_result(result, payload, true);
    return true;
  }

  if (method == "getParam")
  {
    std::string key = static_cast<std::string>(params[1]);
    {
      std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
      auto it = g_parameter_store.find(key);
      if (it != g_parameter_store.end())
      {
        payload = it->second;
        set_master_result(result, payload, true);
        return true;
      }
    }

    const std::string p_name = to_ros2_param_name(key);
    rclcpp::Parameter p;
    if (node->get_parameter(p_name, p))
    {
      payload = parameter_value_to_xmlrpc(p.get_parameter_value());
      {
        std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
        g_parameter_store[key] = payload;
      }
      set_master_result(result, payload, true);
      return true;
    }

    payload = XmlRpc::XmlRpcValue();
    set_master_result(result, payload, false, "parameter not set");
    return false;
  }

  if (method == "hasParam")
  {
    std::string key = static_cast<std::string>(params[1]);
    bool found = false;
    {
      std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
      found = g_parameter_store.find(key) != g_parameter_store.end();
    }

    if (!found)
    {
      found = node->has_parameter(to_ros2_param_name(key));
    }

    payload = found;
    set_master_result(result, payload, true);
    return true;
  }

  if (method == "deleteParam")
  {
    std::string key = static_cast<std::string>(params[1]);
    {
      std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
      g_parameter_store.erase(key);
    }

    node->undeclare_parameter(to_ros2_param_name(key));
    payload = 1;
    set_master_result(result, payload, true);
    return true;
  }

  if (method == "getParamNames")
  {
    XmlRpc::XmlRpcValue arr;
    {
      std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
      arr.setSize(static_cast<int>(g_parameter_store.size()));
      int idx = 0;
      for (const auto& kv : g_parameter_store)
      {
        arr[idx++] = kv.first;
      }
    }
    payload = arr;
    set_master_result(result, payload, true);
    return true;
  }

  if (method == "searchParam")
  {
    std::string ns = static_cast<std::string>(params[0]);
    std::string key = static_cast<std::string>(params[1]);

    if (!key.empty() && key[0] == '/')
    {
      std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
      auto it = g_parameter_store.find(key);
      if (it != g_parameter_store.end())
      {
        payload = key;
        set_master_result(result, payload, true);
        return true;
      }
      set_master_result(result, payload, false);
      return false;
    }

    std::string cur = ns.empty() ? "/" : ns;
    if (cur[0] != '/') cur = "/" + cur;
    bool found = false;
    std::string found_key;

    while (true)
    {
      std::string candidate = cur;
      if (candidate.back() != '/') candidate += "/";
      candidate += key;

      {
        std::lock_guard<std::recursive_mutex> lock(g_parameter_store_mutex);
        if (g_parameter_store.find(candidate) != g_parameter_store.end())
        {
          found = true;
          found_key = candidate;
          break;
        }
      }

      if (cur == "/") break;
      cur = names::parentNamespace(cur);
    }

    if (!found)
    {
      set_master_result(result, payload, false);
      return false;
    }

    payload = found_key;
    set_master_result(result, payload, true);
    return true;
  }

  if (method == "subscribeParam")
  {
    XmlRpc::XmlRpcValue get_params, get_result, get_payload;
    get_params[0] = params[0];
    get_params[1] = params[2];
    bool ok = execute("getParam", get_params, get_result, get_payload, false);
    if (ok)
    {
      payload = get_payload;
      set_master_result(result, payload, true);
      return true;
    }

    payload = XmlRpc::XmlRpcValue();
    set_master_result(result, payload, false);
    return false;
  }

  if (method == "unsubscribeParam")
  {
    payload = 1;
    set_master_result(result, payload, true);
    return true;
  }

  set_master_result(result, payload, false, "unknown method");
  return false;
}
}  // namespace master

namespace param
{

typedef std::map<std::string, XmlRpc::XmlRpcValue> M_Param;
M_Param g_params;
boost::recursive_mutex g_params_mutex;
S_string g_subscribed_params;

void unsubscribeCachedParam(const std::string& key);
bool search(const std::string& ns, const std::string& key, std::string& result_out);

void invalidateParentParams(const std::string& key)
{
  std::string parent = ros::names::parentNamespace(key);
  while (!parent.empty())
  {
    g_params.erase(parent);
    if (parent == "/")
    {
      break;
    }
    parent = ros::names::parentNamespace(parent);
  }
}

void set(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string mapped_key = ros::names::resolve(key);

  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  params[2] = v;

  {
    boost::recursive_mutex::scoped_lock lock(g_params_mutex);

    if (master::execute("setParam", params, result, payload, true))
    {
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
  XmlRpc::XmlRpcValue v(s);
  ros::param::set(key, v);
}

void set(const std::string& key, const char* s)
{
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
  XmlRpc::XmlRpcValue xml_vec;
  xml_vec.setSize(vec.size());

  for(size_t i=0; i < vec.size(); i++) {
    xml_vec[i] = vec.at(i);
  }

  ros::param::set(key, xml_vec);
}

void set(const std::string& key, const std::vector<std::string>& vec) { setImpl(key, vec); }
void set(const std::string& key, const std::vector<double>& vec) { setImpl(key, vec); }
void set(const std::string& key, const std::vector<float>& vec) { setImpl(key, vec); }
void set(const std::string& key, const std::vector<int>& vec) { setImpl(key, vec); }
void set(const std::string& key, const std::vector<bool>& vec) { setImpl(key, vec); }

template <class T>
void setImpl(const std::string& key, const std::map<std::string, T>& map)
{
  XmlRpc::XmlRpcValue xml_value;
  xml_value.begin();

  for(typename std::map<std::string, T>::const_iterator it = map.begin(); it != map.end(); ++it) {
    xml_value[it->first] = it->second;
  }

  ros::param::set(key, xml_value);
}

void set(const std::string& key, const std::map<std::string, std::string>& map) { setImpl(key, map); }
void set(const std::string& key, const std::map<std::string, double>& map) { setImpl(key, map); }
void set(const std::string& key, const std::map<std::string, float>& map) { setImpl(key, map); }
void set(const std::string& key, const std::map<std::string, int>& map) { setImpl(key, map); }
void set(const std::string& key, const std::map<std::string, bool>& map) { setImpl(key, map); }

bool has(const std::string& key)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = ros::names::resolve(key);
  if (!master::execute("hasParam", params, result, payload, false))
  {
    return false;
  }

  return static_cast<bool>(payload);
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
  if (!master::execute("deleteParam", params, result, payload, false))
  {
    return false;
  }

  return true;
}

bool getImpl(const std::string& key, XmlRpc::XmlRpcValue& v, bool use_cache)
{
  std::string mapped_key = ros::names::resolve(key);
  bool ret = false;

  if (use_cache)
  {
    boost::recursive_mutex::scoped_lock lock(g_params_mutex);

    M_Param::iterator it = g_params.find(mapped_key);
    if (it != g_params.end())
    {
      if (it->second.valid())
      {
        v = it->second;
        return true;
      }
      return false;
    }

    XmlRpc::XmlRpcValue params, result, payload;
    params[0] = this_node::getName();
    params[1] = XMLRPCManager::instance()->getServerURI();
    params[2] = mapped_key;

    if (!master::execute("subscribeParam", params, result, payload, false))
    {
      g_subscribed_params.insert(mapped_key);
      g_params[mapped_key] = XmlRpc::XmlRpcValue();
      return false;
    }

    g_subscribed_params.insert(mapped_key);
    g_params[mapped_key] = payload;
    v = payload;
    ret = true;
  }
  else
  {
    XmlRpc::XmlRpcValue params, result, payload;
    params[0] = this_node::getName();
    params[1] = mapped_key;
    ret = master::execute("getParam", params, result, payload, false);
    if (ret)
    {
      v = payload;
    }
  }

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

bool get(const std::string& key, std::string& s) { return getImpl(key, s, false); }
bool get(const std::string& key, double& d) { return getImpl(key, d, false); }
bool get(const std::string& key, float& f) { return getImpl(key, f, false); }
bool get(const std::string& key, int& i) { return getImpl(key, i, false); }
bool get(const std::string& key, bool& b) { return getImpl(key, b, false); }
bool get(const std::string& key, XmlRpc::XmlRpcValue& v) { return getImpl(key, v, false); }

bool getCached(const std::string& key, std::string& s) { return getImpl(key, s, true); }
bool getCached(const std::string& key, double& d) { return getImpl(key, d, true); }
bool getCached(const std::string& key, float& f) { return getImpl(key, f, true); }
bool getCached(const std::string& key, int& i) { return getImpl(key, i, true); }
bool getCached(const std::string& key, bool& b) { return getImpl(key, b, true); }
bool getCached(const std::string& key, XmlRpc::XmlRpcValue& v) { return getImpl(key, v, true); }

template <class T> T xml_cast(XmlRpc::XmlRpcValue xml_value) { return static_cast<T>(xml_value); }
template <class T> bool xml_castable(int) { return false; }

template<> bool xml_castable<std::string>(int XmlType) { return XmlType == XmlRpc::XmlRpcValue::TypeString; }
template<> bool xml_castable<double>(int XmlType)
{
  return (XmlType == XmlRpc::XmlRpcValue::TypeDouble || XmlType == XmlRpc::XmlRpcValue::TypeInt || XmlType == XmlRpc::XmlRpcValue::TypeBoolean);
}
template<> bool xml_castable<float>(int XmlType)
{
  return (XmlType == XmlRpc::XmlRpcValue::TypeDouble || XmlType == XmlRpc::XmlRpcValue::TypeInt || XmlType == XmlRpc::XmlRpcValue::TypeBoolean);
}
template<> bool xml_castable<int>(int XmlType)
{
  return (XmlType == XmlRpc::XmlRpcValue::TypeDouble || XmlType == XmlRpc::XmlRpcValue::TypeInt || XmlType == XmlRpc::XmlRpcValue::TypeBoolean);
}
template<> bool xml_castable<bool>(int XmlType)
{
  return (XmlType == XmlRpc::XmlRpcValue::TypeDouble || XmlType == XmlRpc::XmlRpcValue::TypeInt || XmlType == XmlRpc::XmlRpcValue::TypeBoolean);
}

template<> double xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble: return static_cast<double>(xml_value);
    case XmlRpcValue::TypeInt: return static_cast<double>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean: return static_cast<double>(static_cast<bool>(xml_value));
    default: return 0.0;
  };
}
template<> float xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble: return static_cast<float>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt: return static_cast<float>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean: return static_cast<float>(static_cast<bool>(xml_value));
    default: return 0.0f;
  };
}
template<> int xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble: return static_cast<int>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt: return static_cast<int>(xml_value);
    case XmlRpcValue::TypeBoolean: return static_cast<int>(static_cast<bool>(xml_value));
    default: return 0;
  };
}
template<> bool xml_cast(XmlRpc::XmlRpcValue xml_value)
{
  using namespace XmlRpc;
  switch(xml_value.getType()) {
    case XmlRpcValue::TypeDouble: return static_cast<bool>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt: return static_cast<bool>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean: return static_cast<bool>(xml_value);
    default: return false;
  };
}

template <class T>
bool getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  XmlRpc::XmlRpcValue xml_array;
  if(!getImpl(key, xml_array, cached)) {
    return false;
  }

  if(xml_array.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }

  vec.resize(xml_array.size());

  for (int i = 0; i < xml_array.size(); i++) {
    if(!xml_castable<T>(xml_array[i].getType())) {
      return false;
    }

    vec[i] = xml_cast<T>(xml_array[i]);
  }

  return true;
}

bool get(const std::string& key, std::vector<std::string>& vec) { return getImpl(key, vec, false); }
bool get(const std::string& key, std::vector<double>& vec) { return getImpl(key, vec, false); }
bool get(const std::string& key, std::vector<float>& vec) { return getImpl(key, vec, false); }
bool get(const std::string& key, std::vector<int>& vec) { return getImpl(key, vec, false); }
bool get(const std::string& key, std::vector<bool>& vec) { return getImpl(key, vec, false); }

bool getCached(const std::string& key, std::vector<std::string>& vec) { return getImpl(key, vec, true); }
bool getCached(const std::string& key, std::vector<double>& vec) { return getImpl(key, vec, true); }
bool getCached(const std::string& key, std::vector<float>& vec) { return getImpl(key, vec, true); }
bool getCached(const std::string& key, std::vector<int>& vec) { return getImpl(key, vec, true); }
bool getCached(const std::string& key, std::vector<bool>& vec) { return getImpl(key, vec, true); }

template <class T>
bool getImpl(const std::string& key, std::map<std::string, T>& map, bool cached)
{
  XmlRpc::XmlRpcValue xml_value;
  if(!getImpl(key, xml_value, cached)) {
    return false;
  }

  if(xml_value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }

  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = xml_value.begin(); it != xml_value.end(); ++it)
  {
    if(!xml_castable<T>(it->second.getType())) {
      return false;
    }
    map[it->first] = xml_cast<T>(it->second);
  }

  return true;
}

bool get(const std::string& key, std::map<std::string, std::string>& map) { return getImpl(key, map, false); }
bool get(const std::string& key, std::map<std::string, double>& map) { return getImpl(key, map, false); }
bool get(const std::string& key, std::map<std::string, float>& map) { return getImpl(key, map, false); }
bool get(const std::string& key, std::map<std::string, int>& map) { return getImpl(key, map, false); }
bool get(const std::string& key, std::map<std::string, bool>& map) { return getImpl(key, map, false); }

bool getCached(const std::string& key, std::map<std::string, std::string>& map) { return getImpl(key, map, true); }
bool getCached(const std::string& key, std::map<std::string, double>& map) { return getImpl(key, map, true); }
bool getCached(const std::string& key, std::map<std::string, float>& map) { return getImpl(key, map, true); }
bool getCached(const std::string& key, std::map<std::string, int>& map) { return getImpl(key, map, true); }
bool getCached(const std::string& key, std::map<std::string, bool>& map) { return getImpl(key, map, true); }

bool getParamNames(std::vector<std::string>& keys)
{
  XmlRpc::XmlRpcValue params, result, payload;
  params[0] = this_node::getName();
  if (!master::execute("getParamNames", params, result, payload, false)) {
    return false;
  }
  if (result.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  if (result.size() != 3) {
    return false;
  }
  XmlRpc::XmlRpcValue parameters = result[2];
  keys.resize(parameters.size());

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

  std::string remapped = key;
  M_string::const_iterator it = names::getUnresolvedRemappings().find(key);
  if (it != names::getUnresolvedRemappings().end())
  {
    remapped = it->second;
  }

  params[1] = remapped;
  if (!master::execute("searchParam", params, result, payload, false))
  {
    return false;
  }

  result_out = (std::string)payload;
  return true;
}

void update(const std::string& key, const XmlRpc::XmlRpcValue& v)
{
  std::string mapped_key = ros::names::resolve(key);
  boost::recursive_mutex::scoped_lock lock(g_params_mutex);
  g_params[mapped_key] = v;
  invalidateParentParams(mapped_key);
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
  boost::recursive_mutex::scoped_lock lock(g_params_mutex);

  for(S_string::iterator itr = g_subscribed_params.begin(); itr != g_subscribed_params.end(); ++itr)
  {
    const std::string mapped_key(*itr);
    unsubscribeCachedParam(mapped_key);
  }
}

void init(const M_string& remappings)
{
  names::setUnresolvedRemappings(remappings);

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

  auto node = ensure_node();
  if (!g_event_sub)
  {
    g_event_sub = node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events",
      rclcpp::QoS(100),
      [](const rcl_interfaces::msg::ParameterEvent::SharedPtr msg)
      {
        for (const auto& p : msg->new_parameters)
        {
          std::string key = to_ros1_key(p.name);
          std::string fq = names::resolve(key);
          boost::recursive_mutex::scoped_lock lock(g_params_mutex);
          if (g_subscribed_params.find(fq) != g_subscribed_params.end())
          {
            update(fq, parameter_value_to_xmlrpc(rclcpp::ParameterValue(p.value)));
          }
        }
        for (const auto& p : msg->changed_parameters)
        {
          std::string key = to_ros1_key(p.name);
          std::string fq = names::resolve(key);
          boost::recursive_mutex::scoped_lock lock(g_params_mutex);
          if (g_subscribed_params.find(fq) != g_subscribed_params.end())
          {
            update(fq, parameter_value_to_xmlrpc(rclcpp::ParameterValue(p.value)));
          }
        }
      });
  }

  XMLRPCManager::instance()->bind("paramUpdate", paramUpdateCallback);
}

} // namespace param
} // namespace ros
```