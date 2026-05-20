# Prompt


You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example.
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):

FILE_PATH: param.cpp
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

[FILENAME: param.cpp]
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
#include "rcl_interfaces/srv/delete_parameters.hpp"
#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rcl_interfaces/srv/describe_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

#include <rcutils/logging_macros.h>

#include <mutex>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>

namespace ros
{

namespace param
{

// ROS2 Node pointer for service clients
static std::shared_ptr<rclcpp::Node> g_node = nullptr;
static std::mutex g_node_mutex;

// Cache structures (mimicking ROS1 behavior)
typedef std::map<std::string, rcl_interfaces::msg::ParameterValue> M_Param;
M_Param g_params;
std::mutex g_params_mutex;
std::set<std::string> g_subscribed_params;

// Helper to ensure node is initialized
static std::shared_ptr<rclcpp::Node> get_node()
{
  std::lock_guard<std::mutex> lock(g_node_mutex);
  if (!g_node) {
    g_node = rclcpp::Node::make_shared("param_client");
  }
  return g_node;
}

// Helper to convert XmlRpcValue to ROS2 ParameterValue
// Note: In a full migration, XmlRpcValue usage would be replaced entirely by rcl_interfaces types.
// Here we assume the input 'v' is compatible or we are bridging logic.
// Since the original code uses XmlRpcValue heavily, we will map the logic to ROS2 types internally
// but keep the function signatures as close as possible to the original intent, 
// adapting the internal implementation to use rclcpp services.

// Helper to resolve name (simplified for ROS2 context)
std::string resolve_name(const std::string& key)
{
  if (key.empty()) return "";
  if (key[0] == '~') {
    // Private namespace resolution would require node context
    // For this migration, we assume standard resolution or pass through
    return get_node()->get_namespace() + "/" + get_node()->get_name() + key.substr(1);
  }
  if (key[0] != '/') {
    return get_node()->get_namespace() + "/" + key;
  }
  return key;
}

void invalidateParentParams(const std::string& key)
{
  // Invalidate cached parent namespace entries for a given key.
  // In ROS2, parameters are flat, but we maintain the cache hierarchy logic for compatibility.
  std::lock_guard<std::mutex> lock(g_params_mutex);
  std::string current = key;
  while (!current.empty()) {
    size_t pos = current.find_last_of('/');
    if (pos == std::string::npos) break;
    current = current.substr(0, pos);
    if (current.empty()) current = "/";
    g_params.erase(current);
  }
}

void set(const std::string& key, const rcl_interfaces::msg::ParameterValue& v)
{
  std::string mapped_key = resolve_name(key);

  auto node = get_node();
  auto client = node->create_client<rcl_interfaces::srv::SetParameters>("set_parameters");
  
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node->get_logger(), "Service set_parameters not available");
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter param;
  param.name = mapped_key;
  param.value = v;
  request->parameters.push_back(param);

  auto result = client->async_send_request(request);
  
  // Lock around the update to cache
  {
    std::lock_guard<std::mutex> lock(g_params_mutex);
    // Update cache immediately for consistency if subscribed
    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end()) {
      g_params[mapped_key] = v;
    }
    invalidateParentParams(mapped_key);
  }
}

void set(const std::string& key, const std::string& s)
{
  rcl_interfaces::msg::ParameterValue v;
  v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  v.string_value = s;
  set(key, v);
}

void set(const std::string& key, const char* s)
{
  set(key, std::string(s));
}

void set(const std::string& key, double d)
{
  rcl_interfaces::msg::ParameterValue v;
  v.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  v.double_value = d;
  set(key, v);
}

void set(const std::string& key, int i)
{
  rcl_interfaces::msg::ParameterValue v;
  v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  v.integer_value = i;
  set(key, v);
}

void set(const std::string& key, bool b)
{
  rcl_interfaces::msg::ParameterValue v;
  v.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  v.bool_value = b;
  set(key, v);
}

template <class T>
void setImpl(const std::string& key, const std::vector<T>& vec)
{
  rcl_interfaces::msg::ParameterValue v;
  
  if constexpr (std::is_same<T, std::string>::value) {
    v.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
    v.string_array_value = vec;
  } else if constexpr (std::is_same<T, double>::value) {
    v.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    v.double_array_value = vec;
  } else if constexpr (std::is_same<T, float>::value) {
    v.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
    v.double_array_value.resize(vec.size());
    for(size_t i=0; i<vec.size(); ++i) v.double_array_value[i] = static_cast<double>(vec[i]);
  } else if constexpr (std::is_same<T, int>::value) {
    v.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY;
    v.integer_array_value.resize(vec.size());
    for(size_t i=0; i<vec.size(); ++i) v.integer_array_value[i] = static_cast<int64_t>(vec[i]);
  } else if constexpr (std::is_same<T, bool>::value) {
    v.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY;
    v.bool_array_value = vec;
  }
  
  set(key, v);
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
  // ROS2 parameters do not natively support structs/maps as a single parameter value.
  // We flatten the map into individual parameters: key.subkey
  for(const auto& item : map) {
    std::string sub_key = key + "/" + item.first;
    if constexpr (std::is_same<T, std::string>::value) {
      set(sub_key, item.second);
    } else if constexpr (std::is_same<T, double>::value) {
      set(sub_key, item.second);
    } else if constexpr (std::is_same<T, float>::value) {
      set(sub_key, static_cast<double>(item.second));
    } else if constexpr (std::is_same<T, int>::value) {
      set(sub_key, item.second);
    } else if constexpr (std::is_same<T, bool>::value) {
      set(sub_key, item.second);
    }
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
  auto node = get_node();
  auto client = node->create_client<rcl_interfaces::srv::DescribeParameters>("describe_parameters");
  
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
  request->names.push_back(resolve_name(key));

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    return !result.get()->descriptors.empty();
  }
  return false;
}

bool del(const std::string& key)
{
  std::string mapped_key = resolve_name(key);

  {
    std::lock_guard<std::mutex> lock(g_params_mutex);
    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end()) {
      g_subscribed_params.erase(mapped_key);
      // unsubscribeCachedParam(mapped_key); // Not directly applicable in ROS2 service model without subscriptions
    }
    g_params.erase(mapped_key);
  }

  auto node = get_node();
  auto client = node->create_client<rcl_interfaces::srv::DeleteParameters>("delete_parameters");
  
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::DeleteParameters::Request>();
  request->names.push_back(mapped_key);

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    return result.get()->successful;
  }
  return false;
}

bool getImpl(const std::string& key, rcl_interfaces::msg::ParameterValue& v, bool use_cache)
{
  std::string mapped_key = resolve_name(key);

  if (use_cache) {
    std::lock_guard<std::mutex> lock(g_params_mutex);
    auto it = g_params.find(mapped_key);
    if (it != g_params.end()) {
      v = it->second;
      return true;
    }
  }

  auto node = get_node();
  auto client = node->create_client<rcl_interfaces::srv::GetParameters>("get_parameters");
  
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back(mapped_key);

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    if (!response->values.empty() && response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
      v = response->values[0];
      
      // Update cache if requested
      if (use_cache) {
        std::lock_guard<std::mutex> lock(g_params_mutex);
        g_params[mapped_key] = v;
      }
      return true;
    }
  }
  return false;
}

bool getImpl(const std::string& key, std::string& s, bool use_cache)
{
  rcl_interfaces::msg::ParameterValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
    return false;
  s = v.string_value;
  return true;
}

bool getImpl(const std::string& key, double& d, bool use_cache)
{
  rcl_interfaces::msg::ParameterValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
  {
    d = static_cast<double>(v.integer_value);
  }
  else if (v.type != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
  {
    return false;
  }
  else
  {
    d = v.double_value;
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
  rcl_interfaces::msg::ParameterValue v;
  if (!getImpl(key, v, use_cache))
  {
    return false;
  }

  if (v.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
  {
    double d = v.double_value;
    if (std::fmod(d, 1.0) < 0.5)
    {
      d = std::floor(d);
    }
    else
    {
      d = std::ceil(d);
    }
    i = static_cast<int>(d);
  }
  else if (v.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
  {
    return false;
  }
  else
  {
    i = static_cast<int>(v.integer_value);
  }

  return true;
}

bool getImpl(const std::string& key, bool& b, bool use_cache)
{
  rcl_interfaces::msg::ParameterValue v;
  if (!getImpl(key, v, use_cache))
    return false;
  if (v.type != rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
    return false;
  b = v.bool_value;
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

bool get(const std::string& key, rcl_interfaces::msg::ParameterValue& v)
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

bool getCached(const std::string& key, rcl_interfaces::msg::ParameterValue& v)
{
	return getImpl(key, v, true);
}

template <class T> T xml_cast(rcl_interfaces::msg::ParameterValue xml_value) 
{
  // Placeholder for template specialization logic below
  return T();
}

template <class T> bool xml_castable(int ParamType) 
{
  return false;
}

template<> bool xml_castable<std::string>(int ParamType)
{
  return ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
}

template<> bool xml_castable<double>(int ParamType)
{
  return ( 
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL );
}

template<> bool xml_castable<float>(int ParamType)
{
  return ( 
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL );
}

template<> bool xml_castable<int>(int ParamType)
{
  return ( 
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL );
}

template<> bool xml_castable<bool>(int ParamType)
{
  return ( 
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER ||
      ParamType == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL );
}

template<> double xml_cast(rcl_interfaces::msg::ParameterValue xml_value)
{
  switch(xml_value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return xml_value.double_value;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(xml_value.integer_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      return xml_value.bool_value ? 1.0 : 0.0;
    default:
     return 0.0;
  };
}

template<> float xml_cast(rcl_interfaces::msg::ParameterValue xml_value)
{
  switch(xml_value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return static_cast<float>(xml_value.double_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return static_cast<float>(xml_value.integer_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      return xml_value.bool_value ? 1.0f : 0.0f;
    default:
      return 0.0f;
  };
}

template<> int xml_cast(rcl_interfaces::msg::ParameterValue xml_value)
{
  switch(xml_value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return static_cast<int>(xml_value.double_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return static_cast<int>(xml_value.integer_value);
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      return xml_value.bool_value ? 1 : 0;
    default:
      return 0;
  };
}

template<> bool xml_cast(rcl_interfaces::msg::ParameterValue xml_value)
{
  switch(xml_value.type) {
    case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
      return xml_value.double_value != 0.0;
    case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
      return xml_value.integer_value != 0;
    case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
      return xml_value.bool_value;
    default:
      return false;
  };
}
  
template <class T>
bool getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  rcl_interfaces::msg::ParameterValue xml_array;
  if(!getImpl(key, xml_array, cached)) {
    return false;
  }

  bool is_array = false;
  if constexpr (std::is_same<T, std::string>::value) is_array = (xml_array.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY);
  else if constexpr (std::is_same<T, double>::value) is_array = (xml_array.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
  else if constexpr (std::is_same<T, float>::value) is_array = (xml_array.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY);
  else if constexpr (std::is_same<T, int>::value) is_array = (xml_array.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY);
  else if constexpr (std::is_same<T, bool>::value) is_array = (xml_array.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY);

  if(!is_array) {
    return false;
  }

  if constexpr (std::is_same<T, std::string>::value) {
    vec.assign(xml_array.string_array_value.begin(), xml_array.string_array_value.end());
  } else if constexpr (std::is_same<T, double>::value) {
    vec.assign(xml_array.double_array_value.begin(), xml_array.double_array_value.end());
  } else if constexpr (std::is_same<T, float>::value) {
    vec.resize(xml_array.double_array_value.size());
    for(size_t i=0; i<xml_array.double_array_value.size(); ++i) vec[i] = static_cast<float>(xml_array.double_array_value[i]);
  } else if constexpr (std::is_same<T, int>::value) {
    vec.resize(xml_array.integer_array_value.size());
    for(size_t i=0; i<xml_array.integer_array_value.size(); ++i) vec[i] = static_cast<int>(xml_array.integer_array_value[i]);
  } else if constexpr (std::is_same<T, bool>::value) {
    vec.assign(xml_array.bool_array_value.begin(), xml_array.bool_array_value.end());
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
  // In ROS2, we simulate struct retrieval by listing parameters with the prefix
  auto node = get_node();
  auto client = node->create_client<rcl_interfaces::srv::ListParameters>("list_parameters");
  
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  request->prefixes.push_back(resolve_name(key));
  request->depth = 1; // Only immediate children

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  auto response = result.get();
  if (response->names.empty()) {
    return false;
  }

  for (const auto& name : response->names) {
    // Extract the sub-key
    std::string full_key = name;
    std::string base_key = resolve_name(key);
    
    if (full_key.find(base_key) == 0) {
      std::string sub = full_key.substr(base_key.length());
      if (!sub.empty() && sub[0] == '/') {
        sub = sub.substr(1);
      }
      
      T val;
      if (getImpl(name, val, cached)) {
        map[sub] = val;
      } else {
        return false; // Type mismatch or error
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
  auto node = get_node();
  auto client = node->create_client<rcl_interfaces::srv::ListParameters>("list_parameters");
  
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  // Empty prefix lists all
  auto result = client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    keys.assign(response->names.begin(), response->names.end());
    return true;
  }
  return false;
}

bool search(const std::string& key, std::string& result_out)
{
  return search(get_node()->get_namespace(), key, result_out);
}

bool search(const std::string& ns, const std::string& key, std::string& result_out)
{
  // ROS2 does not have a direct searchParam service equivalent that walks up the hierarchy automatically in the same way.
  // We simulate this by checking the current namespace and parents manually.
  
  std::string current_ns = ns;
  if (current_ns.empty()) current_ns = "/";
  
  // Normalize key
  std::string search_key = key;
  if (search_key[0] == '/') search_key = search_key.substr(1);

  while (true) {
    std::string candidate;
    if (current_ns == "/") {
      candidate = "/" + search_key;
    } else {
      candidate = current_ns + "/" + search_key;
    }

    if (has(candidate)) {
      result_out = candidate;
      return true;
    }

    if (current_ns == "/" || current_ns.empty()) break;
    
    size_t pos = current_ns.find_last_of('/');
    if (pos == 0) {
      current_ns = "/";
    } else if (pos == std::string::npos) {
      current_ns = "";
    } else {
      current_ns = current_ns.substr(0, pos);
    }
  }

  return false;
}

void update(const std::string& key, const rcl_interfaces::msg::ParameterValue& v)
{
  // Apply a remote parameter update to the local cache and invalidate
  // affected parent namespace entries so that hierarchical lookups stay consistent.
  std::lock_guard<std::mutex> lock(g_params_mutex);
  g_params[key] = v;
  invalidateParentParams(key);
}

// Placeholder for callback mechanism (ROS2 uses OnSetParametersCallback or events, not XMLRPC)
void paramUpdateCallback(const std::string& key, const rcl_interfaces::msg::ParameterValue& v)
{
  ros::param::update(key, v);
}

void unsubscribeCachedParam(const std::string& key)
{
  // In ROS2, parameter subscriptions are managed via callbacks on the node.
  // This function is a no-op in the simple service-client model unless using specific event handlers.
  std::lock_guard<std::mutex> lock(g_params_mutex);
  g_subscribed_params.erase(key);
}

void unsubscribeCachedParam(void)
{
  std::lock_guard<std::mutex> lock(g_params_mutex);
  g_subscribed_params.clear();
}

void init(const std::map<std::string, std::string>& remappings)
{
  // Initialize the global node if not already done
  get_node();

  auto it = remappings.begin();
  auto end = remappings.end();
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
        int32_t i = std::stoi(param);
        ros::param::set(resolve_name(local_name), i);
        success = true;
      }
      catch (...)
      {
      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = std::stod(param);
        ros::param::set(resolve_name(local_name), d);
        success = true;
      }
      catch (...)
      {
      }

      if (success)
      {
        continue;
      }

      if (param == "true" || param == "True" || param == "TRUE")
      {
        ros::param::set(resolve_name(local_name), true);
      }
      else if (param == "false" || param == "False" || param == "FALSE")
      {
        ros::param::set(resolve_name(local_name), false);
      }
      else
      {
        ros::param::set(resolve_name(local_name), param);
      }
    }
  }

  // ROS2 parameter updates are handled via node callbacks, not XMLRPC binding.
  // The original XMLRPC binding is removed.
}

} // namespace param

} // namespace ros