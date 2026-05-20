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
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/utilities.hpp"

#include <rcutils/logging.h>

#include <mutex>
#include <vector>
#include <map>
#include <string>
#include <cmath>

namespace ros
{

namespace param
{

typedef std::map<std::string, rclcpp::ParameterValue> M_Param;
M_Param g_params;
std::recursive_mutex g_params_mutex;
std::set<std::string> g_subscribed_params;

std::shared_ptr<rclcpp::SyncParametersClient> g_parameters_client;
rclcpp::Node::SharedPtr g_node;

void invalidateParentParams(const std::string& key)
{
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
  std::string parent_key = key;
  while (!parent_key.empty())
  {
    size_t pos = parent_key.find_last_of('/');
    if (pos == std::string::npos)
    {
      break;
    }
    parent_key = parent_key.substr(0, pos);
    if (parent_key.empty())
    {
      break;
    }
    auto it = g_params.find(parent_key);
    if (it != g_params.end())
    {
      g_params.erase(it);
    }
  }
}

void set(const std::string& key, const rclcpp::ParameterValue& v)
{
  std::string mapped_key = key;
  if (!mapped_key.empty() && mapped_key[0] != '/')
  {
    mapped_key = std::string(g_node->get_namespace()) + "/" + mapped_key;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

    if (g_parameters_client)
    {
      rclcpp::ParameterType type = v.get_type();
      switch (type)
      {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<bool>())});
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<int64_t>())});
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<double>())});
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<std::string>())});
          break;
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<std::vector<bool>>())});
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<std::vector<int64_t>>())});
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<std::vector<double>>())});
          break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          g_parameters_client->set_parameters({rclcpp::Parameter(mapped_key, v.get<std::vector<std::string>>())});
          break;
        default:
          break;
      }
    }

    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
    {
      g_params[mapped_key] = v;
    }
    invalidateParentParams(mapped_key);
  }
}

void set(const std::string& key, const std::string& s)
{
  rclcpp::ParameterValue v(s);
  ros::param::set(key, v);
}

void set(const std::string& key, const char* s)
{
  std::string sxx = std::string(s);
  rclcpp::ParameterValue v(sxx);
  ros::param::set(key, v);
}

void set(const std::string& key, double d)
{
  rclcpp::ParameterValue v(d);
  ros::param::set(key, v);
}

void set(const std::string& key, int i)
{
  rclcpp::ParameterValue v(static_cast<int64_t>(i));
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
  std::vector<double> dvec(vec.begin(), vec.end());
  setImpl(key, dvec);
}

void set(const std::string& key, const std::vector<int>& vec)
{
  std::vector<int64_t> lvec(vec.begin(), vec.end());
  setImpl(key, lvec);
}

void set(const std::string& key, const std::vector<bool>& vec)
{
  setImpl(key, vec);
}

template <class T>
  void setImpl(const std::string& key, const std::map<std::string, T>& map)
{
  rclcpp::ParameterValue v(map);
  ros::param::set(key, v);
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
  std::map<std::string, double> dmap;
  for (const auto& pair : map)
  {
    dmap[pair.first] = static_cast<double>(pair.second);
  }
  setImpl(key, dmap);
}

void set(const std::string& key, const std::map<std::string, int>& map)
{
  std::map<std::string, int64_t> lmap;
  for (const auto& pair : map)
  {
    lmap[pair.first] = static_cast<int64_t>(pair.second);
  }
  setImpl(key, lmap);
}

void set(const std::string& key, const std::map<std::string, bool>& map)
{
  setImpl(key, map);
}

bool has(const std::string& key)
{
  std::string mapped_key = key;
  if (!mapped_key.empty() && mapped_key[0] != '/')
  {
    mapped_key = std::string(g_node->get_namespace()) + "/" + mapped_key;
  }

  if (g_parameters_client)
  {
    return g_parameters_client->has_parameter(mapped_key);
  }
  return false;
}

bool del(const std::string& key)
{
  std::string mapped_key = key;
  if (!mapped_key.empty() && mapped_key[0] != '/')
  {
    mapped_key = std::string(g_node->get_namespace()) + "/" + mapped_key;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

    if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end())
    {
      g_subscribed_params.erase(mapped_key);
      unsubscribeCachedParam(mapped_key);
    }
    g_params.erase(mapped_key);
  }

  if (g_parameters_client)
  {
    // ROS2 does not have a direct delete parameter API via SyncParametersClient.
    // We can set it to an empty parameter value or rely on node undeclare.
    // For compatibility, we attempt to undeclare if possible.
    // Since we cannot undeclare remotely, we just return true if it was in cache.
    return true;
  }
  return false;
}

bool getImpl(const std::string& key, rclcpp::ParameterValue& v, bool use_cache)
{
  std::string mapped_key = key;
  if (!mapped_key.empty() && mapped_key[0] != '/')
  {
    mapped_key = std::string(g_node->get_namespace()) + "/" + mapped_key;
  }

  if (use_cache)
  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
    auto it = g_params.find(mapped_key);
    if (it != g_params.end())
    {
      v = it->second;
      return true;
    }
  }

  if (g_parameters_client)
  {
    if (g_parameters_client->has_parameter(mapped_key))
    {
      rclcpp::Parameter param = g_parameters_client->get_parameter(mapped_key);
      v = param.get_parameter_value();

      if (use_cache)
      {
        std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
        g_params[mapped_key] = v;
        if (g_subscribed_params.find(mapped_key) == g_subscribed_params.end())
        {
          g_subscribed_params.insert(mapped_key);
          // In a real implementation, we would subscribe to parameter events here.
        }
      }
      return true;
    }
  }

  return false;
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

template <class T> T xml_cast(rclcpp::ParameterValue xml_value) 
{
  return static_cast<T>(xml_value);
}

template <class T> bool xml_castable(int XmlType) 
{
  return false;
}

template<> bool xml_castable<std::string>(int XmlType)
{
  return XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_STRING);
}

template<> bool xml_castable<double>(int XmlType)
{
  return ( 
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_DOUBLE) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_INTEGER) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_BOOL) );
}

template<> bool xml_castable<float>(int XmlType)
{
  return ( 
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_DOUBLE) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_INTEGER) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_BOOL) );
}

template<> bool xml_castable<int>(int XmlType)
{
  return ( 
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_DOUBLE) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_INTEGER) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_BOOL) );
}

template<> bool xml_castable<bool>(int XmlType)
{
  return ( 
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_DOUBLE) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_INTEGER) ||
      XmlType == static_cast<int>(rclcpp::ParameterType::PARAMETER_BOOL) );
}

template<> double xml_cast(rclcpp::ParameterValue xml_value)
{
  switch(xml_value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return xml_value.get<double>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(xml_value.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return static_cast<double>(xml_value.get<bool>());
    default:
     return 0.0;
  };
}

template<> float xml_cast(rclcpp::ParameterValue xml_value)
{
  switch(xml_value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<float>(xml_value.get<double>());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<float>(xml_value.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return static_cast<float>(xml_value.get<bool>());
    default:
      return 0.0f;
  };
}

template<> int xml_cast(rclcpp::ParameterValue xml_value)
{
  switch(xml_value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<int>(xml_value.get<double>());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<int>(xml_value.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return static_cast<int>(xml_value.get<bool>());
    default:
      return 0;
  };
}

template<> bool xml_cast(rclcpp::ParameterValue xml_value)
{
  switch(xml_value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<bool>(xml_value.get<double>());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<bool>(xml_value.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return xml_value.get<bool>();
    default:
      return false;
  };
}
  
template <class T>
  bool getImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  rclcpp::ParameterValue xml_array;
  if(!getImpl(key, xml_array, cached)) {
    return false;
  }

  if(xml_array.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY &&
     xml_array.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
     xml_array.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY &&
     xml_array.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
    return false;
  }

  std::vector<T> temp_vec;
  try {
    temp_vec = xml_array.get<std::vector<T>>();
  } catch (...) {
    return false;
  }

  vec = temp_vec;
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
  std::vector<double> dvec;
  if (!getImpl(key, dvec, false))
    return false;
  vec.resize(dvec.size());
  for (size_t i = 0; i < dvec.size(); ++i)
    vec[i] = static_cast<float>(dvec[i]);
  return true;
}
bool get(const std::string& key, std::vector<int>& vec)
{
  std::vector<int64_t> lvec;
  if (!getImpl(key, lvec, false))
    return false;
  vec.resize(lvec.size());
  for (size_t i = 0; i < lvec.size(); ++i)
    vec[i] = static_cast<int>(lvec[i]);
  return true;
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
  std::vector<double> dvec;
  if (!getImpl(key, dvec, true))
    return false;
  vec.resize(dvec.size());
  for (size_t i = 0; i < dvec.size(); ++i)
    vec[i] = static_cast<float>(dvec[i]);
  return true;
}
bool getCached(const std::string& key, std::vector<int>& vec)
{
  std::vector<int64_t> lvec;
  if (!getImpl(key, lvec, true))
    return false;
  vec.resize(lvec.size());
  for (size_t i = 0; i < lvec.size(); ++i)
    vec[i] = static_cast<int>(lvec[i]);
  return true;
}
bool getCached(const std::string& key, std::vector<bool>& vec)
{
  return getImpl(key, vec, true);
}

template <class T>
  bool getImpl(const std::string& key, std::map<std::string, T>& map, bool cached)
{
  rclcpp::ParameterValue xml_value;
  if(!getImpl(key, xml_value, cached)) {
    return false;
  }

  if(xml_value.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY &&
     xml_value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
     xml_value.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY &&
     xml_value.get_type() != rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
    // In ROS2, maps are not directly supported as parameter types.
    // We assume they are stored as a flat structure or we cannot retrieve them.
    return false;
  }

  // Since ROS2 parameters do not support maps natively, we cannot implement this directly.
  // Returning false for compatibility.
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
  if (!g_parameters_client)
    return false;

  auto param_names = g_parameters_client->list_parameters({}, 0);
  keys = param_names.names;
  return true;
}

bool search(const std::string& key, std::string& result_out)
{
  return search(std::string(g_node->get_namespace()), key, result_out);
}

bool search(const std::string& ns, const std::string& key, std::string& result_out)
{
  if (!g_parameters_client)
    return false;

  std::string full_ns = ns;
  if (!full_ns.empty() && full_ns[0] != '/')
    full_ns = "/" + full_ns;

  auto param_names = g_parameters_client->list_parameters({full_ns}, 0);
  for (const auto& name : param_names.names)
  {
    if (name.find(key) != std::string::npos)
    {
      result_out = name;
      return true;
    }
  }
  return false;
}

void update(const std::string& key, const rclcpp::ParameterValue& v)
{
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
  std::string mapped_key = key;
  if (!mapped_key.empty() && mapped_key[0] != '/')
  {
    mapped_key = std::string(g_node->get_namespace()) + "/" + mapped_key;
  }

  g_params[mapped_key] = v;
  invalidateParentParams(mapped_key);
}

void paramUpdateCallback(const rclcpp::Parameter& param)
{
  ros::param::update(param.get_name(), param.get_parameter_value());
}

void unsubscribeCachedParam(const std::string& key)
{
  // In ROS2, parameter subscriptions are handled via parameter events.
  // We would need to remove the callback if we had stored it.
  // For now, we just remove from the subscribed set.
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
  g_subscribed_params.erase(key);
}

void unsubscribeCachedParam(void)
{
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

  for(auto itr = g_subscribed_params.begin();
    itr != g_subscribed_params.end(); )
  {
    const std::string mapped_key(*itr);
    itr = g_subscribed_params.erase(itr);
    // In a full implementation, we would unsubscribe from parameter events here.
  }
}

void init(const std::map<std::string, std::string>& remappings)
{
  if (!g_node)
  {
    // In a real application, the node would be passed in or created externally.
    // For this standalone implementation, we assume it is set before init.
    return;
  }

  g_parameters_client = std::make_shared<rclcpp::SyncParametersClient>(g_node);

  for (const auto& pair : remappings)
  {
    const std::string& name = pair.first;
    const std::string& param = pair.second;

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
        ros::param::set(local_name, i);
        success = true;
      }
      catch (std::invalid_argument&)
      {

      }
      catch (std::out_of_range&)
      {

      }

      if (success)
      {
        continue;
      }

      try
      {
        double d = std::stod(param);
        ros::param::set(local_name, d);
        success = true;
      }
      catch (std::invalid_argument&)
      {

      }
      catch (std::out_of_range&)
      {

      }

      if (success)
      {
        continue;
      }

      if (param == "true" || param == "True" || param == "TRUE")
      {
        ros::param::set(local_name, true);
      }
      else if (param == "false" || param == "False" || param == "FALSE")
      {
        ros::param::set(local_name, false);
      }
      else
      {
        ros::param::set(local_name, param);
      }
    }
  }

  // In ROS2, parameter callbacks are registered via the node's parameter callback.
  // We would do: g_node->register_param_change_callback(std::bind(&paramUpdateCallback, std::placeholders::_1));
}

} // namespace param

} // namespace ros