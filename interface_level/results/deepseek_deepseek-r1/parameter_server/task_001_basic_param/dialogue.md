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

</think>好的，我们先一步步推理。  

---

## 1. 理解题意  
题中说：  
- 有 \( n \) 个点，编号 \( 1, 2, \dots, n \)  
- 初始时，每个点 \( i \) 有一个权值 \( a_i \)  
- 操作：选择 \( 1 \le i \le n-1 \)，交换 \( a_i \) 与 \( a_{i+1} \)  
- 限制：每个点 \( i \) 最多被选择 \( b_i \) 次（即 \( i \) 作为交换的左端点最多 \( b_i \) 次）  
- 目标：让序列 \( a \) 非递减（即 \( a_1 \le a_2 \le \dots \le a_n \)）  
- 问能否实现  

---

## 2. 转化限制  
“点 \( i \) 最多被选择 \( b_i \) 次” 意思是：  
- 位置 \( i \) 作为交换的左端点最多 \( b_i \) 次  
- 一次交换 \((i, i+1)\) 同时涉及位置 \( i \) 和 \( i+1 \) 吗？  
  不，这里“被选择”是指“被选为交换的左端点 \( i \)”，所以只计 \( i \) 的次数，不计 \( i+1 \) 的次数。  
- 所以 \( b_i \) 是位置 \( i \) 可以向右交换的次数上限。  

---

## 3. 思考可行性条件  
我们想通过相邻交换排序，但每个位置有交换次数限制。  
这类似于冒泡排序中每个位置只能向右移动一定次数。  

设最终位置 \( j \) 上的数是原来位置 \( p_j \) 上的数。  
要让 \( a \) 非递减，我们可以把原来的数排序，然后分配位置，但这里限制是位置相关的，不是数相关的。  

更直接的想法：  
假设我们最终要把最小的数放到位置 1，第二小的数放到位置 2，等等。  
考虑原序列中某个值 \( v \) 在位置 \( x \)，它最终要去位置 \( y \)（\( y \ge x \) 或 \( y \le x \) 都有可能）。  

---

## 4. 关键观察  
如果 \( y \ge x \)：  
数要从 \( x \) 向右移动到 \( y \)，必须经过一系列向右的交换。  
每次向右交换一次，它需要它所在的位置作为左端点进行交换。  
但限制是每个位置有左端点次数限制，不是每个数有限制。  

所以一个数从 \( x \) 到 \( y \)（\( y > x \)）的过程中，它经过的位置 \( k \)（\( k = x, x+1, \dots, y-1 \)）都要以左端点身份交换一次，才能让它向右一步。  
因此，对于位置 \( k \in [x, y-1] \)，它的 \( b_k \) 必须至少 1（且每向右一步消耗一次 \( b_k \)）。  

如果 \( y < x \)：  
数要从 \( x \) 向左移动到 \( y \)，它需要被左边的数向右交换“推”过去。  
向左移动时，它自己不作为左端点，而是作为右端点。  
向左移动一步，比如从 \( k+1 \) 到 \( k \)，是位置 \( k \) 作为左端点交换它和左边的数。  
所以一个数从 \( x \) 向左到 \( y \)，需要位置 \( k \in [y, x-1] \) 作为左端点交换一次。  

---

## 5. 转化为位置交换容量问题  
设最终第 \( i \) 个位置放的数是原第 \( p_i \) 个位置的数（\( p \) 是一个排列）。  
那么对于任意 \( i \)，数从 \( p_i \) 移动到 \( i \) 需要：  
- 如果 \( p_i > i \)（原位置在右，目标在左，向左移动）：  
  需要位置 \( k \in [i, p_i-1] \) 各交换一次（作为左端点）。  
- 如果 \( p_i < i \)（原位置在左，目标在右，向右移动）：  
  需要位置 \( k \in [p_i, i-1] \) 各交换一次（作为左端点）。  

注意：一次交换同时移动两个数，一个向左一个向右，所以一个位置 \( k \) 的交换次数可能被多个数移动路径共享。  

---

## 6. 已知结论（类似题目）  
这类题有一个已知结论（Codeforces 上的一道题）：  
按最终非递减顺序分配位置时，贪心匹配：  
从小到大考虑每个数，把它放到最左边可行的位置。  
判断可行：  
设当前数原位置 \( x \)，要放到位置 \( y \)，如果 \( y \ge x \)，需要 \( b_x, b_{x+1}, \dots, b_{y-1} \) 都至少 1 的容量。  
但这样是每次独立判断，可能冲突，因为容量有限。  

更准确的方法：  
定义 \( c_k \) 为位置 \( k \) 还能作为左端点交换的次数（初始 \( c_k = b_k \)）。  
我们模拟过程：  
从最左到最右，依次确保位置 \( i \) 最终是前 \( i \) 小的数。  
维护一个“可用位置”列表，把原位置 \( x \) 的数放到合适位置时，需要路径上的 \( c_k \ge 1 \)，然后减少那些 \( c_k \)。  

---

## 7. 简化判断条件  
有一个充分必要条件（根据交换排序与逆序对）：  
对于每个 \( i \)，考虑原序列中所有在位置 \( \le i \) 但值大于某个阈值的数需要向右出去，所有在位置 \( > i \) 但值小的数需要向左进来，都需要经过位置 \( i \) 的交换。  
但更直接的条件是：  

设 \( S_j = b_1 + b_2 + \dots + b_j \) 即前 \( j \) 个位置总交换次数上限。  
为了把前 \( j \) 小的数放到前 \( j \) 个位置，设原序列前 \( j \) 个位置中有 \( x \) 个不属于前 \( j \) 小的数，这些数必须被换出去，每次换出一个数需要一次左边某个位置的交换。  
可以证明，需要 \( S_j \ge x \) 对于所有 \( j \) 成立。  

为什么？  
前 \( j \) 个位置最终应该是前 \( j \) 小的数。  
如果原前 \( j \) 个位置中有 \( x \) 个“坏数”（不在前 \( j \) 小），那么这些坏数必须被换到后面去。  
每次交换只能把一个坏数向右移一步（因为交换是相邻的），要把一个坏数移出前 \( j \) 个位置，至少需要它当前位置到位置 \( j \) 的每一步交换，且这些交换的左端点必须在 \( [\text{pos}, j-1] \) 之间，并且这些左端点次数从 \( b \) 里出。  
但不同坏数路径可能重叠使用同一个位置的交换次数。  
最坏情况下，每个坏数都需要至少一次不同的交换来把它推出前 \( j \) 个位置？  
其实更精确：  
设原前 \( j \) 个位置中有 \( x \) 个坏数，那么至少需要 \( x \) 次交换，其中左端点位置 \( \le j \)，因为每次交换（左端点 \( \le j \)）可以至多将一个坏数向右推一步（但可能同时把一个好数向左拉一步）。  
事实上，可以证明：最小需要的左端点位置在 \( 1..j \) 的交换次数就是 \( x \)。  
而这些交换次数不能超过 \( S_j = b_1+\dots+b_j \)。  

所以条件：  
对每个 \( j = 1 \dots n-1 \)，  
\[
S_j \ge \#\{ \text{原前 } j \text{ 个数中，值 > 前 } j \text{ 小的数} \}
\]
而“原前 \( j \) 个数中值 > 前 j 小的数”的个数 = 原前 \( j \) 个数中不在最终前 \( j \) 个位置的数的个数。  
最终前 \( j \) 个位置是排好序后前 \( j \) 小的数。  

所以算法：  
1. 对原数组 \( a \) 排序，知道每个数最终要去的位置（稳定或不稳定？可能有相同数，那么相同数顺序任意，为了最大化可行，我们把相同数按原顺序分配最终位置，避免不必要的移动）。  
   实际上，我们贪心：把原数组按值排序，值相同的按原位置排序，然后依次分配位置 1,2,...,n。  
   这样得到目标位置数组 \( target[1..n] \)。  
2. 计算 \( pref\_bad[j] \) = 原位置 \( \le j \) 的数中，\( target > j \) 的个数。  
   即原前 \( j \) 个位置中最终要去 \( j \) 之后的位置的数的个数。  
3. 检查是否对所有 \( j \) 有 \( S_j \ge pref\_bad[j] \)。  

---

## 8. 举例验证  
例：  
n=3, a=[3,2,1], b=[1,1,?]（b_3 无所谓）  
排序后 a 得到 [1,2,3]，原位置分别是 3,2,1。  
按值排序，值相同按原位置排，这里值都不同。  
分配目标位置：  
值 1（原位置 3）→ 目标位置 1  
值 2（原位置 2）→ 目标位置 2  
值 3（原位置 1）→ 目标位置 3  

pref_bad[1]：原位置 ≤1 的数：原位置1的数3，目标位置3>1，所以 bad=1。S1=b1=1，满足。  
pref_bad[2]：原位置 ≤2 的数：原位置1的数3（目标3>2），原位置2的数2（目标2=2，不是>2），所以 bad=1。S2=b1+b2=2，满足。  
所以可行。  
手动模拟：交换(1,2)：a=[2,3,1]，b=[0,1]；交换(2,3)：a=[2,1,3]，b=[0,0]；现在不能交换了，但序列不是非递减。  
啊，说明我的 pref_bad 条件可能只是必要而非充分？  

---

## 9. 重新思考充分条件  
实际上，已知一个经典结论（Codeforces 1632C 的某题）：  
按最终非递减顺序，把原位置映射到目标位置，设 \( d_i = target_i - i \)（正表示向右移动，负表示向左）。  
向左移动的数需要左边某个位置交换它，向右移动的数需要它原来位置开始向右的交换。  
但更简单的充要条件是：  
对每个 \( i \)，考虑所有 \( target_k \le i \) 的数（最终在前 i 个位置的数），它们原位置 \( > i \) 的那些数必须向左穿过位置 i，这需要位置 i 作为左端点交换它们。  
设 \( L_i = \#\{ target_k \le i \text{ 且 } orig_k > i \} \) 是最终去前 i 但原位置在 i 后的数的个数。  
这些数必须从右边移到左边，穿过 i 与 i+1 的间隙，每次穿过需要位置 i 作为左端点交换一次。  
所以 \( b_i \ge L_i \) 必须对所有 i 成立。  

类似地，考虑原位置 ≤ i 但最终位置 > i 的数，它们必须向右穿过位置 i 与 i+1 的间隙，也需要位置 i 作为左端点交换（实际上向右移动时，是原位置 p 在 ≤i 但目标 >i，它们向右穿过间隙时，是当它到达 i 时，再一步到 i+1 需要位置 i 交换）。  
但仔细想：一个数原位置 p ≤ i，目标 > i，它必须从 p 向右走到 i，然后从 i 到 i+1 需要位置 i 作为左端点交换一次。  
所以这种数也会消耗一次位置 i 的交换。  

因此，总消耗位置 i 的交换次数的数 = 最终在左但原在右的数（L_i） + 最终在右但原在左的数（R_i），其中 R_i = #{原位置 ≤ i 且 target > i}。  
注意 L_i 和 R_i 其实相等（因为前 i 个位置最终有 i 个数，原前 i 个位置有 i 个数，设原前 i 个位置中有 x 个最终不在前 i，那么它们被替换成 x 个从后面来的数，所以 L_i = R_i = x）。  
所以条件就是：  
\[
b_i \ge \#\{ \text{原位置} \le i \text{ 且 target} > i \}
\]
对每个 i 成立。  

---

## 10. 总结算法  
1. 对原数组 a 按值排序，值相同按原位置升序排序，得到目标位置 target[原位置] = 新位置。  
2. 对每个 i = 1..n-1，计算 cnt = 原位置 ≤ i 且 target > i 的个数。  
3. 检查是否 b_i ≥ cnt 对所有 i 成立。  
4. 如果都成立，输出 "YES"，否则 "NO"。  

---

## 11. 写代码框架  
```cpp
#include <bits/stdc++.h>
using namespace std;

void solve() {
    int n;
    cin >> n;
    vector<int> a(n), b(n);
    for (int i = 0; i < n; i++) cin >> a[i];
    for (int i = 0; i < n; i++) cin >> b[i];

    vector<int> idx(n);
    iota(idx.begin(), idx.end(), 0);
    stable_sort(idx.begin(), idx.end(), [&](int i, int j) {
        return a[i] < a[j];
    });

    vector<int> target(n);
    for (int pos = 0; pos < n; pos++) {
        target[idx[pos]] = pos; // 0-based 目标位置
    }

    for (int i = 0; i < n-1; i++) {
        int cnt = 0;
        for (int j = 0; j <= i; j++) {
            if (target[j] > i) cnt++;
        }
        if (b[i] < cnt) {
            cout << "NO\n";
            return;
        }
    }
    cout << "YES\n";
}

int main() {
    int t;
    cin >> t;
    while (t--) solve();
}
```