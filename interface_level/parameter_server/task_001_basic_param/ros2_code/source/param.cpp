/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * ROS2 migration for subscription-backed parameter cache semantics.
 */

#include "task_001_basic_param/param.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros
{

namespace param
{

using namespace std::chrono_literals;

typedef std::map<std::string, XmlRpc::XmlRpcValue> M_Param;
M_Param g_params_cache;
std::recursive_mutex g_params_mutex;
std::set<std::string> g_subscribed_params;

rclcpp::Node::SharedPtr g_node;
std::string g_remote_node_name = "/param_provider";
rclcpp::CallbackGroup::SharedPtr g_callback_group;
rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr g_parameter_event_subscription;
std::shared_ptr<rclcpp::AsyncParametersClient> g_parameters_client;

static std::string cleanKey(const std::string & key)
{
  if (key.empty()) {
    return "/";
  }

  std::string result = key;
  std::replace(result.begin(), result.end(), '.', '/');

  if (result.front() != '/') {
    result = "/" + result;
  }

  while (result.find("//") != std::string::npos) {
    result.erase(result.find("//"), 1);
  }

  if (result.size() > 1U && result.back() == '/') {
    result.pop_back();
  }

  return result;
}

static std::string parentNamespace(const std::string & key)
{
  const std::string cleaned = cleanKey(key);
  if (cleaned.empty() || cleaned == "/") {
    return "";
  }

  const std::size_t pos = cleaned.find_last_of('/');
  if (pos == std::string::npos) {
    return "";
  }
  if (pos == 0U) {
    return "/";
  }
  return cleaned.substr(0U, pos);
}

static std::string rosKeyToParameterName(const std::string & key)
{
  std::string mapped = cleanKey(key);
  if (!mapped.empty() && mapped.front() == '/') {
    mapped.erase(mapped.begin());
  }
  std::replace(mapped.begin(), mapped.end(), '/', '.');
  return mapped;
}

static std::string parameterNameToRosKey(const std::string & name)
{
  std::string key = name;
  std::replace(key.begin(), key.end(), '.', '/');
  return cleanKey(key);
}

static XmlRpc::XmlRpcValue parameterMsgToXmlRpc(const rcl_interfaces::msg::Parameter & parameter)
{
  using rcl_interfaces::msg::ParameterType;
  const auto & value = parameter.value;

  switch (value.type) {
    case ParameterType::PARAMETER_BOOL:
      return XmlRpc::XmlRpcValue(value.bool_value);
    case ParameterType::PARAMETER_INTEGER:
      return XmlRpc::XmlRpcValue(static_cast<int>(value.integer_value));
    case ParameterType::PARAMETER_DOUBLE:
      return XmlRpc::XmlRpcValue(value.double_value);
    case ParameterType::PARAMETER_STRING:
      return XmlRpc::XmlRpcValue(value.string_value);
    case ParameterType::PARAMETER_BOOL_ARRAY: {
      XmlRpc::XmlRpcValue array;
      array.setSize(value.bool_array_value.size());
      for (std::size_t i = 0; i < value.bool_array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(value.bool_array_value[i]);
      }
      return array;
    }
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
      XmlRpc::XmlRpcValue array;
      array.setSize(value.integer_array_value.size());
      for (std::size_t i = 0; i < value.integer_array_value.size(); ++i) {
        array[static_cast<int>(i)] =
          XmlRpc::XmlRpcValue(static_cast<int>(value.integer_value));
      }
      return array;
    }
    case ParameterType::PARAMETER_DOUBLE_ARRAY: {
      XmlRpc::XmlRpcValue array;
      array.setSize(value.double_array_value.size());
      for (std::size_t i = 0; i < value.double_array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(value.double_array_value[i]);
      }
      return array;
    }
    case ParameterType::PARAMETER_STRING_ARRAY: {
      XmlRpc::XmlRpcValue array;
      array.setSize(value.string_array_value.size());
      for (std::size_t i = 0; i < value.string_array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(value.string_array_value[i]);
      }
      return array;
    }
    case ParameterType::PARAMETER_NOT_SET:
    default:
      return XmlRpc::XmlRpcValue();
  }
}

static XmlRpc::XmlRpcValue parameterValueToXmlRpc(const rclcpp::ParameterValue & value)
{
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return XmlRpc::XmlRpcValue(value.get<bool>());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return XmlRpc::XmlRpcValue(static_cast<int>(value.get<int64_t>()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return XmlRpc::XmlRpcValue(value.get<double>());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return XmlRpc::XmlRpcValue(value.get<std::string>());
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY: {
      const auto array_value = value.get<std::vector<bool>>();
      XmlRpc::XmlRpcValue array;
      array.setSize(array_value.size());
      for (std::size_t i = 0; i < array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(array_value[i]);
      }
      return array;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY: {
      const auto array_value = value.get<std::vector<int64_t>>();
      XmlRpc::XmlRpcValue array;
      array.setSize(array_value.size());
      for (std::size_t i = 0; i < array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(static_cast<int>(array_value[i]));
      }
      return array;
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      const auto array_value = value.get<std::vector<double>>();
      XmlRpc::XmlRpcValue array;
      array.setSize(array_value.size());
      for (std::size_t i = 0; i < array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(array_value[i]);
      }
      return array;
    }
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY: {
      const auto array_value = value.get<std::vector<std::string>>();
      XmlRpc::XmlRpcValue array;
      array.setSize(array_value.size());
      for (std::size_t i = 0; i < array_value.size(); ++i) {
        array[static_cast<int>(i)] = XmlRpc::XmlRpcValue(array_value[i]);
      }
      return array;
    }
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
    default:
      return XmlRpc::XmlRpcValue();
  }
}

static rclcpp::Parameter xmlRpcToParameter(
  const std::string & parameter_name,
  const XmlRpc::XmlRpcValue & value)
{
  switch (value.getType()) {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return rclcpp::Parameter(parameter_name, static_cast<bool>(value));
    case XmlRpc::XmlRpcValue::TypeInt:
      return rclcpp::Parameter(parameter_name, static_cast<int>(value));
    case XmlRpc::XmlRpcValue::TypeDouble:
      return rclcpp::Parameter(parameter_name, static_cast<double>(value));
    case XmlRpc::XmlRpcValue::TypeString:
      return rclcpp::Parameter(parameter_name, static_cast<std::string>(value));
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
    default:
      return rclcpp::Parameter(parameter_name);
  }
}

void invalidateParentParams(const std::string & key)
{
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

  std::string namespace_key = parentNamespace(key);
  while (namespace_key != "" && namespace_key != "/") {
    if (g_subscribed_params.find(namespace_key) != g_subscribed_params.end()) {
      g_params_cache.erase(namespace_key);
    }
    namespace_key = parentNamespace(namespace_key);
  }
}

void update(const std::string & key, const XmlRpc::XmlRpcValue & v)
{
  const std::string mapped_key = cleanKey(key);

  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

  if (g_subscribed_params.find(mapped_key) == g_subscribed_params.end()) {
    return;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeInvalid) {
    g_params_cache.erase(mapped_key);
  } else {
    g_params_cache[mapped_key] = v;
  }

  invalidateParentParams(mapped_key);
}

void init(
  const rclcpp::Node::SharedPtr & node,
  const std::string & remote_node_name,
  const rclcpp::CallbackGroup::SharedPtr & callback_group)
{
  std::lock_guard<std::recursive_mutex> lock(g_params_mutex);

  g_node = node;
  g_remote_node_name = remote_node_name.empty() ? "/param_provider" : remote_node_name;
  if (!g_remote_node_name.empty() && g_remote_node_name.front() != '/') {
    g_remote_node_name = "/" + g_remote_node_name;
  }
  g_callback_group = callback_group;

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = g_callback_group;

  g_parameter_event_subscription =
    g_node->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events",
    rclcpp::ParameterEventsQoS(),
    [](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
      if (!g_remote_node_name.empty() && event->node != g_remote_node_name) {
        return;
      }

      for (const auto & parameter : event->new_parameters) {
        update(parameterNameToRosKey(parameter.name), parameterMsgToXmlRpc(parameter));
      }
      for (const auto & parameter : event->changed_parameters) {
        update(parameterNameToRosKey(parameter.name), parameterMsgToXmlRpc(parameter));
      }
      for (const auto & parameter : event->deleted_parameters) {
        update(parameterNameToRosKey(parameter.name), XmlRpc::XmlRpcValue());
      }
    },
    subscription_options);

  g_parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
    g_node, g_remote_node_name, rmw_qos_profile_services_default, g_callback_group);
}

void set(const std::string & key, const XmlRpc::XmlRpcValue & v)
{
  const std::string mapped_key = cleanKey(key);
  const std::string parameter_name = rosKeyToParameterName(mapped_key);

  if (g_parameters_client && g_parameters_client->wait_for_service(2s)) {
    auto future = g_parameters_client->set_parameters({xmlRpcToParameter(parameter_name, v)});
    if (future.wait_for(2s) == std::future_status::ready) {
      const auto results = future.get();
      if (!results.empty() && results.front().successful) {
        std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
        if (g_subscribed_params.find(mapped_key) != g_subscribed_params.end()) {
          g_params_cache[mapped_key] = v;
        }
        invalidateParentParams(mapped_key);
      }
    }
  }
}

void set(const std::string & key, const std::string & s)
{
  set(key, XmlRpc::XmlRpcValue(s));
}

void set(const std::string & key, const char * s)
{
  set(key, XmlRpc::XmlRpcValue(std::string(s)));
}

void set(const std::string & key, double d)
{
  set(key, XmlRpc::XmlRpcValue(d));
}

void set(const std::string & key, int i)
{
  set(key, XmlRpc::XmlRpcValue(i));
}

void set(const std::string & key, bool b)
{
  set(key, XmlRpc::XmlRpcValue(b));
}

bool has(const std::string & key)
{
  XmlRpc::XmlRpcValue value;
  return getImpl(key, value, false);
}

bool del(const std::string & key)
{
  const std::string mapped_key = cleanKey(key);
  const std::string parameter_name = rosKeyToParameterName(mapped_key);

  {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
    g_subscribed_params.erase(mapped_key);
    g_params_cache.erase(mapped_key);
    invalidateParentParams(mapped_key);
  }

  if (!g_parameters_client || !g_parameters_client->wait_for_service(2s)) {
    return false;
  }

  auto future = g_parameters_client->delete_parameters({parameter_name});
  if (future.wait_for(2s) != std::future_status::ready) {
    return false;
  }

  const auto results = future.get();
  return !results.empty() && results.front().successful;
}

bool getImpl(const std::string & key, XmlRpc::XmlRpcValue & v, bool use_cache)
{
  const std::string mapped_key = cleanKey(key);

  if (use_cache) {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
    auto cache_it = g_params_cache.find(mapped_key);
    if (cache_it != g_params_cache.end()) {
      v = cache_it->second;
      return true;
    }

    g_subscribed_params.insert(mapped_key);
  }

  if (!g_parameters_client) {
    return false;
  }

  if (!g_parameters_client->wait_for_service(2s)) {
    return false;
  }

  const std::string parameter_name = rosKeyToParameterName(mapped_key);
  auto future = g_parameters_client->get_parameters({parameter_name});
  if (future.wait_for(2s) != std::future_status::ready) {
    return false;
  }

  const std::vector<rclcpp::Parameter> parameters = future.get();
  if (parameters.empty() ||
    parameters.front().get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
  {
    return false;
  }

  v = parameterValueToXmlRpc(parameters.front().get_parameter_value());

  if (use_cache) {
    std::lock_guard<std::recursive_mutex> lock(g_params_mutex);
    g_params_cache[mapped_key] = v;
  }

  return true;
}

bool getImpl(const std::string & key, std::string & s, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache)) {
    return false;
  }
  if (v.getType() != XmlRpc::XmlRpcValue::TypeString) {
    return false;
  }
  s = static_cast<std::string>(v);
  return true;
}

bool getImpl(const std::string & key, double & d, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache)) {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt ||
    v.getType() == XmlRpc::XmlRpcValue::TypeDouble ||
    v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
  {
    d = static_cast<double>(v);
    return true;
  }

  return false;
}

bool getImpl(const std::string & key, float & f, bool use_cache)
{
  double d = 0.0;
  if (!getImpl(key, d, use_cache)) {
    return false;
  }
  f = static_cast<float>(d);
  return true;
}

bool getImpl(const std::string & key, int & i, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache)) {
    return false;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    double d = static_cast<double>(v);
    if (std::fmod(d, 1.0) < 0.5) {
      d = std::floor(d);
    } else {
      d = std::ceil(d);
    }
    i = static_cast<int>(d);
    return true;
  }

  if (v.getType() == XmlRpc::XmlRpcValue::TypeInt ||
    v.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
  {
    i = static_cast<int>(v);
    return true;
  }

  return false;
}

bool getImpl(const std::string & key, bool & b, bool use_cache)
{
  XmlRpc::XmlRpcValue v;
  if (!getImpl(key, v, use_cache)) {
    return false;
  }
  if (v.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
    return false;
  }
  b = static_cast<bool>(v);
  return true;
}

bool get(const std::string & key, XmlRpc::XmlRpcValue & v)
{
  return getImpl(key, v, false);
}

bool get(const std::string & key, std::string & s)
{
  return getImpl(key, s, false);
}

bool get(const std::string & key, double & d)
{
  return getImpl(key, d, false);
}

bool get(const std::string & key, float & f)
{
  return getImpl(key, f, false);
}

bool get(const std::string & key, int & i)
{
  return getImpl(key, i, false);
}

bool get(const std::string & key, bool & b)
{
  return getImpl(key, b, false);
}

bool getCached(const std::string & key, XmlRpc::XmlRpcValue & v)
{
  return getImpl(key, v, true);
}

bool getCached(const std::string & key, std::string & s)
{
  return getImpl(key, s, true);
}

bool getCached(const std::string & key, double & d)
{
  return getImpl(key, d, true);
}

bool getCached(const std::string & key, float & f)
{
  return getImpl(key, f, true);
}

bool getCached(const std::string & key, int & i)
{
  return getImpl(key, i, true);
}

bool getCached(const std::string & key, bool & b)
{
  return getImpl(key, b, true);
}

}  // namespace param

}  // namespace ros