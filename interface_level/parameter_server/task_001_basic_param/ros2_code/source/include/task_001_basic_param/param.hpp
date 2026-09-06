#ifndef TASK_001_BASIC_PARAM__PARAM_HPP_
#define TASK_001_BASIC_PARAM__PARAM_HPP_

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace XmlRpc
{

class XmlRpcValue
{
public:
  enum Type
  {
    TypeInvalid = 0,
    TypeBoolean = 1,
    TypeInt = 2,
    TypeDouble = 3,
    TypeString = 4,
    TypeArray = 5,
    TypeStruct = 6
  };

  using Array = std::vector<XmlRpcValue>;
  using ValueStruct = std::map<std::string, XmlRpcValue>;

  XmlRpcValue() : type_(TypeInvalid), value_(std::monostate{}) {}
  XmlRpcValue(bool value) : type_(TypeBoolean), value_(value) {}
  XmlRpcValue(int value) : type_(TypeInt), value_(value) {}
  XmlRpcValue(int64_t value) : type_(TypeInt), value_(static_cast<int>(value)) {}
  XmlRpcValue(double value) : type_(TypeDouble), value_(value) {}
  XmlRpcValue(const char * value) : type_(TypeString), value_(std::string(value)) {}
  XmlRpcValue(const std::string & value) : type_(TypeString), value_(value) {}

  Type getType() const
  {
    return type_;
  }

  int size() const
  {
    if (type_ == TypeArray) {
      return static_cast<int>(std::get<Array>(value_).size());
    }
    if (type_ == TypeStruct) {
      return static_cast<int>(std::get<ValueStruct>(value_).size());
    }
    return 0;
  }

  void setSize(std::size_t size)
  {
    if (type_ != TypeArray) {
      type_ = TypeArray;
      value_ = Array{};
    }
    std::get<Array>(value_).resize(size);
  }

  XmlRpcValue & operator[](int index)
  {
    if (type_ != TypeArray) {
      type_ = TypeArray;
      value_ = Array{};
    }
    auto & array = std::get<Array>(value_);
    if (index >= static_cast<int>(array.size())) {
      array.resize(static_cast<std::size_t>(index) + 1U);
    }
    return array[static_cast<std::size_t>(index)];
  }

  const XmlRpcValue & operator[](int index) const
  {
    return std::get<Array>(value_)[static_cast<std::size_t>(index)];
  }

  XmlRpcValue & operator[](const std::string & key)
  {
    if (type_ != TypeStruct) {
      type_ = TypeStruct;
      value_ = ValueStruct{};
    }
    return std::get<ValueStruct>(value_)[key];
  }

  const XmlRpcValue & operator[](const std::string & key) const
  {
    return std::get<ValueStruct>(value_).at(key);
  }

  ValueStruct::iterator begin()
  {
    if (type_ != TypeStruct) {
      type_ = TypeStruct;
      value_ = ValueStruct{};
    }
    return std::get<ValueStruct>(value_).begin();
  }

  ValueStruct::iterator end()
  {
    if (type_ != TypeStruct) {
      type_ = TypeStruct;
      value_ = ValueStruct{};
    }
    return std::get<ValueStruct>(value_).end();
  }

  ValueStruct::const_iterator begin() const
  {
    return std::get<ValueStruct>(value_).begin();
  }

  ValueStruct::const_iterator end() const
  {
    return std::get<ValueStruct>(value_).end();
  }

  operator bool() const
  {
    if (type_ == TypeBoolean) {
      return std::get<bool>(value_);
    }
    if (type_ == TypeInt) {
      return std::get<int>(value_) != 0;
    }
    if (type_ == TypeDouble) {
      return std::get<double>(value_) != 0.0;
    }
    return false;
  }

  operator int() const
  {
    if (type_ == TypeInt) {
      return std::get<int>(value_);
    }
    if (type_ == TypeBoolean) {
      return std::get<bool>(value_) ? 1 : 0;
    }
    if (type_ == TypeDouble) {
      return static_cast<int>(std::get<double>(value_));
    }
    return 0;
  }

  operator double() const
  {
    if (type_ == TypeDouble) {
      return std::get<double>(value_);
    }
    if (type_ == TypeInt) {
      return static_cast<double>(std::get<int>(value_));
    }
    if (type_ == TypeBoolean) {
      return std::get<bool>(value_) ? 1.0 : 0.0;
    }
    return 0.0;
  }

  operator std::string() const
  {
    if (type_ == TypeString) {
      return std::get<std::string>(value_);
    }
    return toString();
  }

  std::string toString() const
  {
    std::ostringstream out;
    switch (type_) {
      case TypeBoolean:
        out << (std::get<bool>(value_) ? "true" : "false");
        break;
      case TypeInt:
        out << std::get<int>(value_);
        break;
      case TypeDouble:
        out << std::get<double>(value_);
        break;
      case TypeString:
        out << std::get<std::string>(value_);
        break;
      case TypeArray: {
        out << "[";
        const auto & array = std::get<Array>(value_);
        for (std::size_t i = 0; i < array.size(); ++i) {
          if (i != 0U) {
            out << ", ";
          }
          out << array[i].toString();
        }
        out << "]";
        break;
      }
      case TypeStruct: {
        out << "{";
        const auto & map = std::get<ValueStruct>(value_);
        bool first = true;
        for (const auto & entry : map) {
          if (!first) {
            out << ", ";
          }
          first = false;
          out << entry.first << ": " << entry.second.toString();
        }
        out << "}";
        break;
      }
      case TypeInvalid:
      default:
        out << "<invalid>";
        break;
    }
    return out.str();
  }

private:
  Type type_;
  std::variant<std::monostate, bool, int, double, std::string, Array, ValueStruct> value_;
};

}  // namespace XmlRpc

namespace ros
{
namespace param
{

void init(
  const rclcpp::Node::SharedPtr & node,
  const std::string & remote_node_name = "/param_provider",
  const rclcpp::CallbackGroup::SharedPtr & callback_group = nullptr);

void invalidateParentParams(const std::string & key);
void update(const std::string & key, const XmlRpc::XmlRpcValue & v);

void set(const std::string & key, const XmlRpc::XmlRpcValue & v);
void set(const std::string & key, const std::string & s);
void set(const std::string & key, const char * s);
void set(const std::string & key, double d);
void set(const std::string & key, int i);
void set(const std::string & key, bool b);

bool has(const std::string & key);
bool del(const std::string & key);

bool getImpl(const std::string & key, XmlRpc::XmlRpcValue & v, bool use_cache);
bool getImpl(const std::string & key, std::string & s, bool use_cache);
bool getImpl(const std::string & key, double & d, bool use_cache);
bool getImpl(const std::string & key, float & f, bool use_cache);
bool getImpl(const std::string & key, int & i, bool use_cache);
bool getImpl(const std::string & key, bool & b, bool use_cache);

bool get(const std::string & key, XmlRpc::XmlRpcValue & v);
bool get(const std::string & key, std::string & s);
bool get(const std::string & key, double & d);
bool get(const std::string & key, float & f);
bool get(const std::string & key, int & i);
bool get(const std::string & key, bool & b);

bool getCached(const std::string & key, XmlRpc::XmlRpcValue & v);
bool getCached(const std::string & key, std::string & s);
bool getCached(const std::string & key, double & d);
bool getCached(const std::string & key, float & f);
bool getCached(const std::string & key, int & i);
bool getCached(const std::string & key, bool & b);

}  // namespace param
}  // namespace ros

#endif  // TASK_001_BASIC_PARAM__PARAM_HPP_