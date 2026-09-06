/*
 * Standalone ROS 2 node that demonstrates the migrated loadParams() logic
 * from robot_localization's RosFilter<T>::loadParams().
 *
 * This node declares, retrieves, and validates sensor configuration parameters
 * following the robot_localization conventions (odom0, odom1, ..., imu0, ..., etc.)
 * with 15-element boolean config vectors.
 */

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <limits>

#include "rclcpp/rclcpp.hpp"

// STATE_SIZE for robot_localization is 15
static constexpr int STATE_SIZE = 15;

class SensorFusionParamsNode : public rclcpp::Node
{
public:
  explicit SensorFusionParamsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sensor_fusion_params_node", options)
  {
    // Declare basic frame parameters
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->declare_parameter<std::string>("world_frame", "odom");
    this->declare_parameter<bool>("two_d_mode", false);
    this->declare_parameter<bool>("print_diagnostics", false);
    this->declare_parameter<double>("frequency", 30.0);
    this->declare_parameter<double>("sensor_timeout", 1.0 / 30.0);

    loadParams();
  }

private:
  struct CallbackData
  {
    std::string topic_name_;
    std::vector<bool> update_vector_;
    int update_sum_;
    bool differential_;
    bool relative_;
    double rejection_threshold_;
  };

  std::vector<CallbackData> topic_subs_;

  void loadParams()
  {
    // Retrieve basic parameters
    std::string map_frame;
    this->get_parameter("map_frame", map_frame);
    RCLCPP_INFO(this->get_logger(), "map_frame: %s", map_frame.c_str());

    std::string odom_frame;
    this->get_parameter("odom_frame", odom_frame);
    RCLCPP_INFO(this->get_logger(), "odom_frame: %s", odom_frame.c_str());

    std::string base_link_frame;
    this->get_parameter("base_link_frame", base_link_frame);

    std::string world_frame;
    this->get_parameter("world_frame", world_frame);

    bool two_d_mode = false;
    this->get_parameter("two_d_mode", two_d_mode);

    double frequency = 30.0;
    this->get_parameter("frequency", frequency);

    double sensor_timeout = 1.0 / 30.0;
    this->get_parameter("sensor_timeout", sensor_timeout);

    // Sensor type prefixes to iterate over
    std::vector<std::string> sensor_types = {"odom", "pose", "twist", "imu", "accel"};

    for (const auto & sensor_type : sensor_types) {
      for (int i = 0; i < 100; ++i) {
        std::string sensor_name = sensor_type + std::to_string(i);
        std::string topic_param = sensor_name;

        // Try to declare the topic parameter with a default empty string.
        try {
          this->declare_parameter<std::string>(topic_param, std::string(""));
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
          // Already declared, that's fine
        }

        std::string topic_name;
        this->get_parameter(topic_param, topic_name);

        if (topic_name.empty()) {
          // No more sensors of this type
          break;
        }

        RCLCPP_INFO(
          this->get_logger(), "Found sensor %s with topic: %s",
          sensor_name.c_str(), topic_name.c_str());

        // Declare and retrieve the config vector: <type><index>_config
        std::string config_param = sensor_type + std::to_string(i) + "_config";

        try {
          this->declare_parameter<std::vector<bool>>(config_param, std::vector<bool>(STATE_SIZE, false));
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
          // Already declared
        }

        std::vector<bool> update_vector;
        this->get_parameter(config_param, update_vector);

        // Validate the 15-element requirement
        if (update_vector.size() != static_cast<size_t>(STATE_SIZE)) {
          RCLCPP_WARN(
            this->get_logger(),
            "Sensor %s has a config vector of size %zu, expected %d. "
            "The update vector must have exactly 15 elements. Ignoring this sensor.",
            sensor_name.c_str(), update_vector.size(), STATE_SIZE);
          continue;
        }

        // Compute update sum
        int update_sum = 0;
        for (size_t j = 0; j < update_vector.size(); ++j) {
          update_sum += (update_vector[j] ? 1 : 0);
        }

        // Optionally retrieve differential and relative flags
        std::string diff_param = sensor_name + "_differential";
        try {
          this->declare_parameter<bool>(diff_param, false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        }
        bool differential = false;
        this->get_parameter(diff_param, differential);

        std::string rel_param = sensor_name + "_relative";
        try {
          this->declare_parameter<bool>(rel_param, false);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        }
        bool relative = false;
        this->get_parameter(rel_param, relative);

        // Rejection threshold
        std::string reject_param = sensor_name + "_rejection_threshold";
        try {
          this->declare_parameter<double>(reject_param, std::numeric_limits<double>::max());
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        }
        double rejection_threshold = std::numeric_limits<double>::max();
        this->get_parameter(reject_param, rejection_threshold);

        // Build CallbackData and store
        CallbackData cb_data;
        cb_data.topic_name_ = topic_name;
        cb_data.update_vector_ = update_vector;
        cb_data.update_sum_ = update_sum;
        cb_data.differential_ = differential;
        cb_data.relative_ = relative;
        cb_data.rejection_threshold_ = rejection_threshold;

        topic_subs_.push_back(cb_data);

        RCLCPP_INFO(
          this->get_logger(),
          "Configured sensor %s: topic=%s, update_sum=%d, differential=%s, relative=%s",
          sensor_name.c_str(), topic_name.c_str(), update_sum,
          differential ? "true" : "false",
          relative ? "true" : "false");

        // Log the update vector
        std::stringstream ss;
        ss << "[";
        for (size_t j = 0; j < update_vector.size(); ++j) {
          ss << (update_vector[j] ? "true" : "false");
          if (j < update_vector.size() - 1) {
            ss << ", ";
          }
        }
        ss << "]";
        RCLCPP_INFO(
          this->get_logger(), "  %s update vector: %s",
          config_param.c_str(), ss.str().c_str());
      }
    }

    RCLCPP_INFO(
      this->get_logger(), "Loaded %zu sensor configurations total.",
      topic_subs_.size());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorFusionParamsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}