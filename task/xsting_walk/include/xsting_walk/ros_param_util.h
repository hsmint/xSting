#ifndef XSTING_WALK_ROS_PARAM_UTIL_H_
#define XSTING_WALK_ROS_PARAM_UTIL_H_

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace xsting_walk {

inline std::string TrimParamStr(const std::string& raw) {
  const auto first = raw.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const auto last = raw.find_last_not_of(" \t\r\n");
  return raw.substr(first, last - first + 1);
}

// CLI에서 정수(예: duration_s:=1)나 숫자 문자열("0.20")이 들어와도 안전하게 double로 변환
inline double DeclareDouble(rclcpp::Node* node,
                            const std::string& name,
                            double default_val) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  const auto pv =
      node->declare_parameter(name, rclcpp::ParameterValue(default_val), desc);

  switch (pv.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      const double v = pv.get<double>();
      if (!std::isfinite(v)) {
        throw std::invalid_argument("파라미터 '" + name +
                                    "' 값이 유한한 실수가 아닙니다.");
      }
      return v;
    }
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(pv.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_STRING: {
      const std::string s = TrimParamStr(pv.get<std::string>());
      if (!s.empty()) {
        try {
          size_t idx = 0;
          const double v = std::stod(s, &idx);
          if (idx == s.size() && std::isfinite(v)) return v;
        } catch (const std::exception&) {
        }
      }
      throw std::invalid_argument("파라미터 '" + name + "'에 숫자가 아닌 값('" +
                                  pv.get<std::string>() +
                                  "')이 입력되었습니다.");
    }
    default:
      throw std::invalid_argument("파라미터 '" + name +
                                  "' 타입이 올바르지 않습니다 (실수 필요).");
  }
}

// CLI에서 실수형 정수(예: 1.0)나 숫자 문자열("3")이 들어와도 안전하게 int로 변환
inline int DeclareInt(rclcpp::Node* node,
                      const std::string& name,
                      int default_val) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  const auto pv = node->declare_parameter(
      name, rclcpp::ParameterValue(static_cast<int64_t>(default_val)), desc);

  switch (pv.get_type()) {
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<int>(pv.get<int64_t>());
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      const double d = pv.get<double>();
      if (std::isfinite(d) && std::floor(d) == d) {
        return static_cast<int>(d);
      }
      throw std::invalid_argument("파라미터 '" + name +
                                  "' 값이 정수가 아닙니다.");
    }
    case rclcpp::ParameterType::PARAMETER_STRING: {
      const std::string s = TrimParamStr(pv.get<std::string>());
      if (!s.empty()) {
        try {
          size_t idx = 0;
          const int v = std::stoi(s, &idx);
          if (idx == s.size()) return v;
        } catch (const std::exception&) {
        }
      }
      throw std::invalid_argument("파라미터 '" + name + "'에 정수가 아닌 값('" +
                                  pv.get<std::string>() +
                                  "')이 입력되었습니다.");
    }
    default:
      throw std::invalid_argument("파라미터 '" + name +
                                  "' 타입이 올바르지 않습니다 (정수 필요).");
  }
}

inline bool DeclareBool(rclcpp::Node* node,
                        const std::string& name,
                        bool default_val) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  const auto pv =
      node->declare_parameter(name, rclcpp::ParameterValue(default_val), desc);

  switch (pv.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return pv.get<bool>();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return pv.get<int64_t>() != 0;
    case rclcpp::ParameterType::PARAMETER_STRING: {
      std::string s = TrimParamStr(pv.get<std::string>());
      std::transform(s.begin(), s.end(), s.begin(),
                     [](unsigned char c) { return std::tolower(c); });
      if (s == "true" || s == "1" || s == "yes" || s == "y" || s == "on") {
        return true;
      }
      if (s == "false" || s == "0" || s == "no" || s == "n" || s == "off") {
        return false;
      }
      throw std::invalid_argument(
          "파라미터 '" + name + "'에 불리언이 아닌 값('" +
          pv.get<std::string>() + "')이 입력되었습니다.");
    }
    default:
      throw std::invalid_argument("파라미터 '" + name +
                                  "' 타입이 올바르지 않습니다 (bool 필요).");
  }
}

inline std::string DeclareString(rclcpp::Node* node,
                                 const std::string& name,
                                 const std::string& default_val) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.dynamic_typing = true;
  const auto pv =
      node->declare_parameter(name, rclcpp::ParameterValue(default_val), desc);

  if (pv.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    return pv.get<std::string>();
  }
  return rclcpp::to_string(pv);
}

}  // namespace xsting_walk

#endif  // XSTING_WALK_ROS_PARAM_UTIL_H_
