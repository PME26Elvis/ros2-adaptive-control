// Copyright 2025
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>  // ← 需要這個
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace adaptive_controllers {
namespace param_utils {

// 從參數讀 1D 向量（name+"_data"），長度需為 rows*cols，然後 reshape 成矩陣（row-major）
inline bool read_matrix(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                        const std::string &name,
                        std::size_t rows, std::size_t cols,
                        Eigen::MatrixXd &out)
{
  const std::string key = name + "_data";
  if (!node->has_parameter(key)) {
    return false;
  }

  auto param = node->get_parameter(key);
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(node->get_logger(), "[%s] expected double[]", key.c_str());
    return false;
  }

  const auto data = param.as_double_array();
  if (data.size() != rows * cols) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] size %zu != %zu*%zu",
      key.c_str(), data.size(), rows, cols);
    return false;
  }

  out.resize(static_cast<Eigen::Index>(rows), static_cast<Eigen::Index>(cols));
  // row-major 填入
  for (std::size_t r = 0; r < rows; ++r) {
    for (std::size_t c = 0; c < cols; ++c) {
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
        data[r * cols + c];
    }
  }
  return true;
}

// 讀取 1D 向量參數 name+"_data"，長度需為 len
inline bool read_vector(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                        const std::string &name,
                        std::size_t len,
                        Eigen::VectorXd &out)
{
  const std::string key = name + "_data";
  if (!node->has_parameter(key)) {
    return false;
  }

  auto param = node->get_parameter(key);
  if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(node->get_logger(), "[%s] expected double[]", key.c_str());
    return false;
  }

  const auto data = param.as_double_array();
  if (data.size() != len) {
    RCLCPP_ERROR(
      node->get_logger(), "[%s] size %zu != %zu",
      key.c_str(), data.size(), len);
    return false;
  }

  out.resize(static_cast<Eigen::Index>(len));
  for (std::size_t i = 0; i < len; ++i) {
    out(static_cast<Eigen::Index>(i)) = data[i];
  }
  return true;
}

}  // namespace param_utils
}  // namespace adaptive_controllers
