/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <memory>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "utils/dataset_reader.h"

#include "ros/ROS2Visualizer.h"
#include <rclcpp/rclcpp.hpp>

namespace ov_msckf
{

/* class RunSubscriberMsckf //{ */

class RunSubscriberMsckf : public rclcpp::Node {
public:
  RunSubscriberMsckf(const rclcpp::NodeOptions & options);

private:
  void initialize();

  rclcpp::TimerBase::SharedPtr timer_preinitialization_;
  void timerPreInitialization();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<VioManager> sys_;
  std::shared_ptr<ROS2Visualizer> viz_;
};

//}

/* RunSubscriberMsckf //{ */

RunSubscriberMsckf::RunSubscriberMsckf(const rclcpp::NodeOptions & options) : rclcpp::Node("run_subscriber_msckf", options) {
  timer_preinitialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&RunSubscriberMsckf::timerPreInitialization, this));

}

//}

/* timerPreInitialization() //{ */

void RunSubscriberMsckf::timerPreInitialization() {
  node_ = this->shared_from_this();
  initialize();
  timer_preinitialization_->cancel();
}

//}

/* initialize() //{ */

void RunSubscriberMsckf::initialize(){
  RCLCPP_INFO(node_->get_logger(), "Initializing");

  // Declare parameters
  node_->declare_parameter("uav_name", rclcpp::ParameterType::PARAMETER_STRING);
  node_->declare_parameter("config_path", rclcpp::ParameterType::PARAMETER_STRING);

  // Load the config
  std::string config_path = "unset_path_to_config.yaml";
  node_->get_parameter("config_path", config_path);
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
  parser->set_node(node_);

  // Verbosity level
  std::string verbosity = "DEBUG";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  params.use_multi_threading_subs = true;
  sys_ = std::make_shared<VioManager>(params);

  viz_ = std::make_shared<ROS2Visualizer>(node_, sys_);
  viz_->setup_subscribers(parser);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to parse all parameters, please fix");
    rclcpp::shutdown();
    /* exit(1); */
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "Initialized!");
}

//}

} // namespace ov_msckf

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ov_msckf::RunSubscriberMsckf);
