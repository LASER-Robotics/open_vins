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
 * along with this program.  See <https://www.gnu.org/licenses/>.
 */

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#else
#include <ros/ros.h>
#include <geometry_msgs/pose_stamped.h>
#include <geometry_msgs/pose_with_covariance_stamped.h>
#include <geometry_msgs/transform_stamped.h>
#include <nav_msgs/odometry.h>
#endif

#include "utils/Recorder.h"
#include "utils/print.h"

int main(int argc, char **argv) {

#ifdef ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_to_file");

  std::string verbosity;
  node->declare_parameter<std::string>("verbosity", "INFO");
  node->get_parameter("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  std::string topic, topic_type, fileoutput;
  node->declare_parameter<std::string>("topic", "");
  node->declare_parameter<std::string>("topic_type", "");
  node->declare_parameter<std::string>("output", "");
  node->get_parameter("topic", topic);
  node->get_parameter("topic_type", topic_type);
  node->get_parameter("output", fileoutput);
#else
  ros::init(argc, argv, "pose_to_file");
  ros::NodeHandle nh("~");

  std::string verbosity;
  nh.param<std::string>("verbosity", verbosity, "INFO");
  ov_core::Printer::setPrintLevel(verbosity);

  std::string topic, topic_type, fileoutput;
  nh.getParam("topic", topic);
  nh.getParam("topic_type", topic_type);
  nh.getParam("output", fileoutput);
#endif

  PRINT_DEBUG("Done reading config values");
  PRINT_DEBUG(" - topic = %s", topic.c_str());
  PRINT_DEBUG(" - topic_type = %s", topic_type.c_str());
  PRINT_DEBUG(" - file = %s", fileoutput.c_str());

  ov_eval::Recorder recorder(fileoutput);

#ifdef ROS2
  rclcpp::SubscriptionBase::SharedPtr sub;
  if (topic_type == "PoseWithCovarianceStamped") {
    sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic, 10, std::bind(&ov_eval::Recorder::callback_posecovariance, &recorder, std::placeholders::_1));
  } else if (topic_type == "PoseStamped") {
    sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic, 10, std::bind(&ov_eval::Recorder::callback_pose, &recorder, std::placeholders::_1));
  } else if (topic_type == "TransformStamped") {
    sub = node->create_subscription<geometry_msgs::msg::TransformStamped>(
        topic, 10, std::bind(&ov_eval::Recorder::callback_transform, &recorder, std::placeholders::_1));
  } else if (topic_type == "Odometry") {
    sub = node->create_subscription<nav_msgs::msg::Odometry>(
        topic, 10, std::bind(&ov_eval::Recorder::callback_odometry, &recorder, std::placeholders::_1));
  } else {
    PRINT_ERROR("The specified topic type is not supported");
    PRINT_ERROR("topic_type = %s", topic_type.c_str());
    PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    std::exit(EXIT_FAILURE);
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
#else
  ros::Subscriber sub;
  if (topic_type == std::string("PoseWithCovarianceStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_posecovariance, &recorder);
  } else if (topic_type == std::string("PoseStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_pose, &recorder);
  } else if (topic_type == std::string("TransformStamped")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_transform, &recorder);
  } else if (topic_type == std::string("Odometry")) {
    sub = nh.subscribe(topic, 9999, &ov_eval::Recorder::callback_odometry, &recorder);
  } else {
    PRINT_ERROR("The specified topic type is not supported");
    PRINT_ERROR("topic_type = %s", topic_type.c_str());
    PRINT_ERROR("please select from: PoseWithCovarianceStamped, PoseStamped, TransformStamped, Odometry");
    std::exit(EXIT_FAILURE);
  }
  ros::spin();
#endif

  return EXIT_SUCCESS;
}
