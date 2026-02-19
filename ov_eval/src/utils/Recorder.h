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

#ifndef OV_EVAL_RECORDER_H
#define OV_EVAL_RECORDER_H

#include <fstream>
#include <iostream>
#include <string>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#else
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#endif

namespace ov_eval {

class Recorder {

public:

  Recorder(std::string filename) {
    boost::filesystem::path dir(filename.c_str());
    if (boost::filesystem::create_directories(dir.parent_path())) {
#ifdef ROS2
      std::cout << "[INFO] Created folder path to output file." << std::endl;
      std::cout << "[INFO] Path: " << dir.parent_path().string() << std::endl;
#else
      ROS_INFO("Created folder path to output file.");
      ROS_INFO("Path: %s", dir.parent_path().c_str());
#endif
    }
    if (boost::filesystem::exists(filename)) {
#ifdef ROS2
      std::cout << "[WARN] Output file exists, deleting old file...." << std::endl;
#else
      ROS_WARN("Output file exists, deleting old file....");
#endif
      boost::filesystem::remove(filename);
    }
    outfile.open(filename.c_str());
    if (outfile.fail()) {
#ifdef ROS2
      std::cerr << "[ERROR] Unable to open output file!!" << std::endl;
      std::cerr << "[ERROR] Path: " << filename << std::endl;
#else
      ROS_ERROR("Unable to open output file!!");
      ROS_ERROR("Path: %s", filename.c_str());
#endif
      std::exit(EXIT_FAILURE);
    }
    outfile << "# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33" << std::endl;
    timestamp = -1;
    q_ItoG << 0, 0, 0, 1;
    p_IinG = Eigen::Vector3d::Zero();
    cov_rot = Eigen::Matrix<double, 3, 3>::Zero();
    cov_pos = Eigen::Matrix<double, 3, 3>::Zero();
    has_covariance = false;
  }

#ifdef ROS2
  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
    timestamp = rclcpp::Time(msg->header.stamp).seconds();
    q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
    p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cov_pos << msg->pose.covariance.at(0), msg->pose.covariance.at(1), msg->pose.covariance.at(2), msg->pose.covariance.at(6),
        msg->pose.covariance.at(7), msg->pose.covariance.at(8), msg->pose.covariance.at(12), msg->pose.covariance.at(13),
        msg->pose.covariance.at(14);
    cov_rot << msg->pose.covariance.at(21), msg->pose.covariance.at(22), msg->pose.covariance.at(23), msg->pose.covariance.at(27),
        msg->pose.covariance.at(28), msg->pose.covariance.at(29), msg->pose.covariance.at(33), msg->pose.covariance.at(34),
        msg->pose.covariance.at(35);
    has_covariance = true;
    write();
  }

  void callback_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    timestamp = rclcpp::Time(msg->header.stamp).seconds();
    q_ItoG << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    p_IinG << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    write();
  }

  void callback_posecovariance(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    timestamp = rclcpp::Time(msg->header.stamp).seconds();
    q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
    p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cov_pos << msg->pose.covariance.at(0), msg->pose.covariance.at(1), msg->pose.covariance.at(2), msg->pose.covariance.at(6),
        msg->pose.covariance.at(7), msg->pose.covariance.at(8), msg->pose.covariance.at(12), msg->pose.covariance.at(13),
        msg->pose.covariance.at(14);
    cov_rot << msg->pose.covariance.at(21), msg->pose.covariance.at(22), msg->pose.covariance.at(23), msg->pose.covariance.at(27),
        msg->pose.covariance.at(28), msg->pose.covariance.at(29), msg->pose.covariance.at(33), msg->pose.covariance.at(34),
        msg->pose.covariance.at(35);
    has_covariance = true;
    write();
  }

  void callback_transform(const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    timestamp = rclcpp::Time(msg->header.stamp).seconds();
    q_ItoG << msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w;
    p_IinG << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;
    write();
  }
#else
  void callback_odometry(const nav_msgs::OdometryPtr &msg) {
    timestamp = msg->header.stamp.toSec();
    q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
    p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cov_pos << msg->pose.covariance.at(0), msg->pose.covariance.at(1), msg->pose.covariance.at(2), msg->pose.covariance.at(6),
        msg->pose.covariance.at(7), msg->pose.covariance.at(8), msg->pose.covariance.at(12), msg->pose.covariance.at(13),
        msg->pose.covariance.at(14);
    cov_rot << msg->pose.covariance.at(21), msg->pose.covariance.at(22), msg->pose.covariance.at(23), msg->pose.covariance.at(27),
        msg->pose.covariance.at(28), msg->pose.covariance.at(29), msg->pose.covariance.at(33), msg->pose.covariance.at(34),
        msg->pose.covariance.at(35);
    has_covariance = true;
    write();
  }

  void callback_pose(const geometry_msgs::PoseStampedPtr &msg) {
    timestamp = msg->header.stamp.toSec();
    q_ItoG << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
    p_IinG << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    write();
  }

  void callback_posecovariance(const geometry_msgs::PoseWithCovarianceStampedPtr &msg) {
    timestamp = msg->header.stamp.toSec();
    q_ItoG << msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
    p_IinG << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    cov_pos << msg->pose.covariance.at(0), msg->pose.covariance.at(1), msg->pose.covariance.at(2), msg->pose.covariance.at(6),
        msg->pose.covariance.at(7), msg->pose.covariance.at(8), msg->pose.covariance.at(12), msg->pose.covariance.at(13),
        msg->pose.covariance.at(14);
    cov_rot << msg->pose.covariance.at(21), msg->pose.covariance.at(22), msg->pose.covariance.at(23), msg->pose.covariance.at(27),
        msg->pose.covariance.at(28), msg->pose.covariance.at(29), msg->pose.covariance.at(33), msg->pose.covariance.at(34),
        msg->pose.covariance.at(35);
    has_covariance = true;
    write();
  }

  void callback_transform(const geometry_msgs::TransformStampedPtr &msg) {
    timestamp = msg->header.stamp.toSec();
    q_ItoG << msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z, msg->transform.rotation.w;
    p_IinG << msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z;
    write();
  }
#endif

protected:

  void write() {
    outfile.precision(5);
    outfile.setf(std::ios::fixed, std::ios::floatfield);
    outfile << timestamp << " ";
    outfile.precision(6);
    outfile << p_IinG.x() << " " << p_IinG.y() << " " << p_IinG.z() << " " << q_ItoG(0) << " " << q_ItoG(1) << " " << q_ItoG(2) << " "
            << q_ItoG(3);
    if (has_covariance) {
      outfile.precision(10);
      outfile << " " << cov_rot(0, 0) << " " << cov_rot(0, 1) << " " << cov_rot(0, 2) << " " << cov_rot(1, 1) << " " << cov_rot(1, 2) << " "
              << cov_rot(2, 2) << " " << cov_pos(0, 0) << " " << cov_pos(0, 1) << " " << cov_pos(0, 2) << " " << cov_pos(1, 1) << " "
              << cov_pos(1, 2) << " " << cov_pos(2, 2) << std::endl;
    } else {
      outfile << std::endl;
    }
  }

  std::ofstream outfile;
  bool has_covariance = false;
  double timestamp;
  Eigen::Vector4d q_ItoG;
  Eigen::Vector3d p_IinG;
  Eigen::Matrix<double, 3, 3> cov_rot;
  Eigen::Matrix<double, 3, 3> cov_pos;
};

} // namespace ov_eval

#endif // OV_EVAL_RECORDER_H
