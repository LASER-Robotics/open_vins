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
#include <nav_msgs/msg/path.hpp>
#else
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#endif

#include "alignment/AlignTrajectory.h"
#include "alignment/AlignUtils.h"
#include "utils/Loader.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include <boost/filesystem.hpp>

#ifdef ROS2
using PathMsg = nav_msgs::msg::Path;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
rclcpp::Publisher<PathMsg>::SharedPtr pub_path;
void align_and_publish(const PathMsg::SharedPtr msg);
#else
using PathMsg = nav_msgs::Path;
using PoseStampedMsg = geometry_msgs::PoseStamped;
ros::Publisher pub_path;
void align_and_publish(const PathMsg::ConstPtr &msg);
#endif

std::vector<double> times_gt;
std::vector<Eigen::Matrix<double, 7, 1>> poses_gt;
std::string alignment_type;

int main(int argc, char **argv) {

#ifdef ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("live_align_trajectory");
  
  std::string verbosity;
  node->declare_parameter<std::string>("verbosity", "INFO");
  node->get_parameter("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  node->declare_parameter<std::string>("alignment_type", "posyaw");
  node->get_parameter("alignment_type", alignment_type);

  node->declare_parameter<std::string>("path_gt", "");
  std::string path_to_gt;
  node->get_parameter("path_gt", path_to_gt);
  if (path_to_gt.empty()) {
    std::cerr << "[LOAD]: Please provide a groundtruth file path!!!" << std::endl;
    std::exit(EXIT_FAILURE);
  }
#else
  ros::init(argc, argv, "live_align_trajectory");
  ros::NodeHandle nh("~");

  std::string verbosity;
  nh.param<std::string>("verbosity", verbosity, "INFO");
  ov_core::Printer::setPrintLevel(verbosity);

  nh.param<std::string>("alignment_type", alignment_type, "posyaw");

  if (!nh.hasParam("path_gt")) {
    ROS_ERROR("[LOAD]: Please provide a groundtruth file path!!!");
    std::exit(EXIT_FAILURE);
  }
  std::string path_to_gt;
  nh.param<std::string>("path_gt", path_to_gt, std::string(""));
#endif

  boost::filesystem::path infolder(path_to_gt);
  if (infolder.extension() == ".csv") {
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    ov_eval::Loader::load_data_csv(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
  } else {
    std::vector<Eigen::Matrix3d> cov_ori_temp, cov_pos_temp;
    ov_eval::Loader::load_data(path_to_gt, times_gt, poses_gt, cov_ori_temp, cov_pos_temp);
  }

#ifdef ROS2
  auto sub = node->create_subscription<PathMsg>("/ov_msckf/pathimu", 1, align_and_publish);
  pub_path = node->create_publisher<PathMsg>("/ov_msckf/pathgt", 2);
  RCLCPP_INFO(node->get_logger(), "Subscribing: %s", sub->get_topic_name());
  RCLCPP_INFO(node->get_logger(), "Publishing: %s", pub_path->get_topic_name());
  rclcpp::spin(node);
  rclcpp::shutdown();
#else
  ros::Subscriber sub = nh.subscribe("/ov_msckf/pathimu", 1, align_and_publish);
  pub_path = nh.advertise<PathMsg>("/ov_msckf/pathgt", 2);
  ROS_INFO("Subscribing: %s", sub.getTopic().c_str());
  ROS_INFO("Publishing: %s", pub_path.getTopic().c_str());
  ros::spin();
#endif
  return EXIT_SUCCESS;
}

#ifdef ROS2
void align_and_publish(const PathMsg::SharedPtr msg) {
#else
void align_and_publish(const PathMsg::ConstPtr &msg) {
#endif

  std::vector<double> times_temp;
  std::vector<Eigen::Matrix<double, 7, 1>> poses_temp;
  for (auto const &pose : msg->poses) {
#ifdef ROS2
    times_temp.push_back(rclcpp::Time(pose.header.stamp).seconds());
#else
    times_temp.push_back(pose.header.stamp.toSec());
#endif
    Eigen::Matrix<double, 7, 1> pose_tmp;
    pose_tmp << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w;
    poses_temp.push_back(pose_tmp);
  }

  std::vector<double> gt_times_temp = times_gt;
  std::vector<Eigen::Matrix<double, 7, 1>> gt_poses_temp = poses_gt;
  ov_eval::AlignUtils::perform_association(0, 0.02, times_temp, gt_times_temp, poses_temp, gt_poses_temp);

  if (poses_temp.size() < 3) {
    PRINT_ERROR(RED "[TRAJ]: unable to get enough common timestamps between trajectories.\n" RESET);
    PRINT_ERROR(RED "[TRAJ]: does the estimated trajectory publish the rosbag timestamps??\n" RESET);
    return;
  }

  Eigen::Matrix3d R_ESTtoGT;
  Eigen::Vector3d t_ESTinGT;
  double s_ESTtoGT;
  ov_eval::AlignTrajectory::align_trajectory(poses_temp, gt_poses_temp, R_ESTtoGT, t_ESTinGT, s_ESTtoGT, alignment_type);
  Eigen::Vector4d q_ESTtoGT = ov_core::rot_2_quat(R_ESTtoGT);
  PRINT_DEBUG("[TRAJ]: q_ESTtoGT = %.3f, %.3f, %.3f, %.3f | p_ESTinGT = %.3f, %.3f, %.3f | s = %.2f\n", q_ESTtoGT(0), q_ESTtoGT(1),
              q_ESTtoGT(2), q_ESTtoGT(3), t_ESTinGT(0), t_ESTinGT(1), t_ESTinGT(2), s_ESTtoGT);

  PathMsg arr_groundtruth;
  arr_groundtruth.header = msg->header;
  for (size_t i = 0; i < gt_times_temp.size(); i += std::floor(gt_times_temp.size() / 16384.0) + 1) {

    double timestamp = gt_times_temp.at(i);
    Eigen::Matrix<double, 7, 1> pose_inGT = gt_poses_temp.at(i);
    Eigen::Vector3d pos_IinEST = R_ESTtoGT.transpose() * (pose_inGT.block(0, 0, 3, 1) - t_ESTinGT) / s_ESTtoGT;
    Eigen::Vector4d quat_ESTtoI = ov_core::quat_multiply(pose_inGT.block(3, 0, 4, 1), q_ESTtoGT);
    
    PoseStampedMsg posetemp;
    posetemp.header = msg->header;
#ifdef ROS2
    posetemp.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
#else
    posetemp.header.stamp = ros::Time(timestamp);
#endif
    posetemp.pose.orientation.x = quat_ESTtoI(0);
    posetemp.pose.orientation.y = quat_ESTtoI(1);
    posetemp.pose.orientation.z = quat_ESTtoI(2);
    posetemp.pose.orientation.w = quat_ESTtoI(3);
    posetemp.pose.position.x = pos_IinEST(0);
    posetemp.pose.position.y = pos_IinEST(1);
    posetemp.pose.position.z = pos_IinEST(2);
    arr_groundtruth.poses.push_back(posetemp);
  }
  pub_path->publish(arr_groundtruth);
}
