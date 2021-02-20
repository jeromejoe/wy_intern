///*******************************************************
// * This file is part of PISCES.
// * Author: Chence
// *******************************************************/
#pragma once

#include <memory>
#include <vector>
#include <map>
#include <sophus/se3.hpp>

#include <opencv2/opencv.hpp>

namespace yvr {

struct VioVisualizationData {
  using Ptr = std::shared_ptr<VioVisualizationData>;
  using KeypointId = int;

  VioVisualizationData() {}

  int64_t t_ns;

  Eigen::Vector3d T_w_i;

  std::vector<std::string> imgPaths;
  std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
      keyFramePose;
  std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>
      normalFramePose;

  std::vector<std::map<KeypointId, std::pair<double, double>>> imgPoints;

  std::vector<KeypointId> currentPointsID_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> currentPoints_;
  std::vector<Eigen::Vector3i , Eigen::aligned_allocator<Eigen::Vector3i>> pointsColor_;


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace yvr
