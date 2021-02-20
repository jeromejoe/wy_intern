/*******************************************************
 * This file is part of PISCES.
 * Author: Chence
 *******************************************************/
#pragma once
#include <string>
#include <iostream>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <map>
#include <unordered_map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace yvr {

struct TrueData {
  uint64_t t_ns;
  Sophus::SE3d pose;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class TrueDataReader {
 public:
  using Ptr = std::shared_ptr<TrueDataReader>;
  TrueDataReader(const std::string &true_file) {
    std::ifstream f(true_file);
    if (f.fail()) {
      std::cout << "true file NULL" << std::endl;
      std::abort();
    }
    std::string line;
    while (std::getline(f, line)) {
      if (line[0] == '#') continue;
      for (auto &c : line) {
        if (c == ',') c = ' ';
      }
      std::stringstream ss(line);
      uint64_t timestamp;
      TrueData trueData;
      Eigen::Vector3d position;
      Eigen::Quaterniond q;

      ss >> timestamp;
      trueData.t_ns = timestamp;
      ss >> position(0);
      ss >> position(1);
      ss >> position(2);
      ss >> q.w();
      ss >> q.x();
      ss >> q.y();
      ss >> q.z();

      trueData.t_ns = timestamp;
      trueData.pose = Sophus::SE3d(q, position);

      time_index_[timestamp] = length_;
      true_Data_.push_back(trueData);
      length_++;
    }
    successInit_ = true;
  }

  bool getPose(const uint64_t nowTimestamp, TrueData &pose_data) {
    if (nowTimestamp <= true_Data_[0].t_ns || nowTimestamp > true_Data_[true_Data_.size() - 1].t_ns)
      return false;
    int index = 0;
    while (true_Data_[index].t_ns < nowTimestamp)
      index++;

    double w = (nowTimestamp - true_Data_[index - 1].t_ns) * 1.0 / (true_Data_[index].t_ns - true_Data_[index - 1].t_ns);
    pose_data.t_ns = nowTimestamp;
    Eigen::Quaterniond q1,q2,q_res;
    q1 = true_Data_[index-1].pose.so3().unit_quaternion();
    q2 = true_Data_[index].pose.so3().unit_quaternion();
    q_res = q1.slerp(w, q2);

    pose_data.pose.so3() = Sophus::SO3d(q_res);
    pose_data.pose.translation() = (1 - w) * true_Data_[index - 1].pose.translation() + w * true_Data_[index].pose.translation();
    return true;
  }
  
  std::vector<TrueData> true_Data_;

 private:
  bool successInit_ = false;
  
  std::unordered_map<uint64_t, int> time_index_;
  int length_ = 0;
};
};  // namespace yvr
