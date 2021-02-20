///*******************************************************
// * This file is part of PISCES.
// * Author: Chence
// *******************************************************/
#pragma once
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <pangolin/display/image_view.h>
#include <sophus/se3.hpp>
#include <sophus/common.hpp>
#include "VisualizationData.h"
#include <unordered_map>

#include "BasaltVisualizer_utils.h"
#include "SafeQueue.h"

namespace yvr {
class Visualizer {
 public:
  using KeypointId = uint32_t;

  Visualizer(
      std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &T_i_c);

  void start_visual_thread();

  void insert_data(int64_t timenow, VioVisualizationData::Ptr data);

  void draw_points_file(std::string file);

  void write_result();

  void read_queue();

  void stop_visual_thread() { stop_show_ = true; }

  bool is_stop() const { return stop_show_; }

  //算法和可视化之间数据共享map
  std::unordered_map<int64_t, VioVisualizationData::Ptr> visualMap_;
  // visual显示数据的顺序队列
  std::vector<int64_t> timeStampArray_;
  // imu位姿队列
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      T_w_i_Array_;
  //相机外参
  std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> T_i_c_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      worldPoints_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  //存储所有的空间三维点
  moodycamel::ConcurrentQueue<std::pair<int64_t, VioVisualizationData::Ptr>>
      dataQueue_;

  int camera_num_ = 2;
  std::atomic<bool> stop_show_;
};
}  // namespace yvr