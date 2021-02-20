//
// Created by zzy on 2021/1/14.
//
#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <deque>
#include "ceres/rotation.h"

struct IMUSample
{
  Eigen::Vector3d acc_data;
  Eigen::Vector3d gyro_data;
  double timestamp;
};

struct Param
{
  int dq_size, cur_face, prev_face, face_to_calibrate, static_data_num;
  int light;  // 0-red 1-green
  double g_mag, static_var_thresh;
  std::deque<IMUSample> dq;
  std::vector<std::vector<IMUSample>> static_data;
  std::vector<Eigen::Vector3d> mean_list;
  std::vector<double> scale_bias_vec;  // [s_x, s_y, s_z, b_x, b_y, b_z]
  std::vector<double> angle_axis_vec;  // installation error rotation
  std::vector<double> gyro_bias_vec;

  Param() :
    dq_size(100),
    cur_face(0),
    prev_face(0),
    face_to_calibrate(0),
    g_mag(1.0),
    static_var_thresh(0.01),
    static_data(6),
    mean_list(6),
    static_data_num(500),
    light(0),
    scale_bias_vec(6, 0),
    angle_axis_vec(3, 0),
    gyro_bias_vec(3, 0)
  {
    scale_bias_vec[0] = 1;
    scale_bias_vec[1] = 1;
    scale_bias_vec[2] = 1;
  }
};

struct AccCost
{
  AccCost( double g_mag, Eigen::Vector3d &mean ) :
    g_mag_(g_mag),
    mean_(mean) {}

  template < typename T >
  bool operator() ( const T *params, T *residuals ) const
  {
    Eigen::Matrix< T, 3, 3 > scale_mat;
    Eigen::Matrix< T, 3, 1 > sample_vec, bias_vec, acc_calib;
    scale_mat << T(params[0]), T(0), T(0),
                 T(0), T(params[1]), T(0),
                 T(0), T(0), T(params[2]);
    bias_vec << T(params[3]), T(params[4]), T(params[5]);
    sample_vec << T(mean_[0]), T(mean_[1]), T(mean_[2]);
    acc_calib = scale_mat * ( sample_vec - bias_vec );
    residuals[0] = T(g_mag_) - acc_calib.norm();

    return true;
  }

  const double g_mag_;
  const Eigen::Vector3d mean_;
};

struct InstErrCost
{
  InstErrCost( Eigen::Vector3d &g_vec, Eigen::Vector3d &mean ) :
    g_vec_(g_vec),
    mean_(mean)
    {}

  template < typename T>
  bool operator() ( const T *params, T *residuals ) const
  {
    T sample_vec[3], result[3];
    sample_vec[0] = T( mean_.x() );
    sample_vec[1] = T( mean_.y() );
    sample_vec[2] = T( mean_.z() );
    ceres::AngleAxisRotatePoint(params, sample_vec, result);
    residuals[0] = result[0] - T(g_vec_.x());
    residuals[1] = result[1] - T(g_vec_.y());
    residuals[2] = result[2] - T(g_vec_.z());
    return true;
  }

  const Eigen::Vector3d g_vec_;
  const Eigen::Vector3d mean_;
};

/** @brief Receive a sample of IMU data and calibrate the controller */
void calibrateIMU( const IMUSample &sample, Param &param );

/** @brief Given a sample, return the current face
 *         0: x-axis upwards  1: x-axis downwards
 *         2: y-axis upwards  3: y-axis downwards
 *         4: z-axis upwards  5: z-axis downwards */
int getFace( const IMUSample &sample, const Param &param );

/** @brief Check if current status is static */
bool checkStatic( const IMUSample &sample, Param &param );

/** @brief test functionality from smartbug data */
void importDataFromFile(std::string filename, std::vector<IMUSample> &sample_vec);
