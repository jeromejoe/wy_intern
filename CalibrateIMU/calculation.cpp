//
// Created by zzy on 2021/1/14.
//

#include <iostream>
#include <cstdlib>
#include <vector>
#include <fstream>
#include "ceres/ceres.h"
#include "calculation.h"

using namespace std;

void calibrateIMU( const IMUSample &sample, Param &param )
{
  // Check if finished
  if ( param.face_to_calibrate > 5 )
  {
//    cout << "Finished!" << endl;
    return;
  }

  // Update dq
  if ( param.dq.size() < param.dq_size )
  {
    param.dq.push_back(sample);
//    cout << "Filling dq" << endl;
    return ;
  }
  else
  {
    param.dq.push_back(sample);
    param.dq.pop_front();
  }

  // update face
  param.prev_face = param.cur_face;
  param.cur_face = getFace( sample, param );

  // set light
  if ( param.light == 1 && param.cur_face != param.prev_face )
    param.light = 0;
  else if ( param.light == 1 && param.cur_face == param.prev_face )
  {
//    cout << "Green Light, plz change face." << endl;
    return ;
  }

  // check if current face is correct
  if ( param.cur_face != param.face_to_calibrate )
  {
//    cout << "Wrong face! Current face should be: " << param.face_to_calibrate << endl;
    return ;
  }

  // check if current status is static
  if ( !checkStatic( sample, param ) )
  {
//    cout << "Not static yet" << endl;
    return ;
  }

  // push data into corresponding vector
  if ( param.static_data[param.face_to_calibrate].size() < param.static_data_num )
  {
    param.static_data[param.face_to_calibrate].push_back(sample);
//    cout << "Collecting data for face " << param.cur_face << endl;
  }
  // Do calculations
  else
  {
    // calculate acc mean
    Eigen::Vector3d mean(0, 0, 0);
    for (auto s : param.static_data[param.face_to_calibrate])
      mean += s.acc_data;
    mean /= param.static_data_num;
    param.mean_list[param.face_to_calibrate] = mean;

    if ( param.face_to_calibrate == 5 )
    {
      // calculate gyro bias
      Eigen::Vector3d gyro_bias(0, 0, 0);
      for ( auto s_vec : param.static_data )
        for ( auto s : s_vec )
          gyro_bias += s.gyro_data;
      gyro_bias /= 6 * param.static_data_num;
      param.gyro_bias_vec[0] = gyro_bias.x();
      param.gyro_bias_vec[1] = gyro_bias.y();
      param.gyro_bias_vec[2] = gyro_bias.z();

      // Optimization of scale and bias using 6 mean values
      ceres::Problem problem;
      for (int i = 0; i < 6; i++)
      {
        ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<AccCost, 1, 6>(
            new AccCost(param.g_mag, param.mean_list[i]));

        problem.AddResidualBlock(cost_function, NULL, param.scale_bias_vec.data());
      }
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.minimizer_progress_to_stdout = true;

      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
      std::cout << summary.BriefReport() << "\n\n" << std::endl;

      // Optimization of rotation matrix (installation error)
      ceres::Problem problem2;
      for (int i = 0; i < 6; ++i)
      {
        Eigen::Vector3d g_vec(0, 0, 0);
        g_vec(i/2) = i % 2 ? -1 : 1;
        Eigen::Vector3d mean_calib;
        mean_calib.x() = param.scale_bias_vec[0] * (param.mean_list[i].x() - param.scale_bias_vec[3]);
        mean_calib.y() = param.scale_bias_vec[1] * (param.mean_list[i].y() - param.scale_bias_vec[4]);
        mean_calib.z() = param.scale_bias_vec[2] * (param.mean_list[i].z() - param.scale_bias_vec[5]);

        ceres::CostFunction *cost_function =
          new ceres::AutoDiffCostFunction<InstErrCost, 3, 3>(
            new InstErrCost( g_vec, mean_calib ));
        problem2.AddResidualBlock(cost_function, NULL, param.angle_axis_vec.data());
      }
      ceres::Solver::Options options2;
      options2.linear_solver_type = ceres::DENSE_QR;
      options2.minimizer_progress_to_stdout = true;

      ceres::Solver::Summary summary2;
      ceres::Solve(options2, &problem2, &summary2);
      std::cout << summary2.BriefReport() << "\n\n" << std::endl;

    }
    // set status
    param.face_to_calibrate ++;
    param.light = 1;
//    cout << "This face is done, move to next face" << endl;

  }

  return ;
}

int getFace( const IMUSample &sample, const Param &param )
{
  double residual, threshold = 0.01;
  for ( int i = 0; i < 6; i++)
  {
    residual = sample.acc_data[i/2] + ( i % 2 ? 1 : -1 ) * param.g_mag;
    if ( std::abs(residual) < threshold ) return i;
  }

  return -1;
}


bool checkStatic( const IMUSample &sample, Param &param )
{
  Eigen::Vector3d mean(0, 0, 0);
  for (auto s : param.dq)
    mean += s.acc_data;
  mean /= param.dq_size;

  Eigen::Vector3d variance(0, 0, 0);
  for (auto s : param.dq)
  {
    Eigen::Vector3d diff = s.acc_data - mean;
    variance += (diff.array() * diff.array()).matrix();
  }
  variance /= param.dq_size - 1;

  if ( variance.norm() < param.static_var_thresh ) return true;
  return false;
}

void importDataFromFile(string filename, vector<IMUSample> &sample_vec) {
  string line;
  ifstream infile;
  double ts, d[6];

  infile.open(filename.c_str());
  if (infile.is_open()) {
    int l = 0;
    getline(infile, line);
    while (getline(infile, line)) {
      int res = sscanf(line.data(), "%*lf,%*d,%lf,%lf,%lf", &d[0], &d[1], &d[2]);
      if (res != 3) {
        cout << "importData error in line " << 2 * l + 1 << endl;
      }
      else {
        getline(infile, line);
        sscanf(line.data(), "%*lf,%*d,%lf,%lf,%lf", &d[3], &d[4], &d[5]);
        IMUSample temp_node;
        temp_node.acc_data << d[0], d[1], d[2];
        temp_node.gyro_data << d[3], d[4], d[5];
        sample_vec.push_back(temp_node);
      }
      l++;
    }
    infile.close();
  }
}
