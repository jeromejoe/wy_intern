#include <iostream>
#include <string>
#include "calculation.h"
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

using namespace std;

int main( int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
//  string fileName = "../data/smartbug_acc_1.csv";
  string fileName = "../data/6_faces_acc_gyro.csv";

  Param param;
  vector< IMUSample > samples;
  importDataFromFile(fileName, samples);
  cout << samples.size() << endl;

  for ( int i = 0; i < samples.size(); i++ )
  {
    calibrateIMU(samples[i], param);
  }

  cout << "scale & bias:" << endl;
  for (int i = 0; i < 6; i++)
    cout << param.scale_bias_vec[i] << "  ";
  cout << endl;

  cout << "angle axis:" << endl;
  for (int i = 0; i < 3; i++)
    cout << param.angle_axis_vec[i] << "  ";
  cout << endl;

  cout << "gyro bias:" << endl;
  for (int i = 0; i < 3; i++)
    cout << param.gyro_bias_vec[i] << "  ";
  cout << endl;

  return 0;
}
