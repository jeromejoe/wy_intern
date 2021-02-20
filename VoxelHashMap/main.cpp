#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "VoxelHashMap.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"
#include "Visualizer/BasaltVisualizer.h"
#include "Visualizer/BasaltVisualizer_utils.h"
#include "Visualizer/SafeQueue.h"
#include "Visualizer/VisualizationData.h"
#include <chrono>
#include "TrueValueReader.h"


using namespace std;

vector<point3d> load_data();
vector< point3d > naiveCheckPointInView( vector< point3d > &points, Sophus::SE3d T_c, CameraParam c_param );
void drawPoints_naive( vector< point3d > point_vec, std::vector<yvr::TrueData> T_c_vec, CameraParam c_param );
void drawPoints( VoxelHashMap &vxl_map, std::vector<yvr::TrueData> T_c_vec );


int Voxel::count = 0;

int main()
{
  string cam_pos_filename = "../Data/pose_ground_true.csv";
  yvr::TrueDataReader data_reader( cam_pos_filename ); // camera pose
  vector< point3d > data_points = load_data();
  VoxelHashMap voxel_map;
  CameraParam c_param = { fu: 458.654, fv: 457.296, cu: 367.215, cv:248.375, resolution: {752, 480} };

//  drawPoints_naive( data_points, data_reader.true_Data_, c_param );

  for ( auto p : data_points )
  {
    voxel_map.insertPoint( p );
  }

  voxel_map.sampleFrustumPoints( c_param, 24, 100, 0.3, 8 );

  drawPoints( voxel_map, data_reader.true_Data_ );

  return 0;
}

vector<point3d> load_data()
{
  vector<point3d> data;
  string file_name = "../Data/V1_01_easy_voxel_downsample.ply";
  string line;
  ifstream inFile;
  inFile.open( file_name.c_str() );
  if ( inFile.is_open() )
  {
    for ( int i = 0; i < 8; ++i ) getline( inFile, line );
    while ( getline( inFile, line ) )
    {
      point3d point;
      istringstream iss(line);
      iss >> point.x >> point.y >> point.z;
      data.push_back(point);
    }
  }
  return data;
}

vector< point3d > naiveCheckPointInView( vector< point3d > &points, Sophus::SE3d T_c, CameraParam c_param )
{
  vector< point3d > result_points;
  Sophus::Matrix3d K;
  K << c_param.fu, 0, c_param.cu,
       0, c_param.fv, c_param.cv,
       0, 0 , 1;
  for ( auto &p : points )
  {
    point3d result_point( p.x, p.y, p.z );
    Sophus::Vector3d point_w( p.x, p.y, p.z );
    Sophus::Vector3d point_c = T_c.inverse() * point_w;
    Sophus::Vector3d point_pixel = K * ( point_c / point_c.z() );
    if ( point_pixel.x() >= 0 && point_pixel.x() <= c_param.resolution[0] &&
      point_pixel.y() >= 0 && point_pixel.y() <= c_param.resolution[1] &&
      point_c.z() > 0 )
    {
      result_point.in_view = true;
    }
    result_points.push_back( result_point );
  }
  return result_points;
}

void drawPoints_naive( vector< point3d > point_vec, std::vector<yvr::TrueData> T_c_vec, CameraParam c_param )
{
  Sophus::SE3d T_i_c(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> T_i_c_cams;
  T_i_c_cams.push_back(T_i_c);
  yvr::Visualizer visualizer(T_i_c_cams);
  std::thread t1([&] {visualizer.start_visual_thread(); });
  yvr::VioVisualizationData::Ptr data;

  for ( int i = 0; i < T_c_vec.size(); ++i ) {
    data.reset(new yvr::VioVisualizationData());
    data->t_ns = T_c_vec[i].t_ns;
    data->keyFramePose.emplace_back(T_c_vec[i].pose);

    vector<point3d> result_points;
//    if (i % 1000 == 0)
//    {
      chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
      result_points = naiveCheckPointInView(point_vec, T_c_vec[i].pose, c_param);
      chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
      chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
      cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
//    }
//    else
//    {
//      result_points = naiveCheckPointInView(point_vec, T_c_vec[i].pose, c_param);
//    }

    for (int j = 0; j < result_points.size(); ++j)
    {
      data->currentPointsID_.push_back(j);
      data->currentPoints_.push_back(Eigen::Vector3d(result_points[j].x, result_points[j].y, result_points[j].z));
      if (result_points[j].in_view)
        data->pointsColor_.push_back(Eigen::Vector3i(0, 255, 0));
      else
        data->pointsColor_.push_back(Eigen::Vector3i(125, 0, 255));
    }
    visualizer.insert_data(data->t_ns, data);

  }

  char stop_signal;
  cin >> stop_signal;

  visualizer.stop_visual_thread();
  t1.join();
}

void drawPoints( VoxelHashMap &vxl_map, vector<yvr::TrueData> T_c_vec )
{
  Sophus::SE3d T_i_c(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> T_i_c_cams;
  T_i_c_cams.push_back(T_i_c);
  yvr::Visualizer visualizer(T_i_c_cams);
  std::thread t1([&] {visualizer.start_visual_thread(); });
  yvr::VioVisualizationData::Ptr data;

//  int64_t timestamp = 10;
  for ( int i = 0; i < T_c_vec.size(); ++i ) {
    data.reset(new yvr::VioVisualizationData());
    data->t_ns = T_c_vec[i].t_ns;
    data->keyFramePose.emplace_back(T_c_vec[i].pose);

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    unordered_set<int> vxl_set = vxl_map.findVoxelInView( 0, T_c_vec[i].pose );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    int id = 0;
    for (auto &bucket : vxl_map.hashTable()) {
      for (auto &vxl : bucket) {
        for (auto &p : vxl.point_list) {
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(p.x, p.y, p.z));
          if (vxl_set.count(vxl.id))
            data->pointsColor_.push_back(Eigen::Vector3i(0, 255, 0));
          else
            data->pointsColor_.push_back(Eigen::Vector3i(0, 0, 255));
        }
        // draw voxel
        /*
        if ( vxl_set.count(vxl.id) )
        {
          double voxel_size = 0.1;
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size - 5 , vxl.y * voxel_size - 5, vxl.z * voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size + voxel_size - 5 , vxl.y * voxel_size - 5, vxl.z * voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size - 5 , vxl.y * voxel_size + voxel_size - 5, vxl.z * voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size + voxel_size - 5 , vxl.y * voxel_size + voxel_size - 5, vxl.z * voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));

          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size - 5 , vxl.y * voxel_size - 5, vxl.z * voxel_size + voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size + voxel_size - 5 , vxl.y * voxel_size - 5, vxl.z * voxel_size + voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size - 5 , vxl.y * voxel_size + voxel_size - 5, vxl.z * voxel_size + voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
          data->currentPointsID_.push_back(id++);
          data->currentPoints_.push_back(Eigen::Vector3d(vxl.x * voxel_size + voxel_size - 5 , vxl.y * voxel_size + voxel_size - 5, vxl.z * voxel_size + voxel_size ));
          data->pointsColor_.push_back(Eigen::Vector3i(255, 0, 0));
        }
        */
      }
    }
    visualizer.insert_data(T_c_vec[i].t_ns, data);
  }

  char stop_signal;
  cin >> stop_signal;

  visualizer.stop_visual_thread();
  t1.join();
}