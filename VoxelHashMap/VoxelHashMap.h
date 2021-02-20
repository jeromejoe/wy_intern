//
// Created by zzy on 2021/2/3.
//

#pragma once

#include <vector>
#include <cmath>
#include "sophus/se3.hpp"
#include <unordered_set>

struct point3i
{
  int x, y, z;
};

struct point3d
{
  double x, y, z;
  bool in_view;
  point3d() : x(0), y(0), z(0), in_view(false){}
  point3d(double x_, double y_, double z_ ) : x(x_), y(y_), z(z_), in_view(false){}
};

struct CameraParam
{
  double fu, fv, cu, cv;
  int resolution[2];
};

struct Voxel
{
  int x, y, z;
  int id;
  static int count;
  std::vector<point3d> point_list;
  Voxel() :
    x(-1), y(-1), z(-1) { id = ++count; }
};

class VoxelHashMap
{
public:
  /** @brief Default constructor */
  VoxelHashMap() :
    voxel_size( 0.1 ),
    coord_offset( 5.0 ),
    dis_thresh( 0.005 ),
    hashtable_size( 5000 ),
    bucket_size ( 10 ),
    hash_table( hashtable_size )
    {
      for ( auto bucket : hash_table )
        bucket.reserve( bucket_size );
    }

  /** @brief return the modifiable hash table */
  std::vector< std::vector< Voxel > >& hashTable() { return hash_table; }

  /** @brief insert a point into hash map */
  void insertPoint( point3d point );

  /** @brief Sample discrete points along rays in the frustum in camera frame */
  void sampleFrustumPoints( CameraParam c_param, int grid_size, int num_raypoints, double D_min, double D_max );

  /** @brief Return points that can be viewed given a camera position */
  std::vector< point3d > getPointInView( int occlude_thresh, Sophus::SE3d T_c );

  /** @brief Find voxels' indices that can be viewed given a camera position */
  std::unordered_set< int > findVoxelInView( int occlude_thresh, Sophus::SE3d T_c );


private:
  double voxel_size;
  double coord_offset;  // to make coordinates positive
  int dis_thresh;
  int hashtable_size;
  int bucket_size;
  std::vector< std::vector< Voxel > > hash_table;
  std::vector< std::vector< point3d > > frustum_samples;
  std::unordered_set< int > voxels_in_view;

  /** @brief Given a voxel's coordinates, calculate the its hash key */
  int calculateHashKey( point3i voxel_coord );

  /** @brief Given a point's coordinates, calculate its voxel's coordinates */
  point3i getVoxelCoordFromPoint( point3d point );

  /** @brief Distance between two points */
  double pointDistance( point3d p1, point3d p2 )
  {
    return std::sqrt( pow(p1.x - p2.x, 2) +
                        pow(p1.y - p2.y, 2) +
                        pow(p1.z - p2.z, 2));
  }

};


