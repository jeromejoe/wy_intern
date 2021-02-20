//
// Created by zzy on 2021/2/3.
//

#include <iostream>
#include <cstdint>
#include <Eigen/Core>
#include "VoxelHashMap.h"
#include "sophus/se3.hpp"

void VoxelHashMap::insertPoint(point3d point)
{
  point3i voxel_coord = getVoxelCoordFromPoint( point );
  int hash_key = calculateHashKey( voxel_coord );
  if ( hash_key < 0 ) return;
//  std::cout << hash_key << std::endl;

  // find target voxel
  for ( auto& vxl : hash_table[hash_key] )
  {
    if ( vxl.x == voxel_coord.x && vxl.x == voxel_coord.y && vxl.x == voxel_coord.z )
    {
//      for ( auto& p : vxl.point_list )
//      {
//        if ( pointDistance( p, point ) < dis_thresh )
//        {
//          // update point description here
//          return;
//        }
//      }
      vxl.point_list.push_back( point );
      return;
    }
  }
  // if not found, check bucket size and create new voxel
  if ( hash_table[hash_key].size() >= bucket_size ) return;
  Voxel new_voxel;
  new_voxel.x = voxel_coord.x;
  new_voxel.y = voxel_coord.y;
  new_voxel.z = voxel_coord.z;
  new_voxel.point_list.push_back( point );
  hash_table[hash_key].push_back( new_voxel );

  return;
}

point3i VoxelHashMap::getVoxelCoordFromPoint( point3d point )
{
  point3i voxel_coord;
  voxel_coord.x = ( point.x + coord_offset ) / voxel_size;
  voxel_coord.y = ( point.y + coord_offset ) / voxel_size;
  voxel_coord.z = ( point.z ) / voxel_size;
  return voxel_coord;
}

int VoxelHashMap::calculateHashKey(point3i voxel_coord)
{
  int64_t p1 = 73856093, p2 = 19349669, p3 = 83492791;
  int64_t voxel_x = int64_t(voxel_coord.x);
  int64_t voxel_y = int64_t(voxel_coord.y);
  int64_t voxel_z = int64_t(voxel_coord.z);
  int hash_key = ( (voxel_x*p1)^(voxel_y*p2)^(voxel_z*p3) ) % hashtable_size;
  return hash_key;
}

void VoxelHashMap::sampleFrustumPoints( CameraParam c_param, int grid_size, int num_raypoints, double D_min, double D_max )
{
  frustum_samples.clear();
  double D_delta = ( D_max - D_min ) / num_raypoints;
  for ( int u = 0; u < c_param.resolution[0] / grid_size; ++u )
  {
    for ( int v = 0; v < c_param.resolution[1] / grid_size; ++v )
    {
      int grid_center_u = grid_size * u + grid_size / 2;
      int grid_center_v = grid_size * v + grid_size / 2;
      double normal_plane_x = ( grid_center_u - c_param.cu ) / c_param.fu;
      double normal_plane_y = ( grid_center_v - c_param.cv ) / c_param.fv;
      Eigen::Vector3d ray( normal_plane_x, normal_plane_y, 1 );
      ray.normalize();
      std::vector< point3d > ray_points;
      for ( int s = 0; s < num_raypoints; ++s )
      {
        double scale = ( D_min + s * D_delta ) / ray.z();
        point3d sample( scale*ray.x(), scale*ray.y(), scale*ray.z() );
        ray_points.push_back( sample );
      }
      frustum_samples.push_back( ray_points );
    }
  }
  return;
}

std::vector< point3d > VoxelHashMap::getPointInView( int occlude_thresh, Sophus::SE3d T_c )
{
  std::vector< point3d > result_points;
  for ( auto &ray : frustum_samples )
  {
    for ( auto &sample : ray )
    {
      Sophus::Vector3d point_c( sample.x, sample.y, sample.z );
      Sophus::Vector3d point_w = T_c * point_c;
      point3d sample_w( point_w.x(), point_w.y(), point_w.z() );
      point3i vxl_coord = getVoxelCoordFromPoint( sample_w );
      int hash_key = calculateHashKey( vxl_coord );
      if ( hash_key < 0 ) continue;
      if ( hash_table[hash_key].size() == 0 ) continue;
      int num_points_found = 0;
      for ( auto &vxl : hash_table[hash_key] )
      {
        if ( vxl.x == vxl_coord.x && vxl.y == vxl_coord.y && vxl.z == vxl_coord.z )
        {
          for ( auto &p : vxl.point_list )
          {
            point3d point( p.x, p.y, p.z );
            point.in_view = true;
            result_points.push_back( point );
            num_points_found ++;
          }
          break;
        }
      }
      if ( num_points_found > occlude_thresh ) break;
    }
  }
  return result_points;
}

std::unordered_set< int > VoxelHashMap::findVoxelInView( int occlude_thresh, Sophus::SE3d T_c )
{
  voxels_in_view.clear();
  for ( auto &ray : frustum_samples )
  {
    for ( auto &sample : ray )
    {
      Sophus::Vector3d point_c( sample.x, sample.y, sample.z );
      Sophus::Vector3d point_w = T_c * point_c;
      point3d sample_w( point_w.x(), point_w.y(), point_w.z() );
      point3i vxl_coord = getVoxelCoordFromPoint( sample_w );
      int hash_key = calculateHashKey( vxl_coord );
      if ( hash_key < 0 ) continue;
      if ( hash_table[hash_key].size() == 0 ) continue;
      int num_points_found = 0;
      for ( auto &vxl : hash_table[hash_key] )
      {
        if ( vxl.x == vxl_coord.x && vxl.y == vxl_coord.y && vxl.z == vxl_coord.z )
        {
          voxels_in_view.insert( vxl.id );
          num_points_found = vxl.point_list.size();
          break;
        }
      }
      if ( num_points_found > occlude_thresh ) break;
    }
  }
  return voxels_in_view;
}

