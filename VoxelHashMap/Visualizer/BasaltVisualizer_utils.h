///*******************************************************
// * This file is part of PISCES.
// * Author: Chence
// *******************************************************/
#pragma once

#include <Eigen/Dense>
#include <pangolin/gl/gldraw.h>

namespace yvr {
const u_int8_t cam_color[3]{250, 0, 26};
const u_int8_t state_color[3]{250, 0, 26};
const u_int8_t pose_color[3]{0, 50, 255};
const u_int8_t gt_color[3]{0, 171, 47};

inline void render_camera(const Eigen::Matrix4d &T_w_c, float lineWidth,
                          const u_int8_t *color, float sizeFactor) {
  const float sz = sizeFactor;
  const float width = 640, height = 480, fx = 500, fy = 500, cx = 320, cy = 240;

  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      lines = {{0, 0, 0},
               {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
               {0, 0, 0},
               {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
               {0, 0, 0},
               {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
               {0, 0, 0},
               {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
               {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
               {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
               {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
               {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
               {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
               {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
               {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
               {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz}};

  glPushMatrix();
  glMultMatrixd(T_w_c.data());
  glColor3ubv(color);
  glLineWidth(lineWidth);
  pangolin::glDrawLines(lines);
  glPopMatrix();
}

inline void getcolor(float p, float np, float &r, float &g, float &b) {
  float inc = 4.0 / np;
  float x = p * inc;
  r = 0.0f;
  g = 0.0f;
  b = 0.0f;

  if ((0 <= x && x <= 1) || (5 <= x && x <= 6))
    r = 1.0f;
  else if (4 <= x && x <= 5)
    r = x - 4;
  else if (1 <= x && x <= 2)
    r = 1.0f - (x - 1);

  if (1 <= x && x <= 3)
    g = 1.0f;
  else if (0 <= x && x <= 1)
    g = x - 0;
  else if (3 <= x && x <= 4)
    g = 1.0f - (x - 3);

  if (3 <= x && x <= 5)
    b = 1.0f;
  else if (2 <= x && x <= 3)
    b = x - 2;
  else if (5 <= x && x <= 6)
    b = 1.0f - (x - 5);
}
}  // namespace yvr