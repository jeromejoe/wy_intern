///*******************************************************
// * This file is part of PISCES.
// * Author: Chence
// *******************************************************/
#include "BasaltVisualizer.h"
#include <thread>
namespace yvr {

constexpr int UI_WIDTH = 200;
constexpr int IMG_WIDTH = 320;
constexpr int IMG_HEIGHT = 240;

// GUI function
void draw_image_overlay(pangolin::View &v, Visualizer *vi, int camera_id);
void draw_scene(pangolin::View &view, Visualizer *vi);

// Pangolin menu
pangolin::Var<int> show_frame("menu.show_frame", -1, -1, 1500);
pangolin::Var<bool> follow("menu.follow", true, false, true);
pangolin::Var<bool> show_ids("menu.show_ids", false, false, true);
pangolin::Var<bool> show_continue("menu.continue", true, false, true);

// 3D View Camera
pangolin::OpenGlRenderState camera;

yvr::Visualizer::Visualizer(
    std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &T_i_c) : stop_show_(false) {
  T_i_c_ = T_i_c;
  camera_num_ = T_i_c.size();
  dataQueue_ = moodycamel::ConcurrentQueue<
      std::pair<int64_t, VioVisualizationData::Ptr>>(50);
  T_w_i_Array_.reserve(5000);
  timeStampArray_.reserve(5000);  // TODO:
  // 总数据读取超过5000时，清空或者丢弃数据
}

void yvr::Visualizer::start_visual_thread() {
  pangolin::CreateWindowAndBind("Pices", 1800, 1000);
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));


  // Define Camera Render Object (for view / scene browsing)
  camera = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(1800, 1000, 700, 700, 900, 500, 0.001, 10000),
      pangolin::ModelViewLookAt(0, 0.5, -3, 0, 0, 0, pangolin::AxisY));

  pangolin::View &display3D = pangolin::CreateDisplay()
      .SetAspect(1800 / 1000.0)
      .SetBounds(0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0)
      .SetHandler(new pangolin::Handler3D(camera));

  display3D.extern_draw_function =
      std::bind(&(draw_scene), std::placeholders::_1, this);

  pangolin::View &img_view_display = pangolin::CreateDisplay()
      .SetBounds(pangolin::Attach::Pix(1000 - IMG_HEIGHT), 1.0, pangolin::Attach::Pix(UI_WIDTH), pangolin::Attach::Pix(UI_WIDTH + 2 * IMG_WIDTH))
      .SetLayout(pangolin::LayoutEqualHorizontal);

  std::vector<std::shared_ptr<pangolin::ImageView>> img_view;
  for (int i = 0; i < camera_num_; ++i) {
    std::shared_ptr<pangolin::ImageView> iv(new pangolin::ImageView);
    //iv->SetBounds(0, 0.5, 0, pangolin::Attach::Pix(IMG_WIDTH));
    // size_t idx = img_view.size();
    img_view.push_back(iv);
    img_view_display.AddDisplay(*iv);
    iv->extern_draw_function =
        std::bind(&(draw_image_overlay), std::placeholders::_1, this, i);
  }

  while (!stop_show_) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    if (pangolin::ShouldQuit()) stop_show_ = true;

    if (follow && show_frame != -1) {
      int frame_id = show_frame;
      Eigen::Vector3d trans;
      trans = T_w_i_Array_[frame_id];

      Sophus::SE3d T_W_I;
      T_W_I.so3() = Sophus::SO3d();
      T_W_I.translation() = trans;
      camera.Follow(T_W_I.matrix());
    }

    display3D.Activate(camera);

    if (show_frame.GuiChanged()) {
      int frame_id = show_frame;
      int64_t timestamp = timeStampArray_[frame_id];
      auto data = visualMap_[timestamp];

//      if ((int)data->imgPaths.size() != camera_num_) {
//          if(data->imgPaths.size() == 1){
//              std::vector<cv::Mat> imgs(camera_num_);
//              if (std::experimental::filesystem::exists(data->imgPaths[0])) {
//                cv::Mat stereo_img = cv::imread(data->imgPaths[0], cv::IMREAD_GRAYSCALE);
//                imgs[0] = stereo_img(cv::Range(0, stereo_img.rows), cv::Range(0, (int)stereo_img.cols / 2)).clone();
//                imgs[1] = stereo_img(cv::Range(0, stereo_img.rows), cv::Range((int)stereo_img.cols / 2, stereo_img.cols)).clone();
//                // cv::imshow("left", imgs[0]);
//                // cv::imshow("right", imgs[1]);
//                // cv::waitKey(1);
//              }else{
//                continue;
//              }
//              for (size_t j = 0; j < imgs.size(); ++j) {
//                cv::Mat img = imgs[j];
//                if (img.type() == CV_16UC1) {
//                  pangolin::GlPixFormat fmt;
//                  fmt.glformat = GL_LUMINANCE;
//                  fmt.gltype = GL_UNSIGNED_SHORT;  // 16位
//                  fmt.scalable_internal_format = GL_LUMINANCE16;
//                  img_view[j]->SetImage(img.data, img.cols, img.rows,
//                                        img.cols * sizeof(unsigned short), fmt);
//
//                } else if (img.type() == CV_8UC1) {
//                  pangolin::GlPixFormat fmt;
//                  fmt.glformat = GL_LUMINANCE;
//                  fmt.gltype = GL_UNSIGNED_BYTE;  // 8位
//                  fmt.scalable_internal_format = GL_LUMINANCE8;
//                  img_view[j]->SetImage(img.data, img.cols, img.rows,
//                                        img.cols * sizeof(unsigned char), fmt);
//                }
//              }
//          }
//      } else {
//        for (size_t j = 0; j < data->imgPaths.size(); ++j) {
//          cv::Mat img = cv::imread(data->imgPaths[j], cv::IMREAD_GRAYSCALE);
//          if (img.empty()) {
//            std::cout << "Visualize Error: Can't Read Img" << std::endl;
//            break;
//          }
//          if (img.type() == CV_16UC1) {
//            pangolin::GlPixFormat fmt;
//            fmt.glformat = GL_LUMINANCE;
//            fmt.gltype = GL_UNSIGNED_SHORT;  // 16位
//            fmt.scalable_internal_format = GL_LUMINANCE16;
//            img_view[j]->SetImage(img.data, img.cols, img.rows,
//                                  img.cols * sizeof(unsigned short), fmt);
//
//          } else if (img.type() == CV_8UC1) {
//            pangolin::GlPixFormat fmt;
//            fmt.glformat = GL_LUMINANCE;
//            fmt.gltype = GL_UNSIGNED_BYTE;  // 8位
//            fmt.scalable_internal_format = GL_LUMINANCE8;
//            img_view[j]->SetImage(img.data, img.cols, img.rows,
//                                  img.cols * sizeof(unsigned char), fmt);
//          }
//        }
//      }
      //      leftImg = cv::imread(leftCamImgPath + std::to_string(timestamp) +
      //      ".png", cv::IMREAD_GRAYSCALE); rightImg =
      //      cv::imread(rightCamImgPath + std::to_string(timestamp) + ".png",
      //      cv::IMREAD_GRAYSCALE);
      //
      //      pangolin::GlPixFormat fmt;
      //      fmt.glformat = GL_LUMINANCE;
      //      fmt.gltype = GL_UNSIGNED_BYTE; //8位
      //      fmt.scalable_internal_format = GL_LUMINANCE8;
      ////    fmt.gltype = GL_UNSIGNED_SHORT; //16位
      ////    fmt.scalable_internal_format = GL_LUMINANCE16;

      //      for (int n = 0; n < camera_num_ / 2; ++n) {
      //        img_view[2 * n]->SetImage(leftImg.data, leftImg.cols,
      //        leftImg.rows, leftImg.cols * sizeof(unsigned char), fmt);
      //        img_view[2 * n + 1]->SetImage(rightImg.data, rightImg.cols,
      //        rightImg.rows, rightImg.cols * sizeof(unsigned char), fmt);
      //      }
    }
    pangolin::FinishFrame();

    if (show_continue && show_frame < int(timeStampArray_.size()) - 1) {
      show_frame = show_frame + 1;
      show_frame.Meta().gui_changed = true;
    }

    read_queue();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  pangolin::DestroyWindow("Pices");
  std::cout << "DestroyWindow" << std::endl;
}

void yvr::Visualizer::insert_data(int64_t timenow,
                                  yvr::VioVisualizationData::Ptr data) {
  dataQueue_.enqueue(std::make_pair(timenow, data));
}

void yvr::Visualizer::draw_points_file(std::string file) {
  std::ifstream f(file);
  std::string line;
  worldPoints_.reserve(10000);
  while (std::getline(f, line)) {
    if (line[0] == '#') continue;
    for (auto &c : line) {
      if (c == ',') c = ' ';
    }
    std::stringstream ss(line);
    int64_t timestamp;

    Eigen::Vector3d point;
    ss >> point(0);
    ss >> point(1);
    ss >> point(2);

    worldPoints_.push_back(point);
  }
}

void yvr::Visualizer::read_queue() {
  std::pair<int64_t, yvr::VioVisualizationData::Ptr> data;
  bool getData = dataQueue_.try_dequeue(data);
  if (getData) {
    if (visualMap_.count(data.first) != 0) {
      std::cout << "Visualization Error: Currect Data Already Exists!"
                << std::endl;
      return;
    }
    visualMap_[data.first] = data.second;
    T_w_i_Array_.push_back(data.second->T_w_i);
    timeStampArray_.push_back(data.first);
  }
}

void draw_image_overlay(pangolin::View &v, yvr::Visualizer *vi, int camera_id) {
  if (show_frame < 0) return;

  int frame_id = show_frame;
  int64_t timestamp = vi->timeStampArray_[frame_id];
  auto data = vi->visualMap_[timestamp];

  if (static_cast<int>(data->imgPoints.size()) <= camera_id) {
    return;
  }

  glLineWidth(1.0);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  std::map<int, std::pair<double, double>> points = data->imgPoints[camera_id];

  for (auto &point : points) {
    pangolin::glDrawCirclePerimeter(point.second.first, point.second.second,
                                    2.0f);

    if (show_ids)
      pangolin::GlFont::I()
          .Text("%d", point.first)
          .Draw(5 + point.second.first, 5 + point.second.second);
  }
}

void draw_scene(pangolin::View &v, yvr::Visualizer *vi) {
  if (show_frame < 0) return;

  v.Activate(camera);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  glPointSize(3);
  glColor3f(1.0, 0.0, 0.0);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glColor3ubv(cam_color);
  if (!vi->timeStampArray_.empty()) {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
        traj(vi->T_w_i_Array_.begin(),
             vi->T_w_i_Array_.begin() + show_frame + 1);
    pangolin::glDrawLineStrip(traj);
  }

  glColor3ubv(gt_color);

  size_t frame_id = show_frame;
  int64_t timestamp = vi->timeStampArray_[frame_id];
  auto data = vi->visualMap_[timestamp];

  // KeyFrame
  for (const auto &p : data->keyFramePose)
    for (size_t i = 0; i < vi->T_i_c_.size(); i++)
      render_camera((p * vi->T_i_c_[i]).matrix(), 2.0f, state_color, 0.1f);

  // NormalFrame
  for (const auto &p : data->normalFramePose)
    for (size_t i = 0; i < vi->T_i_c_.size(); i++)
      render_camera((p * vi->T_i_c_[i]).matrix(), 2.0f, pose_color, 0.1f);

  if (data->pointsColor_.size() != data->currentPoints_.size()) {
    glColor3ubv(pose_color);
    pangolin::glDrawPoints(data->currentPoints_);
  } else {
    glEnable(GL_POINT_SMOOTH);
    glPointSize(3);
    glBegin(GL_POINTS);
    for (size_t ii = 0; ii < data->currentPoints_.size(); ++ii) {
      glColor3d(data->pointsColor_[ii](0), data->pointsColor_[ii](1), data->pointsColor_[ii](2));
      glVertex3d(data->currentPoints_[ii](0), data->currentPoints_[ii](1), data->currentPoints_[ii](2));
    }
    glEnd();
  }

  if(vi->worldPoints_.size() != 0){
    glEnable(GL_POINT_SMOOTH);
    glPointSize(3);
    glBegin(GL_POINTS);
    for (size_t ii = 0; ii < vi->worldPoints_.size(); ++ii) {
      glColor3d(0, 255, 0);
      glVertex3d(vi->worldPoints_[ii](0), vi->worldPoints_[ii](1), vi->worldPoints_[ii](2));
    }
    glEnd();
  }

  pangolin::glDrawAxis(Sophus::SE3d().matrix(), 1.0);
}

}  // namespace yvr