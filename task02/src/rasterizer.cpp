// clang-format no
//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <math.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <vector>

#define _MSAA_ENABLED_

rst::pos_buf_id rst::rasterizer::load_positions(
    const std::vector<Eigen::Vector3f>& positions) {
  auto id = get_next_id();
  pos_buf.emplace(id, positions);

  return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(
    const std::vector<Eigen::Vector3i>& indices) {
  auto id = get_next_id();
  ind_buf.emplace(id, indices);

  return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(
    const std::vector<Eigen::Vector3f>& cols) {
  auto id = get_next_id();
  col_buf.emplace(id, cols);

  return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
  return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector3f* _v) {
  // TODO : Implement this function to check if the point (x, y) is inside the
  // triangle represented by _v[0], _v[1], _v[2]
  Eigen::Vector3f P{x, y, 1};
  Eigen::Vector3f AB{_v[1] - _v[0]}, BC{_v[2] - _v[1]}, CA{_v[0] - _v[2]};
  Eigen::Vector3f AP{P - _v[0]}, BP{P - _v[1]}, CP{P - _v[2]};
  Eigen::Vector3f cAP = AB.cross(AP), cBP = BC.cross(BP), cCP = CA.cross(CP);
  double signa = cAP.z(), signb = cBP.z(), signc = cCP.z();
  if (signa <= 0 && signb <= 0 && signc <= 0) return true;
  if (signa >= 0 && signb >= 0 && signc >= 0) return true;
  return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y,
                                                            const Vector3f* v) {
  float c1 =
      (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y +
       v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
      (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() +
       v[1].x() * v[2].y() - v[2].x() * v[1].y());
  float c2 =
      (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y +
       v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
      (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() +
       v[2].x() * v[0].y() - v[0].x() * v[2].y());
  float c3 =
      (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y +
       v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
      (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() +
       v[0].x() * v[1].y() - v[1].x() * v[0].y());
  return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer,
                           col_buf_id col_buffer, Primitive type) {
  auto& buf = pos_buf[pos_buffer.pos_id];
  auto& ind = ind_buf[ind_buffer.ind_id];
  auto& col = col_buf[col_buffer.col_id];

  float f1 = (50 - 0.1) / 2.0;
  float f2 = (50 + 0.1) / 2.0;

  Eigen::Matrix4f mvp = projection * view * model;
  for (auto& i : ind) {
    Triangle t;
    Eigen::Vector4f v[] = {mvp * to_vec4(buf[i[0]], 1.0f),
                           mvp * to_vec4(buf[i[1]], 1.0f),
                           mvp * to_vec4(buf[i[2]], 1.0f)};
    // Homogeneous division
    for (auto& vec : v) {
      vec /= vec.w();
    }
    // Viewport transformation
    for (auto& vert : v) {
      vert.x() = 0.5 * width * (vert.x() + 1.0);
      vert.y() = 0.5 * height * (vert.y() + 1.0);
      vert.z() = vert.z() * f1 + f2;
    }

    for (int i = 0; i < 3; ++i) {
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
      t.setVertex(i, v[i].head<3>());
    }

    auto col_x = col[i[0]];
    auto col_y = col[i[1]];
    auto col_z = col[i[2]];

    t.setColor(0, col_x[0], col_x[1], col_x[2]);
    t.setColor(1, col_y[0], col_y[1], col_y[2]);
    t.setColor(2, col_z[0], col_z[1], col_z[2]);

    rasterize_triangle(t);
  }
  // dump msaa result into dst buffers
  for (int i = 0; i < msaa_frame.size(); i++) {
    // average depth and color
    float avg_depth = 0;
    // std::cout << "color " << i << std::endl;
    Eigen::Vector3f avg_color{0, 0, 0};
    for (int ii = 0; ii < 4; ii++) {
      avg_depth += msaa_depth[i][ii];
      avg_color += msaa_frame[i][ii];
      // std::cout << msaa_frame[i][ii] << std::endl;
      // std::cout << "------\n";
    }
    // std::cout << "==============\n";
    depth_buf[i] = avg_depth / 4.0f;
    frame_buf[i] = avg_color / 4.0f;
  }
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
  auto v = t.toVector4();

  // TODO : Find out the bounding box of current triangle.
  // iterate through the pixel and find if the current pixel is inside the
  // triangle
  int lft_bound = std::min(t.v[0].x(), std::min(t.v[1].x(), t.v[2].x()));
  int rit_bound = std::max(t.v[0].x(), std::max(t.v[1].x(), t.v[2].x()));
  int bot_bound = std::min(t.v[0].y(), std::min(t.v[1].y(), t.v[2].y()));
  int upp_bound = std::max(t.v[0].y(), std::max(t.v[1].y(), t.v[2].y()));
  float msaa_dir[4][2] = {
      {-0.25, -0.25}, {-0.25, +0.25}, {+0.25, -0.25}, {+0.25, +0.25}};

  for (int x = lft_bound; x <= rit_bound; x++) {
    for (int y = bot_bound; y <= upp_bound; y++) {
      for (int i = 0; i < 4; i++) {
        float dx = x + msaa_dir[i][0], dy = y + msaa_dir[i][1];
        if (insideTriangle(dx, dy, t.v)) {
          // std::cout << dx << " " << dy << std::endl;
          // If so, use the following code to get the interpolated z value.
          auto [alpha, beta, gamma] = computeBarycentric2D(dx, dy, t.v);
          float w_reciprocal =
              1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
          float z_interpolated = alpha * v[0].z() / v[0].w() +
                                 beta * v[1].z() / v[1].w() +
                                 gamma * v[2].z() / v[2].w();
          z_interpolated *= w_reciprocal;
          // TODO : set the current pixel (use the set_pixel function) to the
          // color of the triangle (use getColor function) if it should be
          // painted.
          // std::cout << "inter z: " << z_interpolated << std::endl;
          // std::cout << "depth_buf: " << depth_buf[get_index(x, y)] <<
          // std::endl;
          float buffer_idx = get_index(x, y);
          std::vector<float>& msaa_d = msaa_depth[buffer_idx];
          if (msaa_d[i] == std::numeric_limits<float>::infinity() ||
              msaa_d[i] < z_interpolated) {
            msaa_frame[buffer_idx][i] = t.getColor();
            msaa_depth[buffer_idx][i] = z_interpolated;
          }
        }
      }
    }
  }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) { model = m; }

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) { view = v; }

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {
  projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
  if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
    std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    for (int i = 0; i < msaa_frame.size(); i++) {
      msaa_frame[i].resize(4);
      std::fill(msaa_frame[i].begin(), msaa_frame[i].end(),
                Eigen::Vector3f{0, 0, 0});
    }
  }
  if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
    std::fill(depth_buf.begin(), depth_buf.end(),
              std::numeric_limits<float>::infinity());
    for (int i = 0; i < msaa_depth.size(); i++) {
      msaa_depth[i].resize(4);
      std::fill(msaa_depth[i].begin(), msaa_depth[i].end(),
                std::numeric_limits<float>::infinity());
    }
  }
}
// TODO : dump the buffers
rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
  frame_buf.resize(w * h);
  depth_buf.resize(w * h);
  ////////////////
  msaa_frame.resize(w * h);
  msaa_depth.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y) {
  return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point,
                                const Eigen::Vector3f& color) {
  // old index: auto ind = point.y() + point.x() * width;
  auto ind = (height - 1 - point.y()) * width + point.x();
  frame_buf[ind] = color;
}

// clang-format on