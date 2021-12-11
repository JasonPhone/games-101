#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "Triangle.hpp"
#include "rasterizer.hpp"

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  Eigen::Matrix4f translate;
  translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2],
      0, 0, 0, 1;

  view = translate * view;

  return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
  Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

  // TODO: Implement this function
  // Create the model matrix for rotating the triangle around the Z axis.
  // Then return it.

  // correct the ang to rad
  float theta = rotation_angle / 180.0 * MY_PI;
  model << cos(theta), -sin(theta), 0, 0,
           sin(theta),  cos(theta), 0, 0,
                    0,           0, 1, 0,
                    0,           0, 0, 1;
  // std::cout << "model\n";
  // std::cout << model << std::endl;
  return model;
}

// Eigen::Matrix4f my_proj(float eye_fov, float aspect_ratio,
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
  // Students will implement this function
  Eigen::Matrix4f to_orthogonal = Eigen::Matrix4f::Identity();
  // TODO: Implement this function
  // Create the projection matrix for the given parameters.
  // Then return it.
  float n = -zNear, f = -zFar;
  eye_fov = eye_fov / 180.0 * MY_PI;
  to_orthogonal << n, 0,     0,      0,
                   0, n,     0,      0,
                   0, 0, n + f, -n * f,
                   0, 0,     1,      0;
  float ner = n, far = f;
  float top = abs(ner) * tan(eye_fov / 2), bot = -top;
  float rit = aspect_ratio * top, lft = -rit;

  Eigen::Matrix4f to_canonical;
  to_canonical << 2 / (rit - lft),               0,               0, 0,
                                0, 2 / (top - bot),               0, 0,
                                0,               0, 2 / (ner - far), 0,
                                0,               0,               0, 1;
  // std::cout << "mine\n" << ner << " " << far << " " << top << " " << bot << " " << rit << " " << lft << std::endl;
  // std::cout << projection << std::endl;
  // std::cout << "------------\n";
  // std::cout << to_canonical << std::endl;

  return to_canonical * to_orthogonal;
}

// Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
//                                       float zNear, float zFar)
// {
//     Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
//     float n = -zNear;
//     float f = -zFar;
//     float t = std::abs(n) * std::tan(.5f * (eye_fov / 180.0 * M_PI));
//     float b = -t;
//     float r = aspect_ratio * t;
//     float l = -r;

//     Eigen::Matrix4f perspective_to_orthogonal = Eigen::Matrix4f::Identity();
//     perspective_to_orthogonal(0,0) = n;
//     perspective_to_orthogonal(1,1) = n;
//     perspective_to_orthogonal(2,2) = n + f;
//     perspective_to_orthogonal(2,3) = -f * n;
//     perspective_to_orthogonal(3,2) = 1;
//     perspective_to_orthogonal(3,3) = 0;

//     Eigen::Matrix4f scale_to_canonical = Eigen::Matrix4f::Identity();
//     // std::cout << 2.f / (r - l) << std::endl;
//     scale_to_canonical(0,0) = 2.f/(r-l);
//     scale_to_canonical(1,1) = 2.f/(t-b);
//     scale_to_canonical(2,2) = 2.f/(n-f);

//     Eigen::Matrix4f orthogonal_projection = scale_to_canonical;
//     projection = orthogonal_projection * perspective_to_orthogonal;

//     std::cout << "right\n" << n << " " << f << " " << t << " " << b << " " << r << " " << l << std::endl;
//     std::cout << perspective_to_orthogonal << std::endl;
//     std::cout << "==================\n";
//     std::cout << scale_to_canonical << std::endl;
//     my_proj(eye_fov, aspect_ratio, zNear, zFar);

//     return projection;
// }

int main(int argc, const char** argv) {
  float angle = 0;
  bool command_line = false;
  std::string filename = "output.png";

  if (argc >= 3) {
    command_line = true;
    angle = std::stof(argv[2]);  // -r by default
    if (argc == 4) {
      filename = std::string(argv[3]);
    } else
      return 0;
  }
  std::cout << filename << " " << angle << std::endl;
  rst::rasterizer r(700, 700);

  Eigen::Vector3f eye_pos = {0, 0, 5};

  std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

  std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

  auto pos_id = r.load_positions(pos);
  auto ind_id = r.load_indices(ind);

  int key = 0;
  int frame_count = 0;

  if (command_line) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);
    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);

    cv::imwrite(filename, image);

    return 0;
  }

  while (key != 27) {
    r.clear(rst::Buffers::Color | rst::Buffers::Depth);

    r.set_model(get_model_matrix(angle));
    r.set_view(get_view_matrix(eye_pos));
    r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

    r.draw(pos_id, ind_id, rst::Primitive::Triangle);

    cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::imshow("image", image);
    key = cv::waitKey(10);

    std::cout << "frame count: " << frame_count++ << '\n';

    if (key == 'a') {
      angle += 10;
    } else if (key == 'd') {
      angle -= 10;
    }
  }

  return 0;
}
