#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
using namespace Eigen;
using namespace std;
int main() {
  // P(2, 1)
  // 1. rotate 45 deg ccw around origin
  // 2. translation (+1, +2)
  float theta = 45.0 / 180.0 * acos(-1);
  // cout << cos(theta) << endl;
  Vector3f v{2.0f, 1.0f, 1.0f};
  Matrix3f rotat, trans;
  rotat << cos(theta), -sin(theta), 0.0f, sin(theta), cos(theta), 0.0f, 0.0f,
      0.0f, 1.0f;
  trans << 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 2.0f, 0.0f, 0.0f, 1.0f;
  v = rotat * v;
  // cout << v << endl;
  v = trans * v;
  cout << v << endl;

  return 0;
}
