#pragma once
#include <cmath>

using namespace std;

static const float EPSILON = 0.00001;

#include <scan_matching_skeleton/correspond.h>

struct Transform {
  float x_disp, y_disp, theta_rot;

  Transform() : x_disp(0), y_disp(0), theta_rot(0) {}

  Transform(float x_disp, float y_disp, float theta_rot) :
    x_disp(x_disp), y_disp(y_disp), theta_rot(theta_rot) {}

  bool operator==(Transform& t2) {
    return abs(x_disp-t2.x_disp) < EPSILON &&  abs(y_disp-t2.y_disp) < EPSILON &&
           abs(theta_rot - t2.theta_rot) < EPSILON;
  }

  bool operator!=(Transform& t2) {
    return !(*this == t2);
  }

  Transform operator+(Transform& t2) {
    Eigen::Matrix3f H1 = getMatrix();
    Eigen::Matrix3f H2 = t2.getMatrix();
    Eigen::Matrix3f H = H1*H2;
    float x = H(0,2);
    float y = H(1,2);
    float theta = atan2(H(1,0),H(0,0));
    // std::cout<<"H:\n"<<H<<std::endl;
    return Transform(x, y, theta);
  }

  /** keep as a pass-by-value so you can't change the underlying point. */
  Point apply(Point p) {
    float x = p.getX() + x_disp;
    float y = p.getY() + y_disp;
    p.r = sqrt(x*x+y*y);
    p.theta = atan2(y,x);
    p.theta = p.theta + theta_rot;
    return p;
  }

  Eigen::Matrix3f getMatrix() {
    Eigen::Matrix3f mat;
    mat << cos(theta_rot), -sin(theta_rot), x_disp, sin(theta_rot), cos(theta_rot), y_disp, 0, 0, 1;
    return mat;
  }
};


void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points);

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans);
