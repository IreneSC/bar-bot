#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

// #define DEBUG_MODE

// Lengths in meters of arm components
#ifdef DEBUG_MODE
static constexpr double d1 = 1;
static constexpr double d3 = 1;
static constexpr double d4 = 1;
static constexpr double d5 = 1;
#else
static constexpr double d1 = .0861;
static constexpr double d3 = .15556;
static constexpr double d4 = .14142;
static constexpr double d5 = .1827;
#endif

using namespace Eigen;
static inline Affine3d rotation_z(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(0, 0, 1)));
}
static inline Affine3d rotation_x(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(1, 0, 0)));
}
static inline Affine3d translation(double x, double y, double z){
    return Affine3d(Translation3d(Vector3d(x,y,z)));
}

static inline Affine3d G1(double d, double theta){
    return translation(0,0,d)*rotation_z(theta);
}
static inline Affine3d G2(double theta){
    return rotation_x(-M_PI/2.0)*rotation_z(theta);
}
static inline Affine3d G3(double d, double theta){
    return translation(d,0,0)*rotation_z(theta);
}
static inline Affine3d G4(double d, double theta){
    return translation(d,0,0)*rotation_z(theta);
}
static inline Affine3d G5(double d){
// #ifndef DEBUG_MODE
    // return rotation_x(M_PI/2.0)*translation(0,0,-d);
// #else
    return rotation_x(M_PI/2.0)*translation(d,0,0);
// #endif
}
static inline Affine3d G6(double theta){
    return rotation_z(theta);
}
static inline Affine3d GST(double d1, double theta1, double theta2, double d3, double theta3,
            double d4, double theta4, double d5, double theta6){
    return G1(d1, theta1)*G2(theta2)*G3(d3, theta3)*G4(d4,theta4)*G5(d5)*G6(theta6);
}

geometry_msgs::Point jointangles2position(const sensor_msgs::JointState& joints);
#endif
