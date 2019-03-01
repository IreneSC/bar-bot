#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

static const std::vector<std::string> joint_names = {"z_rotate", "pitch_1", "pitch_2", "pitch_3", "yaw"};

static constexpr double d1 = .11;
static constexpr double d2 = .463;
static constexpr double d3 = .463;
static constexpr double d4 = .108;
constexpr int num_joints = 5;


using namespace Eigen;

static inline Affine3d rotation_x(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(1, 0, 0)));
}
static inline Affine3d rotation_y(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(0, 1, 0)));
}
static inline Affine3d rotation_z(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(0, 0, 1)));
}
static inline Affine3d translation(double x, double y, double z){
    return Affine3d(Translation3d(Vector3d(x,y,z)));
}

static inline Affine3d G1(double theta){
    return rotation_z(theta);
}
static inline Affine3d G2(double d, double theta){
    return translation(0,0,d)*rotation_x(theta);
}
static inline Affine3d G3(double d, double theta){
    return translation(0,d,0)*rotation_x(theta);
}
static inline Affine3d G4(double d, double theta){
    return translation(0,d,0)*rotation_x(theta);
}
static inline Affine3d G5(double d, double theta){
    return translation(0,d,0)*rotation_y(theta);
}
static inline Affine3d GST(double theta1, double theta2, double theta3,
    double theta4, double theta5){
    return G1(theta1)*G2(d1, theta2)*G3(d2, theta3)*G4(d3, theta4)*G5(d4, theta5);
}

static inline double theta1(double y,double x){
    return atan2(-x,y);
}
static inline double theta2(double r, double z){
    double alpha = atan2(z,r); // angle to z from horizontal
    // std::cout << "alpha: " << alpha << std::endl;
    //angle from z up to arm using law of cosines
    double acos_arg = (d2*d2 + r*r + z*z - d3*d3)/(2*d2*sqrt(r*r + z*z));
    // std::cout << "acos arg theta2: " << acos_arg << std::endl;
    if (abs(acos_arg) > 1) {
        ROS_WARN("acos arg too big: %f", acos_arg);
        acos_arg /= abs(acos_arg);
    }
    double beta = acos(acos_arg);
    return alpha + beta;
}
static inline double theta3(double r,double z){
    double acos_arg = (d2*d2 + d3*d3 -r*r - z*z)/(2*d2*d3);
    // std::cout << "acos arg theta3: " << acos_arg << std::endl;
    if (abs(acos_arg) > 1) {
        ROS_WARN("acos arg too big: %f", acos_arg);
        acos_arg /= abs(acos_arg);
    }
    return acos(acos_arg) - M_PI;
}

// TODO: flip this as well?
static inline double theta4(double theta2, double theta3){
    return -theta2 - theta3;
}

geometry_msgs::Point jointAnglesToPosition(const sensor_msgs::JointState& joints);
sensor_msgs::JointState positionToJointAngles(const geometry_msgs::Point& joints);
#endif
