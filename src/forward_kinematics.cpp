#include "forward_kinematics.hpp"

geometry_msgs::Point jointangles2position(const sensor_msgs::JointState& joints) {
    geometry_msgs::Point pt;
    std::vector<double> thetas = joints.position;
    if (thetas.size() == 4){
        ROS_INFO("4 angles given, assuming wrist rotation is 0");
        thetas.push_back(0.0);
    }
    else if (thetas.size() != 5){
        ROS_ERROR("Incorrect number of angles given");
        return pt;
    }
    Affine3d transform = GST(d1, thetas[0], thetas[1], d3, thetas[2], d4, thetas[3], d5, thetas[4]);
    Vector4d end = transform*Vector4d(0,0,0,1);
    pt.x = end[0];
    pt.y = end[1];
    pt.z = end[2];

}
