#include "kinematics.hpp"

geometry_msgs::Point jointAnglesToPosition(const sensor_msgs::JointState& joints) {
    geometry_msgs::Point pt;
    std::vector<double> thetas = joints.position;
    if (thetas.size() == 4){
        ROS_INFO("4 angles given, assuming wrist rotation is 0");
        thetas.push_back(0.0);
    }
    Affine3d transform = GST(thetas[0], thetas[1], thetas[2], thetas[3], thetas[4]);
    Vector4d end = transform*Vector4d(0,0,0,1);
    pt.x = end[0];
    pt.y = end[1];
    pt.z = end[2];
    return pt;
}

sensor_msgs::JointState positionToJointAngles(const geometry_msgs::Point& position) {
    sensor_msgs::JointState joint_state;
    std::vector<double> angles(num_joints);
    double x = position.x; double y = position.y; double z = position.z;
    double adjusted_r = sqrt(x*x + y*y) - d4;
    double adjusted_z = z - d1;
    angles[0] = theta1(y,x);
    angles[1] = theta2(adjusted_r, adjusted_z);
    angles[2] = theta3(adjusted_r, adjusted_z);
    angles[3] = theta4(angles[1], angles[2]);

    joint_state.position = angles;

    return joint_state;
}
