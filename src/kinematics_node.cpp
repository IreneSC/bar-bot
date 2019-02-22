#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "kinematics.hpp"
#include "tf/transform_datatypes.h"

constexpr int num_joints = 6;
const std::vector<std::string> names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_tip"};
const std::string target_subscriber_name("/gripper_position");
const std::string joint_state_name("/joint_states");

ros::Publisher joint_state_publisher;

static inline double sq(double x) {
    return x*x;
}
void processTargetState(const geometry_msgs::PoseStamped& target_pose) {
    const geometry_msgs::Point&      target_loc = target_pose.pose.position;
    const geometry_msgs::Quaternion& target_ori = target_pose.pose.orientation;
    sensor_msgs::JointState angles = position2angles(target_loc);
    angles.header.stamp = ros::Time::now();
    angles.name = names;

    // Set the roll of the gripper
    tf::Quaternion q;
    tf::quaternionMsgToTF(target_ori, q);
    tf::Matrix3x3 m(q);
    // Roll pitch yaw
    double r, p, y;
    m.getRPY(r, p, y);
    angles.position[num_joints-1] = r;

    joint_state_publisher.publish(angles);
    std::cout << "Publised: " << angles << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics_node");
    ros::start();

    ros::NodeHandle node_handler;

    joint_state_publisher =
        node_handler.advertise<sensor_msgs::JointState>(joint_state_name, 1);

    ros::Subscriber target_subscriber =
        node_handler.subscribe(target_subscriber_name, 10,
                              &processTargetState);

#define TEST_ANGLE

    /* Test: */
#ifdef TEST_ANGLE
    // std::vector<double> angles = {0, 0, -M_PI/2, 0};
    std::vector<double> angles = {0, 0, 0, 0};
    sensor_msgs::JointState joints;
    joints.position = angles;
    auto ret = position2angles(jointangles2position(joints));
    std::cout << ret << std::endl;
    std::cout << "after ret" <<std::endl;
    std::cout << position2angles(jointangles2position(ret)) << std::endl;
#else
    geometry_msgs::Point positions;
    positions.x = .45;
    positions.y = 0;
    positions.z = 0.0861;
    auto ret = jointangles2position(position2angles(positions));
    std::cout << ret << std::endl;
    std::cout << jointangles2position(position2angles(ret)) << std::endl;
    std::cout << "Done printing" << std::endl;
#endif


    ros::spin();

    ros::shutdown();

    return 0;
}
