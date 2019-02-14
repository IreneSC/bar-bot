#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

constexpr int num_joints = 5;
const std::vector<std::string> names = {"joint1", "joint2", "joint3", "joint4", "joint5"};
const std::string target_subscriber_name("/gripper_position");
const std::string joint_state_name("/joint_states");

// Lengths in meters of arm components
double d1 = .0311;
double a2 = .0611;
double a3 = .15556;
double d5 = .14142;

ros::Publisher joint_state_publisher;

static inline double sq(double x) {
    return x*x;
}

/* Calculates the link angles required to move the tool to a position/rotation */
sensor_msgs::JointState position2angles(const geometry_msgs::Point& position) {
    sensor_msgs::JointState joint_state;
    std::vector<double> angles(num_joints);
    double x = position.x; double y = position.y; double z = position.z;

    /* Equations that are re-used throughout the following code */
    double equation1; double equation2; double equation3;

    // ensure that w/in acos [-1,1] range
    equation1 = (sq(x) + sq(y) + sq(z-d1 + d5) - sq(a2) - sq(a3)) / (2*sq(a3));

    // If out of range, do not update
    if (abs(equation1) > 1) {
        std::cout << "Invalid position: " << position << std::endl;
        // for (auto const& elt : position)
        //     std::cout << elt << ", ";
        // std::cout << "]" << std::endl;
        return joint_state;
    }

    /* kinematic equations TODO: label these angles */
    angles[0] = atan2(y,x); // RANGE : [-Pi,Pi]
    angles[2] = acos(equation1); // RANGE : [0,Pi]

    equation2 = (-a3*sin(angles[2]) - sqrt(equation3)) /
                    (a2 + a3*cos(angles[2]) + x/cos(angles[0]));
    equation3 = sq((a2 + a3*cos(angles[2]))) + sq(a3*sin(angles[2])) -
                    sq(x/cos(angles[0]));

    /* Check equations */
    if (equation3 < 0) {
        std::cout << "Warning: eq3 < 0 for position: " << position << std::endl;
        equation3 = 0;
    }

    /* kinematic equations TODO: label these angles */
    angles[1] = 2 * atan(equation2); // RANGE : [-Pi/2,Pi/2]
    angles[3] = -(angles[1] + angles[2]); // RANGE : [-2*Pi,2*Pi]

    // Convert the values to degrees
    // for (int i = 0; i < num_joints; i++){
    //     angles[i] = angles[i]/DEGREE;
    // }
    joint_state.position = angles;
    return joint_state;
}

// const geometry_msgs::PoseStamped& target_pose

void processTargetState(const geometry_msgs::PoseStamped& target_pose) {
    const geometry_msgs::Point&      target_loc = target_pose.pose.position;
    const geometry_msgs::Quaternion& target_ori = target_pose.pose.orientation;
    sensor_msgs::JointState angles = position2angles(target_loc);
    angles.header.stamp = ros::Time::now();
    angles.name = names;
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

    /* Test: */
    // std::vector<double>

    ros::spin();

    ros::shutdown();

    return 0;
}

