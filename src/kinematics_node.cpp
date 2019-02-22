#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "kinematics.hpp"
#include "tf/transform_datatypes.h"
#include "HebiHelper.hpp"

static const std::string target_subscriber_name("/gripper_position");
static const std::string joint_state_name("/joint_states");

static ros::Time prev_time;

ros::Publisher joint_state_publisher;

void processTargetState(const geometry_msgs::PoseStamped& target_pose) {
    const geometry_msgs::Point&      target_loc = target_pose.pose.position;
    const geometry_msgs::Quaternion& target_ori = target_pose.pose.orientation;
    sensor_msgs::JointState angles = positionToJointAngles(target_loc);
    angles.header.stamp = ros::Time::now();
    angles.name = joint_names;

    // Set the roll of the gripper
    tf::Quaternion q;
    tf::quaternionMsgToTF(target_ori, q);
    tf::Matrix3x3 m(q);
    // Roll pitch yaw
    double r, p, y;
    m.getRPY(r, p, y);
    std::cout << "sizes: " << num_joints - 1 << ", " << angles.position.size() << std::endl;
    angles.position[num_joints-1] = r;

    joint_state_publisher.publish(angles);
    std::cout << "Publised: " << angles << std::endl;
}

void followTrajectory() {
    // Advance time, but hold at t=0 to stay at the final position.
    // t = ros::now() - prev_time;
    // if (t > 0.0)
    //     t = 0.0;

    // // Compute the new position and velocity commands.
    // for (i = 0 ; i < 5 ; i++)
    // {
    //     q[i]    = a[i]+t*(b[i]+t*(c[i]+t*d[i]));
    //     qdot[i] = b[i]+t*(2.0*c[i]+t*3.0*d[i]);

    //     cmdMsg.position[i] = q[i];
    //     cmdMsg.velocity[i] = qdot[i];
    // }

    // // Publish.
    // cmdMsg.header.stamp = ros::Time::now();
    // cmdPub.publish(cmdMsg);

    // // Wait for next cycle.
    // ros::spinOnce();
    // loop_rate.sleep();
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
    std::vector<std::string> families = {"Arm", "Arm", "Arm", "Arm", "Arm", "Arm"};
    std::vector<std::string> names  = {"base", "pitch_1", "pitch_2", "pitch_3", "wrist", "gripper"};
    HebiHelper helper("all", names, families);

#define TEST_ANGLE

    /* Test: */
#ifdef TEST_ANGLE
    // std::vector<double> angles = {0, 0, -M_PI/2, 0};
    while(ros::ok()) {
        if (!helper.isValid()) {
            ROS_INFO("waiting for valid feedback");
            ros::spinOnce();
            continue;
        }
        // std::vector<double> angles = {0, 0, 0, 0};
        // std::vector<double> angles = {0.1, 0.2, -0.4, -.2+.4};
        sensor_msgs::JointState joints = helper.getFeedback();
        joints.header.stamp = ros::Time::now();
        joints.name = joint_names;
        joints.position.pop_back();
        joint_state_publisher.publish(joints);
        // joints.position = angles;

        // geometry_msgs::Point positions;
        // positions.x = .23;
        // positions.y = .40;
        // positions.z = 0.11;


        // auto ret = positionToJointAngles(jointAnglesToPosition(joints));
        // auto ret = jointAnglesToPosition(joints);
        // // std::cout << ret << std::endl;
        // geometry_msgs::PoseStamped target_pose;
        // target_pose.pose.position = ret;

        // tf::Quaternion q;
        // q.setRPY(joints.position[num_joints-1],0,0);
        // tf::quaternionTFToMsg(q, target_pose.pose.orientation);

        // processTargetState(target_pose);
        // std::cout << "after ret" <<std::endl;
        ros::spinOnce();
    }
    // std::cout << positionToJointAngles(jointAnglesToPosition(ret)) << std::endl;
#else
    geometry_msgs::Point positions;
    positions.x = .45;
    positions.y = 0;
    positions.z = 0.0861;
    auto ret = jointAnglesToPosition(positionToJointAngles(positions));
    std::cout << ret << std::endl;
    std::cout << jointAnglesToPosition(positionToJointAngles(ret)) << std::endl;
    std::cout << "Done printing" << std::endl;
#endif


    ros::shutdown();

    return 0;
}
