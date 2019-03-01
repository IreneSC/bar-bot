#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

static std::string cup_detection_topic;
static std::string target_position_topic;
static std::string target_gripper_state_topic;
static std::string current_pose_topic;

static ros::Publisher target_position_publisher;
static ros::Publisher target_gripper_state_publisher;
static ros::Subscriber cup_detection_subscriber;
static ros::Subscriber current_pose_subscriber;

static geometry_msgs::PointStamped last_cup_pos;
static geometry_msgs::PointStamped last_arm_pos;

static const float scanning_z = .20;//m
static const float scanning_r = .50;//m
static float scanning_speed = -2.5;//rad/s
static const float scanning_limit_left = 0.0;//rad
static const float scanning_limit_right = -2 * M_PI/3.0;//rad

static ros::Time last_cup_detection(0);
static ros::Duration detection_expiration(1.0);


void processCupPoint(const geometry_msgs::PointStamped& cup_position) {
    last_cup_pos = cup_position;
    last_cup_detection = cup_position.header.stamp;
    std::cout << last_cup_pos << std::endl;
    ROS_INFO("got a cup");
}

void processArmPose(const geometry_msgs::PoseStamped& arm_pose) {
    last_arm_pos.header = arm_pose.header;
    last_arm_pos.point = arm_pose.pose.position;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "commander_node");
    ros::start();
    ros::Rate loop_rate(200);

    ros::NodeHandle nh;

    // last_arm_pos = geometry_msgs::PointStamped();
    last_arm_pos.point.x = .3;
    last_arm_pos.point.y = 0;
    last_arm_pos.point.z = .1;
    // last_cup_pos = geometry_msgs::PointStamped();
    last_cup_pos.point.x = .3;
    last_cup_pos.point.y = 0;
    last_cup_pos.point.z = .10;

    if (!nh.getParam("cup_detection_topic", cup_detection_topic))
    {
      ROS_ERROR("cup_detection_topic param not specified");
      return -1;
    }
    if (!nh.getParam("target_position_topic", target_position_topic))
    {
      ROS_ERROR("target_position_topic param not specified");
      return -1;
    }
    if (!nh.getParam("target_gripper_state_topic", target_gripper_state_topic))
    {
      ROS_ERROR("target_gripper_state_topic param not specified");
      return -1;
    }
    if (!nh.getParam("current_pose_topic", current_pose_topic))
    {
      ROS_ERROR("current_pose_topic param not specified");
      return -1;
    }

    target_position_publisher
        = nh.advertise<geometry_msgs::PointStamped>(target_position_topic, 1);
    target_gripper_state_publisher
        = nh.advertise<std_msgs::Bool>(target_gripper_state_topic, 1);

    cup_detection_subscriber = nh.subscribe(cup_detection_topic, 10,
        &processCupPoint);
    current_pose_subscriber = nh.subscribe(current_pose_topic, 10,
        &processArmPose);


    int count = 0;

    while(ros::ok()){
        double goal_theta = atan2(last_arm_pos.point.y , last_arm_pos.point.x);
        //scan for cup
        if ((ros::Time::now() - last_cup_detection) <= detection_expiration) {
            if (count++ % 100 == 0)
                target_position_publisher.publish(last_cup_pos);
        }
        loop_rate.sleep();
        ros::spinOnce();
        // while ((ros::Time::now() - last_cup_detection) > detection_expiration){
        //     double current_theta = atan2(last_arm_pos.point.y , last_arm_pos.point.x);
        //     if(current_theta > scanning_limit_left){
        //         scanning_speed = -fabs(scanning_speed);
        //     }
        //     else if(current_theta < scanning_limit_right){
        //         scanning_speed = fabs(scanning_speed);
        //     }
        //     goal_theta += loop_rate.cycleTime().toSec() * scanning_speed;
        //     geometry_msgs::PointStamped target_pos;
        //     target_pos.point.x = scanning_r * cos(goal_theta);
        //     target_pos.point.y = scanning_r * sin(goal_theta);
        //     target_pos.point.z = scanning_z;
        //     target_pos.header.stamp = ros::Time::now();
        //     if (count++ % 100 == 0) {
        //         // target_position_publisher.publish(target_pos);
        //         std::cout << "target_pos: " << target_pos << std::endl;
        //         ROS_INFO("curr_theta: %f", current_theta);
        //     }
        //     ros::spinOnce();
        //     loop_rate.sleep();
        // }
        // target_position_publisher.publish(last_cup_pos);
        // loop_rate.sleep();
        // ros::spinOnce();
      }

    ros::shutdown();

    return 0;
}
