#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "bar_bot/Mobility.h"

static std::string cup_detection_topic;
static std::string target_position_topic;
static std::string target_gripper_state_topic;
static std::string current_pose_topic;

static ros::ServiceClient mobility_client;
// static ros::Publisher target_position_publisher;
// static ros::Publisher target_gripper_state_publisher;
static ros::Subscriber cup_detection_subscriber;
static ros::Subscriber current_pose_subscriber;

static geometry_msgs::PoseArray cup_pose_array;
static geometry_msgs::PoseArray empty_cup_pose_array;
static geometry_msgs::PointStamped last_arm_pos;

static const float scanning_z = .20;//m
static const float scanning_r = .50;//m
static float scanning_speed = -2.5;//rad/s
static const float scanning_limit_left = 0.0;//rad
static const float scanning_limit_right = -2 * M_PI/3.0;//rad

static ros::Time last_cup_detection(0);
static ros::Duration detection_expiration(1.0);


void processCupPoses(const geometry_msgs::PoseArray& cup_positions) {
    cup_pose_array = cup_positions;
    last_cup_detection = cup_positions.header.stamp;
    std::cout << cup_pose_array << std::endl;
    ROS_ERROR("got a cup");
}

void waitForNewCups() {
    auto curr = last_cup_detection;
    while (curr == last_cup_detection) {
        ros::spinOnce();
    }
}

void processArmPose(const geometry_msgs::PoseStamped& arm_pose) {
    last_arm_pos.header = arm_pose.header;
    last_arm_pos.point = arm_pose.pose.position;
}

static double hypot(double x, double y, double z) {
    return sqrt(x*x + y*y + z*z);
}

static double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return hypot(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

static geometry_msgs::Point furthestPointFromTip() {
    auto curr_arm_pos = last_arm_pos;
    std::cerr << "last arm pos: " << curr_arm_pos << std::endl;
    auto curr_cup_poses = cup_pose_array;
    double max_distance = -1;
    geometry_msgs::Point best_pos = last_arm_pos.point;
    if (curr_cup_poses.poses.size() <= 0)
        ROS_ERROR("NO CUPS FOUND");
    for (const auto& cup : curr_cup_poses.poses) {
        std::cerr << "cup pos: " << cup.position << std::endl;
        double dist = distance(cup.position, curr_arm_pos.point);
        if (dist > max_distance) {
            max_distance = dist;
            best_pos = cup.position;
        }
    }
    return best_pos;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "commander_node");
    ros::start();
    ros::Rate loop_rate(5);

    ros::NodeHandle nh;
    std::vector<geometry_msgs::Pose> empty_poses;
    cup_pose_array.poses = empty_poses;
    // last_arm_pos = geometry_msgs::PointStamped();
    last_arm_pos.point.x = .3;
    last_arm_pos.point.y = 0;
    last_arm_pos.point.z = .1;
    // cup_pose_array = geometry_msgs::PointStamped();
    // cup_pose_array.point.x = .3;
    // cup_pose_array.point.y = 0;
    // cup_pose_array.point.z = .10;

    cup_detection_topic = "/bar_bot/cup_multidetections";

    // if (!nh.getParam("cup_detection_topic", cup_detection_topic))
    // {
    //   ROS_ERROR("cup_detection_topic param not specified");
    //   return -1;
    // }
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

    mobility_client = nh.serviceClient<bar_bot::Mobility>("mobility");

    // target_position_publisher
    //     = nh.advertise<geometry_msgs::PointStamped>(target_position_topic, 1);
    // target_gripper_state_publisher
    //     = nh.advertise<std_msgs::Bool>(target_gripper_state_topic, 1);

    cup_detection_subscriber = nh.subscribe(cup_detection_topic, 10,
        &processCupPoses);
    current_pose_subscriber = nh.subscribe(current_pose_topic, 10,
        &processArmPose);

    bar_bot::Mobility temp_mobility;
    temp_mobility.request.pour_angle         = 0;
    temp_mobility.request.is_blocking        = true;
    temp_mobility.request.use_trajectory     = true;
    temp_mobility.request.close_gripper      = false;
    temp_mobility.request.move_time          = 4; // Seconds

    temp_mobility.request.target_loc.x = 0;
    temp_mobility.request.target_loc.y = .5;
    temp_mobility.request.target_loc.z = .3;

    ros::spinOnce();
    if (mobility_client.call(temp_mobility)) {
        ROS_ERROR("call successful!");
    } else {
        ROS_ERROR("call failed!");
    }

    int count = 0;

    // Move above the cup
    bar_bot::Mobility mobility;
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        if (cup_pose_array.poses.size() > 0) {
            mobility.request.target_loc         = cup_pose_array.poses[0].position;
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 2.5; // Seconds

            mobility.request.target_loc.z += .2;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            // waitForNewCups();
            break;
        }
    }

    // Move in front of the cup
    geometry_msgs::Point saved_pos;
    while(ros::ok()){
        if (cup_pose_array.poses.size() > 0) {
            saved_pos                           = cup_pose_array.poses[0].position;
            mobility.request.target_loc         = saved_pos;
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.5; // Seconds

            mobility.request.target_loc.x *= .945;
            mobility.request.target_loc.y *= .945;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Move into the cup
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        // if (cup_pose_array.poses.size() > 0) {
            mobility.request.target_loc         = saved_pos;
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.5; // Seconds

            mobility.request.target_loc.x *= 1.1;
            mobility.request.target_loc.y *= 1.1;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            break;
        // }
    }
    // waitForNewCups();

    // Grab the cup
    // bar_bot::Mobility mobility;
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        // if (cup_pose_array.poses.size() > 0) {
            mobility.request.target_loc         = saved_pos;
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = false;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 1.5; // Seconds

            mobility.request.target_loc.x *= 1.1;
            mobility.request.target_loc.y *= 1.1;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            break;
        // }
    }

    // Pick up the cup
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        // if (cup_pose_array.poses.size() > 0) {
            mobility.request.target_loc         = saved_pos;
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 1.5; // Seconds

            mobility.request.target_loc.z = 0.3;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            break;
        // }
    }

    // Move to above other cup
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        if (cup_pose_array.poses.size() > 0) {
            auto target                         = furthestPointFromTip();
            mobility.request.target_loc         = target;
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 1.5; // Seconds

            mobility.request.target_loc.x *= 1.09;
            mobility.request.target_loc.y *= 1.09;
            mobility.request.target_loc.z = 0.35;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            break;
        }
    }

    // Pour!
    while(ros::ok()){
        if (cup_pose_array.poses.size() > 0) {
            auto target                         = furthestPointFromTip();
            mobility.request.target_loc         = target;
            mobility.request.pour_angle         = -3.14;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 1.5; // Seconds

            mobility.request.target_loc.x *= 1.090;
            mobility.request.target_loc.y *= 1.090;
            mobility.request.target_loc.z = 0.35;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    while(ros::ok()){
        ROS_ERROR("size: %d", cup_pose_array.poses.size());
        if (cup_pose_array.poses.size() > 0) {
            // saved_pos                           = cup_pose_array.poses[0].position;
            mobility.request.target_loc         = furthestPointFromTip();
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 0.5; // Seconds

            mobility.request.target_loc.x *= 1.09;
            mobility.request.target_loc.y *= 1.09;
            mobility.request.target_loc.z = 0.35;
            if (mobility_client.call(mobility)) {
                ROS_ERROR("call successful!");
            } else {
                ROS_ERROR("call failed!");
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // count = 0;
    // while(ros::ok()){
    //     auto temp = cup_pose_array;
    //     temp.point.z = .06;
    //     temp.point.x *= .9;
    //     temp.point.y *= .9;
    //     target_position_publisher.publish(temp);
    //     if (++count % 500 == 0) {
    //         break;
    //     }
    //     loop_rate.sleep();
    // }

    // count = 0;
    // while(ros::ok()){
    //     auto temp = cup_pose_array;
    //     temp.point.z = .06;
    //     temp.point.x *= 1.04;
    //     temp.point.y *= 1.04;
    //     target_position_publisher.publish(temp);
    //     if (++count % 500 == 0) {
    //         target_gripper_state_publisher.publish(false);
    //         break;
    //     }
    //     loop_rate.sleep();
    // }

    // count = 0;
    // while(ros::ok()){
    //     auto temp = cup_pose_array;
    //     temp.point.z = .4;
    //     target_position_publisher.publish(temp);
    //     if (++count % 500 == 0) {
    //         target_gripper_state_publisher.publish(false);
    //     }
    //     loop_rate.sleep();
    // }

    ros::shutdown();

    return 0;
}


        //scan for cup

        // double goal_theta = atan2(last_arm_pos.point.y , last_arm_pos.point.x);
        // if ((ros::Time::now() - last_cup_detection) <= detection_expiration) {
        //     if (count++ % 100 == 0) {
        //         auto temp = cup_pose_array;
        //         temp.point.z += .4; /* Go to above the cup */
        //         target_position_publisher.publish(temp);
        //     }
        // }
        // if (count > 2000)
        //     break;
        // loop_rate.sleep();
        // ros::spinOnce();



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
        // target_position_publisher.publish(cup_pose_array);
        // loop_rate.sleep();
        // ros::spinOnce();
