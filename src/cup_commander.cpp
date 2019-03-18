#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "bar_bot/Mobility.h"
#include "bar_bot/Detections.h"
#include "std_msgs/String.h"
#include <unordered_map>
#include <vector>

// Topics
static std::string detection_topic;
static std::string target_position_topic;
static std::string target_gripper_state_topic;
static std::string current_pose_topic;
static std::string drink_type_topic;

// Pub/sub/clients
static ros::ServiceClient mobility_client;
static ros::Subscriber detection_subscriber;
static ros::Subscriber current_pose_subscriber;
static ros::Subscriber drink_subscriber;


// Things gotten from topics
// static geometry_msgs::PoseArray cup_pose_array;
// static geometry_msgs::PoseArray empty_cup_pose_array;
static geometry_msgs::PointStamped last_arm_pos;
// static ros::Time last_cup_detection(0);
// static ros::Duration detection_expiration(1.0);

// Set up map of detections
static const std::string CUP("cup");
static const std::string COKE("coke");
static const std::string SPRITE("sprite");
static const std::string VODKA("vodka?");
static const std::string RUM("rum");
static const std::string OJ("orange juice");
static std::unordered_map<std::string, geometry_msgs::Point> det_positions;
// mixed drink definitions
static const std::vector<std::string> MARGARITA{SPRITE, COKE};
static const std::vector<std::string> SCREWDRIVER{RUM, VODKA};
static std::unordered_map<std::string, std::vector<std::string>> det_ingredients {
    {"margarita", MARGARITA},
    {"screwdriver", SCREWDRIVER}
};

// Function definitions
static bool moveWithFailureChecking(bar_bot::Mobility mobility);

// Other constants
static geometry_msgs::Point end_point;

static void backAndForth();
static void trackCups();
static void pourBeer();
static void replaceDrink(std::string drink, geometry_msgs::Point end_loc);
static void pourDrinkIntoCup(std::string drink);
static void pourIntoTarget(std::string drink);
static geometry_msgs::Point retrieveDrink(std::string drink);
static std::vector<std::string> drinkQueue;

// Constants for looking for a cup
// static const float scanning_z = .20;//m
// static const float scanning_r = .50;//m
// static float scanning_speed = -2.5;//rad/s
// static const float scanning_limit_left = 0.0;//rad
// static const float scanning_limit_right = -2 * M_PI/3.0;//rad

// void processCupPoses(const geometry_msgs::PoseArray& cup_positions) {
//     cup_pose_array = cup_positions;
//     last_cup_detection = cup_positions.header.stamp;
//     // std::cout << cup_pose_array << std::endl;
//     // ROS_ERROR("got a cup");
// }

void processDetections(const bar_bot::Detections& dets) {
    for (int i = 0; i < (int) dets.detection_types.size(); i++) {
        const std::string& type = dets.detection_types[i];
        det_positions[type] = dets.detection_positions[i];
        ROS_INFO_STREAM(type << " loc: " << det_positions[type]);
    }
}

// void waitForNewCups() {
//     auto curr = last_cup_detection;
//     while (curr == last_cup_detection) {
//         ros::spinOnce();
//     }
// }

void processArmPose(const geometry_msgs::PoseStamped& arm_pose) {
    last_arm_pos.header = arm_pose.header;
    last_arm_pos.point = arm_pose.pose.position;
}

void processDrinkRequest(const std_msgs::String& drinkType) {
    std::vector<std::string> mixedDrink = det_ingredients[drinkType.data];
    if(drinkQueue.empty())
        std::copy(mixedDrink.begin(), mixedDrink.end(), drinkQueue.end());
}

static double hypot(double x, double y, double z) {
    return sqrt(x*x + y*y + z*z);
}

static double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return hypot(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

// static geometry_msgs::Point furthestPointFromTip() {
//     auto curr_arm_pos = last_arm_pos;
//     // std::cerr << "last arm pos: " << curr_arm_pos << std::endl;
//     auto curr_cup_poses = cup_pose_array;
//     double max_distance = -1;
//     geometry_msgs::Point best_pos = last_arm_pos.point;
//     if (curr_cup_poses.poses.size() <= 0)
//         ROS_ERROR("NO CUPS FOUND");
//     for (const auto& cup : curr_cup_poses.poses) {
//         // std::cerr << "cup pos: " << cup.position << std::endl;
//         double dist = distance(cup.position, curr_arm_pos.point);
//         if (dist > max_distance) {
//             max_distance = dist;
//             best_pos = cup.position;
//         }
//     }
//     return best_pos;
// }

// Chooses a drink from the queue
std::string popDrink() {
    std::string drink = drinkQueue.front();
    drinkQueue.erase(drinkQueue.begin());
    return drink;
}

// Returns where it picked it up
geometry_msgs::Point retrieveDrink(std::string drink) {
    // Move above the cup
    bar_bot::Mobility mobility;
    mobility.request.pour_angle         = 0;
    mobility.request.is_blocking        = true;
    mobility.request.use_trajectory     = true;
    mobility.request.disable_collisions = false;
    static constexpr double height      = .15;

    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(drink)>0) {
            mobility.request.target_loc         = det_positions[drink];
            ROS_INFO_STREAM("coke loc: " << det_positions[drink]);
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 2.5; // Seconds

            mobility.request.target_loc.x *= .945;
            mobility.request.target_loc.y *= .945;
            mobility.request.target_loc.z += .3;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Move in front of the cup
    geometry_msgs::Point saved_pos;
    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(drink)>0) {
            saved_pos                           = det_positions[drink];
            mobility.request.target_loc         = saved_pos;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.25; // Seconds

            mobility.request.target_loc.x *= .905;
            mobility.request.target_loc.y *= .905;
            auto x = mobility.request.target_loc.x;
            auto y = mobility.request.target_loc.y;
            mobility.request.target_loc.z = height * sqrt(x*x + y*y) / .6;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Move into the cup
    while(ros::ok()){
        saved_pos                           = det_positions[drink];
        mobility.request.target_loc         = saved_pos;
        mobility.request.close_gripper      = false;
        mobility.request.move_time          = 1.2; // Seconds
        mobility.request.disable_collisions = true;

        mobility.request.target_loc.x *= 1.05;
        mobility.request.target_loc.y *= 1.05;
        auto x = mobility.request.target_loc.x;
        auto y = mobility.request.target_loc.y;
        mobility.request.target_loc.z = height * sqrt(x*x + y*y) / .6;
        if(moveWithFailureChecking(mobility)) {
            break;
        }
    }

    // Grab the cup
    // bar_bot::Mobility mobility;
    while(ros::ok()){
        ros::spinOnce();
        mobility.request.target_loc         = saved_pos;
        mobility.request.use_trajectory     = false;
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 1.2; // Seconds
        mobility.request.disable_collisions = true;

        mobility.request.target_loc.x *= 1.05;
        mobility.request.target_loc.y *= 1.05;
        auto x = mobility.request.target_loc.x;
        auto y = mobility.request.target_loc.y;
        mobility.request.target_loc.z = height * sqrt(x*x + y*y) / .6;
        if(moveWithFailureChecking(mobility)) {
            break;
        }
    }

    mobility.request.use_trajectory     = true;

    // Pick up the cup
    while(ros::ok()){
        ros::spinOnce();
        mobility.request.target_loc         = saved_pos;
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 1.25; // Seconds
        mobility.request.disable_collisions = false;

        mobility.request.target_loc.z = 0.3;
        if(moveWithFailureChecking(mobility)) {
            break;
        }
    }
    return saved_pos;
}

void pourIntoTarget(std::string drink) {
    bar_bot::Mobility mobility;
    mobility.request.disable_collisions = false;
    mobility.request.pour_angle         = 0;
    mobility.request.is_blocking        = true;
    mobility.request.use_trajectory     = true;

    const double height = .35;

    // Move to above other cup
    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(drink)>0) {
            auto target                         = det_positions[drink]; //furthestPointFromTip();
            mobility.request.target_loc         = target;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 2.5; // Seconds

            mobility.request.target_loc.z = height;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Wait until we get a new cup detection
    ros::Duration(1).sleep();

    // Recenter above the cup, and save that as the target to pour over 
    geometry_msgs::Point target;
    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(drink)>0) {
            target                              = det_positions[drink]; //furthestPointFromTip();
            mobility.request.target_loc         = target;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = .75; // Seconds

            mobility.request.target_loc.z = height;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Pour!
    while(ros::ok()){
        if (det_positions.count(drink)>0) {
            mobility.request.target_loc         = target;
            mobility.request.pour_angle         = M_PI*3.0/4;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 4.25; // Seconds
            mobility.request.pouring_beer       = true;
            mobility.request.beer_nh            = 0.05;
            mobility.request.beer_gh            = 0.09;

            mobility.request.target_loc.z = height;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
        ros::spinOnce();
    }
}

void replaceDrink(std::string drink, geometry_msgs::Point end_loc) {
    bar_bot::Mobility mobility;
    mobility.request.disable_collisions = false;
    mobility.request.pour_angle         = 0;
    mobility.request.is_blocking        = true;
    mobility.request.use_trajectory     = true;
    mobility.request.pouring_beer       = false;
    mobility.request.beer_nh            = 0;
    mobility.request.beer_gh            = 0;

    // Put it back down.
    mobility.request.target_loc         = end_loc; // furthestPointFromTip();
    mobility.request.close_gripper      = true;
    mobility.request.move_time          = 2.5; // Seconds

    mobility.request.target_loc.z = 0.275;
    moveWithFailureChecking(mobility);

    // All the way to the ground
    mobility.request.target_loc         = end_loc; // furthestPointFromTip();
    mobility.request.close_gripper      = true;
    mobility.request.move_time          = 1.35; // Seconds
    mobility.request.disable_collisions = true;

    // mobility.request.target_loc.x *= .925;
    // mobility.request.target_loc.y *= .925;
    mobility.request.target_loc.x *= 1.02;
    mobility.request.target_loc.y *= 1.02;
    mobility.request.target_loc.z = 0.2;
    moveWithFailureChecking(mobility);

    ros::Duration(1).sleep(); // Make sure you're really on the ground

    // Release! I said, release boy!
    mobility.request.target_loc         = end_loc; // furthestPointFromTip();
    mobility.request.close_gripper      = false;
    mobility.request.move_time          = 1.5; // Seconds
    mobility.request.disable_collisions = true;

    mobility.request.target_loc.x *= 1.02;
    mobility.request.target_loc.y *= 1.02;
    mobility.request.target_loc.z = 0.2;
    moveWithFailureChecking(mobility);

    ros::Duration(6).sleep();

    mobility.request.target_loc         = end_loc; // furthestPointFromTip();
    mobility.request.close_gripper      = false;
    mobility.request.move_time          = 3; // Seconds
    mobility.request.disable_collisions = false;

    // Back it up
    mobility.request.target_loc.x *= .8;
    mobility.request.target_loc.y *= .8;
    mobility.request.target_loc.z = 0.45;
    moveWithFailureChecking(mobility);
}

void pourDrinkIntoCup(std::string drink) {
    auto loc = retrieveDrink(drink);
    pourIntoTarget(CUP);
    replaceDrink(drink, loc);
}

// Sequence to pour a mixed drink
void pourMixedDrink()  {
    // stall until drink request arrives
    while (drinkQueue.size() == 0) {
        ROS_INFO("Waiting for drink request");
    } 

    std::string drink;
    while (drinkQueue.size() != 0) {
        drink = popDrink();
        pourDrinkIntoCup(drink);
    }
}

static bool moveWithFailureChecking(bar_bot::Mobility mobility) {
    mobility.response.target_reached = false;
    mobility_client.call(mobility);
    if (mobility.response.target_reached) {
        ROS_INFO("call successful!");
        return true;
    } else {
        ROS_INFO("call failed!");
        ros::Duration(4).sleep();
        return false;
    }
}

static void goHome() {
    bar_bot::Mobility temp_mobility;
    temp_mobility.request.disable_collisions = false;
    temp_mobility.request.pour_angle         = 0;
    temp_mobility.request.is_blocking        = true;
    temp_mobility.request.use_trajectory     = true;
    temp_mobility.request.close_gripper      = false;
    temp_mobility.request.move_time          = 4; // Seconds

    temp_mobility.request.target_loc.x = 0;
    temp_mobility.request.target_loc.y = .5;
    temp_mobility.request.target_loc.z = .35;

    ros::spinOnce();
    while (true) {
        temp_mobility.response.target_reached = false;
        mobility_client.call(temp_mobility); 
        if(temp_mobility.response.target_reached)
            break;
        ros::Duration(2).sleep();
        ROS_INFO("call failed!");
    }
    ROS_INFO("made it home!");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "commander_node");
    ros::start();
    ros::Rate loop_rate(5);

    ros::NodeHandle nh;
    last_arm_pos.point.x = .3;
    last_arm_pos.point.y = 0;
    last_arm_pos.point.z = .1;

    end_point.x = -.06;
    end_point.y = .74;
    end_point.z = 0.05;

    detection_topic = "/bar_bot/detections";

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

    detection_subscriber = nh.subscribe(detection_topic, 10,
        &processDetections);
    current_pose_subscriber = nh.subscribe(current_pose_topic, 10,
        &processArmPose);
    drink_subscriber = nh.subscribe(drink_type_topic, 1,
        &processDrinkRequest);

    goHome();

    // pourCups();
    // backAndForth();
    // trackCups();
    // pourBeer();
    pourDrinkIntoCup(SPRITE);
    // trackCups();

    goHome();

    while(ros::ok()) {
        ros::spinOnce();
    }

    ros::shutdown();

    return 0;
}

void trackCups() {
    ros::Rate loop_rate(2);
    bar_bot::Mobility mobility;
    while(ros::ok()){
        if (det_positions.count(CUP)>0) {
            // saved_pos                           = det_positions[CUP];
            mobility.request.target_loc         = det_positions[CUP];
            mobility.request.pour_angle         = 0;
            mobility.request.is_blocking        = true;
            mobility.request.use_trajectory     = true;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.5; // Seconds
            mobility.request.pouring_beer       = false;

            mobility.request.target_loc.z = 0.45;

            if(moveWithFailureChecking(mobility)) {
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void backAndForth() {
    ros::Rate loop_rate(5);
    // Move back and forth
    bar_bot::Mobility mobility;
    mobility.request.pour_angle         = 0;
    mobility.request.is_blocking        = true;
    mobility.request.use_trajectory     = true;
    mobility.request.close_gripper      = false;
    mobility.request.move_time          = 1.5; // Seconds

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        mobility.request.target_loc.x =  0;
        mobility.request.target_loc.y = .5;
        mobility.request.target_loc.z = .3;

        if(moveWithFailureChecking(mobility)) {
            }

        mobility.request.target_loc.x = .3;
        mobility.request.target_loc.y = .3;
        mobility.request.target_loc.z = .15;

        if(moveWithFailureChecking(mobility)) {
            }
    }
}

void pourBeer() {
    ros::Rate loop_rate(1);
    bar_bot::Mobility mobility;

    // Pour!
    while(ros::ok()){
        mobility.request.pour_angle         = M_PI*3.0/4;
        mobility.request.is_blocking        = true;
        mobility.request.use_trajectory     = true;
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 5; // Seconds
        mobility.request.pouring_beer       = true;
        mobility.request.beer_nh            = 0.15;
        mobility.request.beer_gh            = 0.05;

        mobility.request.target_loc.x = 0;
        mobility.request.target_loc.y = 0.5;
        mobility.request.target_loc.z = 0.4;

        if(moveWithFailureChecking(mobility)) {
            // Do nothing
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

static void pourCups() {
    // Move above the cup
    bar_bot::Mobility mobility;
    mobility.request.pour_angle         = 0;
    mobility.request.is_blocking        = true;
    mobility.request.use_trajectory     = true;
    static constexpr double z_offset    = .15;

    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(CUP)>0) {
            mobility.request.target_loc         = det_positions[CUP];
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.2; // Seconds

            mobility.request.target_loc.x *= .985;
            mobility.request.target_loc.y *= .985;
            mobility.request.target_loc.z += .3;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Move in front of the cup
    geometry_msgs::Point saved_pos;
    while(ros::ok()){
        if (det_positions.count(CUP)>0) {
            saved_pos                           = det_positions[CUP];
            mobility.request.target_loc         = saved_pos;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.75; // Seconds

            mobility.request.target_loc.x *= .945;
            mobility.request.target_loc.y *= .945;
            mobility.request.target_loc.z += z_offset;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
        ros::spinOnce();
    }

    // Move into the cup
    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(CUP)>0) {
            saved_pos                           = det_positions[CUP];
            mobility.request.target_loc         = saved_pos;
            mobility.request.close_gripper      = false;
            mobility.request.move_time          = 1.75; // Seconds

            mobility.request.target_loc.x *= 1.05;
            mobility.request.target_loc.y *= 1.05;
            mobility.request.target_loc.z += z_offset;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Grab the cup
    // bar_bot::Mobility mobility;
    while(ros::ok()){
        ros::spinOnce();
        mobility.request.target_loc         = saved_pos;
        mobility.request.use_trajectory     = false;
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 1; // Seconds

        mobility.request.target_loc.x *= 1.05;
        mobility.request.target_loc.y *= 1.05;
        mobility.request.target_loc.z += z_offset;
        if(moveWithFailureChecking(mobility)) {
            break;
        }
    }

    mobility.request.use_trajectory     = true;

    // Pick up the cup
    while(ros::ok()){
        ros::spinOnce();
        mobility.request.target_loc         = saved_pos;
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 1.25; // Seconds

        mobility.request.target_loc.z = 0.3;
        if(moveWithFailureChecking(mobility)) {
            break;
        }
    }

    // Move to above other cup
    while(ros::ok()){
        ros::spinOnce();
        if (det_positions.count(CUP)>0) {
            auto target                         = det_positions[CUP]; //furthestPointFromTip();
            mobility.request.target_loc         = target;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 2.25; // Seconds

            mobility.request.target_loc.z = 0.335;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
    }

    // Pour!
    while(ros::ok()){
        if (det_positions.count(CUP)>0) {
            auto target                         = det_positions[CUP]; //furthestPointFromTip();
            mobility.request.target_loc         = target;
            mobility.request.pour_angle         = M_PI*3.0/4;
            mobility.request.close_gripper      = true;
            mobility.request.move_time          = 4.25; // Seconds
            mobility.request.pouring_beer       = true;
            mobility.request.beer_nh            = 0.09;
            mobility.request.beer_gh            = 0.05;

            mobility.request.target_loc.z = 0.335;
            if(moveWithFailureChecking(mobility)) {
                break;
            }
        }
        // ros::spinOnce();
        // loop_rate.sleep();
    }

    // Reset to default values
    mobility.request.pour_angle         = 0;
    mobility.request.pouring_beer       = false;
    mobility.request.beer_nh            = 0;
    mobility.request.beer_gh            = 0;

    // Put it back down.
    while(ros::ok()){
        // ros::spinOnce();
        mobility.request.target_loc         = end_point; // furthestPointFromTip();
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 1; // Seconds

        mobility.request.target_loc.z = 0.275;
        if(moveWithFailureChecking(mobility)) {
                break;
            }
    }

    // All the way to the ground
    while(ros::ok()){
        // ros::spinOnce();
        mobility.request.target_loc         = end_point; // furthestPointFromTip();
        mobility.request.close_gripper      = true;
        mobility.request.move_time          = 1.35; // Seconds

        mobility.request.target_loc.z = 0.05;
        if(moveWithFailureChecking(mobility)) {
                break;
            }
    }

    ros::Duration(1).sleep();

    // Release! I said, release boy!
    while(ros::ok()){
        // ros::spinOnce();
        mobility.request.target_loc         = end_point; // furthestPointFromTip();
        mobility.request.close_gripper      = false;
        mobility.request.move_time          = 5; // Seconds

        mobility.request.target_loc.z = 0.05;
        if(moveWithFailureChecking(mobility)) {
            }
    }

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
