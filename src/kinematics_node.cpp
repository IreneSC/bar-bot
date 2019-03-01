#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "kinematics.hpp"
#include "tf/transform_datatypes.h"
#include "HebiHelper.hpp"

static std::string target_subscriber_name;
static const std::string joint_state_name("/joint_state");
static const std::string joint_state_feedback_name("/hebiros/all/feedback/joint_state");
static std::string current_pose_topic_name;

static const std::vector<std::string> families = {"Arm", "Arm", "Arm", "Arm", "Arm", "Arm"};
static const std::vector<std::string> names  = {"base", "pitch_1", "pitch_2", "pitch_3", "wrist", "gripper"};
static HebiHelper* helper_p;

static ros::Time prev_time;
static double t_f;

static double  q[num_joints];
static double  qdot[num_joints];

// Cubic spline parameters
static double  a[num_joints], b[num_joints], c[num_joints], d[num_joints];

// Max speeds.
static double  qdotmax[num_joints] = {.8, .8, .8, 1, .8};

static double default_pos[num_joints] = {0, 0.785, -1.57, -0.785, 0};

ros::Publisher joint_state_publisher;
ros::Publisher fkin_pub;

void initTrajectory(const sensor_msgs::JointState& target_joints) {
    prev_time = ros::Time::now();
    std::vector<double> qfinal = target_joints.position;
    int     i;
    double  tmove;        // Total move time
    double  tmp;

    // Pick a move time.  Note this is approximate.  We could compute
    // the absolute fastest time or pass as an argument.
    tmove = .25;
    for (i = 0 ; i < num_joints ; i++)
    {
        tmp = 2.0 * fabs(qfinal[i] - q[i]) / qdotmax[i];
        if (tmp > tmove)
            tmove = tmp;
    }

    // Set the cubic spline parameters.
    for (i = 0 ; i < num_joints ; i++)
    {
        a[i] = qfinal[i];
        b[i] = 0.0;
        c[i] = (3.0*(q[i]-qfinal[i])/tmove + qdot[i])/tmove;
        d[i] = (2.0*(q[i]-qfinal[i])/tmove + qdot[i])/tmove/tmove;
    }

    // Set the time, so the move starts t=0 and ends t=t_f.
    t_f = tmove;
}

void processTargetState(const geometry_msgs::PointStamped& target_loc) {
    // const geometry_msgs::Point&      target_loc = target_pose.pose.position;
    // const geometry_msgs::Quaternion& target_ori = target_pose.pose.orientation;
    sensor_msgs::JointState angles = positionToJointAngles(target_loc.point);
    initTrajectory(angles);

    angles.header.stamp = ros::Time::now();
    angles.name = joint_names;

    // // Set the roll of the gripper
    // tf::Quaternion q;
    // tf::quaternionMsgToTF(target_ori, q);
    // tf::Matrix3x3 m(q);
    // // Roll pitch yaw
    // double r, p, y;
    // m.getRPY(r, p, y);
    // // std::cout << "sizes: " << num_joints - 1 << ", " << angles.position.size() << std::endl;
    // // angles.position[num_joints-1] = r;

    joint_state_publisher.publish(angles);
    // helper_p->goToJointState(angles);
    std::cout << "Published: " << angles << std::endl;
}

void processFeedback(const sensor_msgs::JointState& joints) {
    geometry_msgs::Point pt = jointAnglesToPosition(joints);
    geometry_msgs::PoseStamped pose;
    pose.pose.position = pt;
    pose.header.stamp = ros::Time::now();
    fkin_pub.publish(pose);
}

void followTrajectory() {
    sensor_msgs::JointState cmdMsg;
    cmdMsg.position.resize(num_joints);
<<<<<<< HEAD

    // cmdMsg.velocity.resize(num_joints);

    // Advance time, but hold at t=0 to stay at the final position.
    double t = (ros::Time::now() - prev_time).toSec();
    //if (t > 0.0)
    //    t = 0.0;

    // Calculates desired triangle position
    int servo_test_ind = 0;
    double triang_amp = 0.785; // Sweep between +- 45 deg
    double triang_period = 5; // s
    double triang_vel = triang_amp/triang_period;  // rad/s
    double triang_pos = t * triang_vel; // rad
    // Switch direction at appropriate time
    if (t >= triang_period) {
        prev_time = ros::Time::now();
        triang_amp *= -1;
    }

//     cmdMsg.velocity.resize(num_joints);

//     // Advance time, but hold at t=0 to stay at the final position.
//     double t = (ros::Time::now() - prev_time).toSec() - t_f;
//     if (t > 0.0)
//         t = 0.0;

    // Compute the new position and velocity commands.
    for (int i = 0 ; i < num_joints ; i++)
    {
        if (i = 0) {
            q[i] = triang_pos;
            qdot[i] = triang_vel;
        } else {
            q[i] = default_pos[i];
            qdot[i] = 0;
        }

        //q[i]    = a[i]+t*(b[i]+t*(c[i]+t*d[i]));
        //qdot[i] = b[i]+t*(2.0*c[i]+t*3.0*d[i]);

        cmdMsg.position[i] = q[i];
        cmdMsg.velocity[i] = qdot[i];
    }

    // Publish.
    cmdMsg.header.stamp = ros::Time::now();
    // cmdPub.publish(cmdMsg);
    helper_p->goToJointState(cmdMsg);

    // Wait for next cycle.
    // ros::spinOnce();
    // loop_rate.sleep();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "kinematics_node");
    ros::start();

    // Initialize the global variables before setting up the services.
    prev_time = ros::Time::now();
    for (int i = 0 ; i < 5 ; i++)
    {
        q[i]    = 0.0;
        qdot[i] = 0.0;

        a[i] = b[i] = c[i] = d[i] = 0.0;
    }


    ros::NodeHandle node_handler;

    joint_state_publisher =
        node_handler.advertise<sensor_msgs::JointState>(joint_state_name, 1);

    if (!node_handler.getParam("target_position_topic", target_subscriber_name))
    {
      ROS_ERROR("target_gripper_state_topic param not specified");
      return -1;
    }


    if (!node_handler.getParam("current_pose_topic", current_pose_topic_name))
    {
      ROS_ERROR("target_gripper_state_topic param not specified");
      return -1;
    }

    ros::Subscriber target_subscriber =
        node_handler.subscribe(target_subscriber_name, 10,
                              &processTargetState);

    ros::Subscriber feedback_subscriber =
        node_handler.subscribe(joint_state_feedback_name, 10,
                              &processFeedback);

    fkin_pub = node_handler.advertise<geometry_msgs::PoseStamped>(
            current_pose_topic_name, 10, &processFeedback);

    HebiHelper helper(node_handler, "all", names, families);
    helper_p = &helper;

    ROS_INFO("About to loop");

#define TEST_ANGLE

    /* Test: */
#ifdef TEST_ANGLE
    // std::vector<double> angles = {0, 0, -M_PI/2, 0};
    while(ros::ok()) {
        // if (!helper.isValid()) {
        //     ROS_INFO("waiting for valid feedback");
        //     ros::spinOnce();
        //     continue;
        // }
        // std::vector<double> angles = {0, 0, 0, 0};
        // std::vector<double> angles = {0.1, 0.2, -0.4, -.2+.4};

        // sensor_msgs::JointState joints = helper.getFeedback();
        // joints.header.stamp = ros::Time::now();
        // joints.name = joint_names;
        // joints.position.pop_back();
        // joint_state_publisher.publish(joints);
        // joints.position = angles;

        // geometry_msgs::Point positions;
        // positions.x = .23;
        // positions.y = .40;
        // // positions.z = 0.11;
        // positions.z = 0.11;


        // // auto ret = positionToJointAngles(jointAnglesToPosition(joints));
        // // auto ret = jointAnglesToPosition(joints);
        // // // std::cout << ret << std::endl;
        // geometry_msgs::PoseStamped target_pose;
        // target_pose.pose.position = positions;

        // tf::Quaternion q;
        // q.setRPY(0,0,0);
        // // q.setRPY(joints.position[num_joints-1],0,0);
        // tf::quaternionTFToMsg(q, target_pose.pose.orientation);

        // processTargetState(target_pose);
        // std::cout << "after ret" <<std::endl;
        followTrajectory();
        ros::spinOnce();
    }
    // std::cout << positionToJointAngles(jointAnglesToPosition(ret)) << std::endl;
#else
    while(ros::ok()) {
        sensor_msgs::JointState jointState;
        std::vector<double> angles = {0, 0, .3, 0, 0};
        jointState.position = angles;
        helper_p->goToJointState(jointState);
        ros::spinOnce();
    }
    // geometry_msgs::Point positions;
    // positions.x = .45;
    // positions.y = 0;
    // positions.z = 0.0861;
    // auto ret = jointAnglesToPosition(positionToJointAngles(positions));
    // std::cout << ret << std::endl;
    // std::cout << jointAnglesToPosition(positionToJointAngles(ret)) << std::endl;
    // std::cout << "Done printing" << std::endl;
#endif


    ros::shutdown();

    return 0;
}
