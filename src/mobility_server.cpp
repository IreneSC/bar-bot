#include "mobility_server.hpp"

// Ros subscriber/publisher info
static std::string target_subscriber_name;
static const std::string joint_state_name("/joint_state");
static const std::string joint_state_feedback_name("/hebiros/all/feedback/joint_state");
static std::string current_pose_topic_name;
static ros::Publisher joint_state_publisher;
static ros::Publisher fkin_pub;

// HEBI info
static const std::vector<std::string> families = {"Arm", "Arm", "Arm", "Arm", "Arm", "Arm"};
static const std::vector<std::string> names  = {"base", "pitch_1", "pitch_2", "pitch_3", "wrist", "gripper"};
static HebiHelper* helper_p;
static geometry_msgs::PoseStamped feedback_pose;

// Stuff for handling trajectories

// Time
static ros::Time prev_time;
static double t_f;

// Positions (angular)
static double  q[num_joints];
static double  qdot[num_joints];

// Cubic spline parameters
static double  a[num_joints], b[num_joints], c[num_joints], d[num_joints];

// Max speeds.
static double  qdotmax[num_joints] = {.1, .1, .1, .1, .1};

static double default_pos[num_joints] = {0, 0.785, -1.57, -0.785, 0};

// Function declarations
static double hypot(double x, double y, double z);
static double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
static bool areWeThereYet(const geometry_msgs::PointStamped& target_loc, double dist);
static void initTrajectory(const sensor_msgs::JointState& target_joints, const bool use_cubic_spline);
static void followTrajectory();
// static void processTargetState(const geometry_msgs::PointStamped& target_loc);
static void processFeedback(const sensor_msgs::JointState& joints);
static bool block(const geometry_msgs::Point& target_loc, double timeout_secs);

// Simple helpers

static double hypot(double x, double y, double z) {
    return sqrt(x*x + y*y + z*z);
}

static double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return hypot(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

static bool areWeThereYet(const geometry_msgs::Point& target_loc, double dist) {
    return distance(target_loc, feedback_pose.pose.position) < dist;
}

// More complicated helpers

// Calculate the cubic spline parameters. If use_cubic_spline is false, then
// go immediately to end state by setting time-to-move to 0
static void initTrajectory(const sensor_msgs::JointState& target_joints,
                           const bool use_cubic_spline,
                           const double min_time) {
    prev_time = ros::Time::now();
    std::vector<double> qfinal = target_joints.position;
    int     i;
    double  tmove;        // Total move time
    double  tmp;

    // Pick a move time.  Note this is approximate.  We could compute
    // the absolute fastest time or pass as an argument.
    tmove = min_time;
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
    if (use_cubic_spline)
        t_f = tmove;
    else
        t_f = 0; // No time to move - just go right there.
}

// Handle feedback from hebi
static void processFeedback(const sensor_msgs::JointState& joints) {
    geometry_msgs::Point pt = jointAnglesToPosition(joints);
    feedback_pose.pose.position = pt;
    feedback_pose.header.stamp = ros::Time::now();
    fkin_pub.publish(feedback_pose);
}

// Block until we reach the target location target_loc
// Return true if the target was reached
static bool block(const geometry_msgs::Point& target_loc, double timeout_secs) {
    static constexpr double max_dist = 0.08; // meters
    auto cur_time = ros::Time::now();
    while (!areWeThereYet(target_loc, max_dist) && (ros::Time::now() - cur_time).toSec() < timeout_secs) {
        usleep(100);
        followTrajectory();
    }
    return areWeThereYet(target_loc, max_dist);
}

static void followTrajectory() {
    sensor_msgs::JointState cmdMsg;
    cmdMsg.position.resize(num_joints);
    cmdMsg.velocity.resize(num_joints);

    // Advance time, but hold at t=0 to stay at the final position.
    double t = (ros::Time::now() - prev_time).toSec();
    if (t > 0.0)
        t = 0.0;

    // Compute the new position and velocity commands.
    for (int i = 0 ; i < num_joints ; i++)
    {
        q[i]    = a[i]+t*(b[i]+t*(c[i]+t*d[i]));
        qdot[i] = b[i]+t*(2.0*c[i]+t*3.0*d[i]);

        cmdMsg.position[i] = q[i];
        cmdMsg.velocity[i] = qdot[i];
    }

    // Publish.
    cmdMsg.header.stamp = ros::Time::now();
    // cmdPub.publish(cmdMsg);
    helper_p->goToJointState(cmdMsg);
}

// Processes a request to move to some pose
bool MobilitySrv::processRequest(Mobility::Request& req, Mobility::Response& res) {
    // Parse the message into a joint state
    sensor_msgs::JointState joint_state = positionToJointAngles(req.target_loc);
    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_names;
    res.target_reached = true;

    helper_p->setGripperClosed(req.close_gripper);
    helper_p->setPourAngle(req.pour_angle);

    // If use trajectory is false, it just sets the "time to move" to 0 seconds
    initTrajectory(joint_state, req.use_trajectory, req.move_time);

    if (req.is_blocking) {
        res.target_reached = block(req.target_loc, req.move_time * 1.5);
    }

    joint_state_publisher.publish(joint_state);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mobility_server");
    ros::start();

    // Initialize the global variables before setting up the services.
    prev_time = ros::Time::now();
    for (int i = 0 ; i < 5 ; i++)
    {
        q[i]    = 0.0;
        qdot[i] = 0.0;

        a[i] = b[i] = c[i] = d[i] = 0.0;
    }

    // Set up pub/sub

    ros::NodeHandle n;

    // Set up service
    MobilitySrv mobility;
    ros::ServiceServer ss = n.advertiseService("mobility", &MobilitySrv::processRequest, &mobility);

    joint_state_publisher =
        n.advertise<sensor_msgs::JointState>(joint_state_name, 1);

    // Get topic names

    // if (!n.getParam("target_position_topic", target_subscriber_name))
    // {
    //   ROS_ERROR("target_gripper_state_topic param not specified");
    //   return -1;
    // }
    if (!n.getParam("current_pose_topic", current_pose_topic_name))
    {
      ROS_ERROR("target_gripper_state_topic param not specified");
      return -1;
    }

    // ros::Subscriber target_subscriber =
    //     n.subscribe(target_subscriber_name, 10,
    //                           &processTargetState);

    ros::Subscriber feedback_subscriber =
        n.subscribe(joint_state_feedback_name, 10,
                              &processFeedback);

    fkin_pub = n.advertise<geometry_msgs::PoseStamped>(
            current_pose_topic_name, 10, &processFeedback);

    // Set up HEBI 
    HebiHelper helper(n, "all", names, families);
    helper_p = &helper;

    ROS_INFO("About to loop");

    // TODO: add back in loop rate?
    while(ros::ok()) {
        followTrajectory();
        ros::spinOnce();
    }
    ros::shutdown();

    return 0;
}

// UNUSED CODE

// static void processTargetState(const geometry_msgs::Point& target_loc) {
//     // const geometry_msgs::Point&      target_loc = target_pose.pose.position;
//     // const geometry_msgs::Quaternion& target_ori = target_pose.pose.orientation;
//     sensor_msgs::JointState angles = positionToJointAngles(target_loc.point);
//     initTrajectory(angles);

//     angles.header.stamp = ros::Time::now();
//     angles.name = joint_names;

//     // // Set the roll of the gripper
//     // tf::Quaternion q;
//     // tf::quaternionMsgToTF(target_ori, q);
//     // tf::Matrix3x3 m(q);
//     // // Roll pitch yaw
//     // double r, p, y;
//     // m.getRPY(r, p, y);
//     // // std::cout << "sizes: " << num_joints - 1 << ", " << angles.position.size() << std::endl;
//     // // angles.position[num_joints-1] = r;

//     joint_state_publisher.publish(angles);
//     // helper_p->goToJointState(angles);
//     std::cout << "Published: " << angles << std::endl;
// }
//
//
// From while loop
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


// #else
//     while(ros::ok()) {
//         sensor_msgs::JointState jointState;
//         std::vector<double> angles = {0, 0, .3, 0, 0};
//         jointState.position = angles;
//         helper_p->goToJointState(jointState);
//         ros::spinOnce();
//     }
//     // geometry_msgs::Point positions;
//     // positions.x = .45;
//     // positions.y = 0;
//     // positions.z = 0.0861;
//     // auto ret = jointAnglesToPosition(positionToJointAngles(positions));
//     // std::cout << ret << std::endl;
//     // std::cout << jointAnglesToPosition(positionToJointAngles(ret)) << std::endl;
//     // std::cout << "Done printing" << std::endl;
// #endif


