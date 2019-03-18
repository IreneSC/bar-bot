#include "mobility_server.hpp"

// Ros subscriber/publisher info
static std::string target_subscriber_name;
static const std::string joint_state_name("/joint_state");
static const std::string joint_state_feedback_name("/hebiros/all/feedback/joint_state");
static std::string current_pose_topic_name;
static ros::Publisher joint_state_publisher;
static ros::Publisher corrected_feedback_publisher;
static ros::Publisher fkin_pub;
static volatile bool service_ready;
static volatile double pour_angle;

// HEBI info
static const std::vector<std::string> families = {"Arm", "Arm", "Arm", "Arm", "Arm", "Arm"};
static const std::vector<std::string> names  = {"base", "pitch_1", "pitch_2", "pitch_3", "wrist", "gripper"};
static constexpr int gripper_index = 5;
static HebiHelper* helper_p;
static volatile bool feedback_ready;
static geometry_msgs::PoseStamped feedback_pose;
static sensor_msgs::JointState  feedback_joint_state;

// Function parameters for tuning
static constexpr double max_dist = 0.005; // meters, move_to function
static constexpr double tol_def = 0.55; // Default tolerance
// Max difference between actual and predicted speeds, per joint
static constexpr double tolerance[] = {tol_def, tol_def, tol_def, tol_def, tol_def, 1.2, 1.2}; 
static constexpr double time_deadzone = 0.15; // Time zone (s) in which collisions
                                              // are ignored at start and end of trajectory.
static bool disable_collisions = false;

//
// Stuff for handling trajectories
//

// Pouring beer
static bool pouring = false;
static double beer_nh;
static double beer_gh;
static geometry_msgs::Point action_pos_init;

// Salting
static bool salting = false;
static double saltTimes[3] = {3, 5, 8};

// Time
static ros::Time prev_time;
static double t_f;
static constexpr double max_time = 7; // No more than seven seconds.

// Positions (angular)
static double  q[num_joints];
static double  qdot[num_joints];

// Cubic spline parameters
static double  a[num_joints], b[num_joints], c[num_joints], d[num_joints];

// Max speeds.
static double  qdotmax[num_joints] = {.8, .8, .8, .8, .8, .33};

// static double default_pos[num_joints] = {0, 0.785, -1.57, -0.785, 0};

// Function declarations
static double hypot(double x, double y, double z);
static double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
static bool areWeThereYet(const geometry_msgs::PointStamped& target_loc, double dist);
static void initTrajectory(const sensor_msgs::JointState& target_joints, const bool use_cubic_spline, const double min_time);
static bool followTrajectory();
static geometry_msgs::Point getPourTrajectory(double t, double beer_ang_init, double beer_ang_max);
static geometry_msgs::Point getSaltTrajectory(double t);
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
    tmove = max_time < tmove ? max_time : tmove;

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
    // ROS_ERROR("t_f: %f", t_f);
}

// Handle feedback from hebi
static void processFeedback(const sensor_msgs::JointState& joints) {
    // Fix negatives
    feedback_joint_state = joints;
    feedback_joint_state.position[1] = -feedback_joint_state.position[1];
    feedback_joint_state.position[2] = -feedback_joint_state.position[2];
    feedback_joint_state.velocity[1] = -feedback_joint_state.velocity[1];
    feedback_joint_state.velocity[2] = -feedback_joint_state.velocity[2];

    geometry_msgs::Point pt = jointAnglesToPosition(feedback_joint_state);
    feedback_pose.pose.position = pt;
    feedback_pose.header.stamp = ros::Time::now();
    fkin_pub.publish(feedback_pose);

    feedback_ready = true;
    // ROS_ERROR("Got feedback");
    // ROS_INFO_STREAM("corrected feedback joints: " << feedback_joint_state);
    corrected_feedback_publisher.publish(feedback_joint_state);
}

// Block until we reach the target location target_loc
// Return true if the target was reached
static bool block(const geometry_msgs::Point& target_loc, double timeout_secs) {
    auto cur_time = ros::Time::now();
    while (!areWeThereYet(target_loc, max_dist) && (ros::Time::now() - cur_time).toSec() < timeout_secs) {
        ros::spinOnce();
        if(!followTrajectory()) {
            ROS_INFO("Failed moveto, returning false");
            return false;
        }
        usleep(100);
    }
    // ROS_INFO_STREAM("target_loc: " << target_loc << ", feedback: " << feedback_pose.pose.position);
    ROS_INFO("distance off: %f, time taken: %f, timeout: %f",
             distance(target_loc, feedback_pose.pose.position),
             (ros::Time::now() - cur_time).toSec(),
             timeout_secs);
    ROS_INFO_STREAM("target: " << target_loc << ", current: " << feedback_pose.pose.position);

    return true; //areWeThereYet(target_loc, max_dist);
}

// Return false if a collission is detected.
static bool followTrajectory() {
    sensor_msgs::JointState cmdMsg;
    cmdMsg.position.resize(num_joints);
    cmdMsg.velocity.resize(num_joints);
    cmdMsg.effort.resize(num_joints);

    // Advance time, but hold at t=0 to stay at the final position.
    // double t = (ros::Time::now() - prev_time).toSec();
    double t = (ros::Time::now() - prev_time).toSec() - t_f;
    double time_since_start = (ros::Time::now() - prev_time).toSec();
    double time_till_end = t_f - (ros::Time::now() - prev_time).toSec();
    if (t > 0.0)
        t = 0.0;

    double theta_pitch1 = feedback_joint_state.position[1];
    double theta_pitch2 = feedback_joint_state.position[2];
    // Compute the new position and velocity commands.
    if (pouring) {
        geometry_msgs::Point traj_loc;
        traj_loc = getPourTrajectory(t, 0, abs(pour_angle)); // curr time, min angle, max angle
        // ROS_INFO_STREAM("pour_loc: " << por_pos_init <<
                        // ", traj_loc: " << traj_loc << 
                        // ", current loc: " << feedback_pose.pose.position);

        cmdMsg             = positionToJointAngles(traj_loc);
        cmdMsg.effort.resize(num_joints);
        // Set the gripper position regardless
        q[5]               = a[5]+t*(b[5]+t*(c[5]+t*d[5]));
        cmdMsg.position[5] = q[5];
    } else if (salting){
        geometry_msgs::Point traj_loc;
        traj_loc = getSaltTrajectory(t); // curr time
        cmdMsg             = positionToJointAngles(traj_loc);
        cmdMsg.effort.resize(num_joints);
        // Set the gripper position regardless
        if (t < t_f/2)
            q[5] = M_PI;
        else
            q[5] = 0;
        cmdMsg.position[5] = q[5];
    } else {
        for (int i = 0 ; i < num_joints ; i++)
        {
            q[i]    = a[i]+t*(b[i]+t*(c[i]+t*d[i]));
            qdot[i] = b[i]+t*(2.0*c[i]+t*3.0*d[i]);

            cmdMsg.position[i] = q[i];
            cmdMsg.velocity[i] = qdot[i];
            cmdMsg.effort[i]   = 0;

            // -6 nM for pitch 2
            // -6 * cos(pitch1 + pitch2)

            // -16.5 nM for pitch 1 to -10.2 nM
            // -10 * cos(pitch1) + -6 * cos(pitch1 + pitch2)
            // We only check if we're in the middle of the trajectory
            if (feedback_ready && !disable_collisions &&
                    time_since_start > time_deadzone && time_till_end > time_deadzone) {
                if (abs(qdot[i] - feedback_joint_state.velocity[i]) > tolerance[i]) {
                    ROS_ERROR_THROTTLE(0.1,
                            "Collision detected on joint %d! "
                            "Qdot: %f, feedback: %f, delta: %f, tolerance: %f",
                            i, qdot[i], feedback_joint_state.velocity[i],
                            abs(qdot[i] - feedback_joint_state.velocity[i]),
                            tolerance[i]);

                    // Stop moving, since we hit something
                    auto temp = feedback_joint_state;
                    for (int j = 0; j < temp.velocity.size(); j++) {
                        temp.velocity[j] = 0;
                    }

                    helper_p->goToJointState(temp);
                    initTrajectory(temp, false, 0);
                    // helper_p->goToJointState(feedback_joint_state);
                    return false;
                }
            } else {
                // ROS_INFO_THROTTLE(time_deadzone, "IN DEAD ZONE TIME");
            }
        }
    } 

    cmdMsg.effort[1]   = -10 * cos(theta_pitch1)
                            + -6 * cos(theta_pitch1 + theta_pitch2);
    cmdMsg.effort[2]   = -6 * cos(theta_pitch1 + theta_pitch2);

    // Publish.
    cmdMsg.header.stamp = ros::Time::now();
    // ROS_INFO_STREAM("command: " << cmdMsg << "\nFeedback: " << feedback_joint_state);
    // cmdPub.publish(cmdMsg);
    helper_p->goToJointState(cmdMsg);
    return true;
}


//   angles in rad
static geometry_msgs::Point getPourTrajectory(double t, double beer_ang_init, double beer_ang_max) {
    t = t+t_f;
    if (t >= t_f) t=t_f;

    double x0 = action_pos_init.x;
    double y0 = action_pos_init.y;
    double z0 = action_pos_init.z;

    double base_ang_init = atan2(y0,x0);    // original base angle
    double r = sqrt(x0*x0 + y0*y0);         // radius

    double beer_ang;                        // current desired angle of beer
    if (t < t_f/2.0)    // half time to pour beer, half time to reset
        beer_ang = beer_ang_init + (t/t_f) * 2 * (beer_ang_max - beer_ang_init);
    else
        beer_ang = beer_ang_init + ((t_f - t) / t_f) * 2 * (beer_ang_max - beer_ang_init);
    helper_p->setPourAngle(-beer_ang);

    double ds = beer_nh * sin(beer_ang);          // horizontal offset from tipping bottle
    double base_ang_off = acos(1-(ds*ds)/(2*r*r));

    double x1, y1, z1;                          // desired position
    if (beer_ang > 3.14/2)
        z1 = z0 +  beer_gh * cos(beer_ang);
    else
        z1 = z0;

    x1 = r*cos(base_ang_init - base_ang_off);
    y1 = r*sin(base_ang_init - base_ang_off);

    geometry_msgs::Point ptmsg;
    ptmsg.x = x1;
    ptmsg.y = y1;
    ptmsg.z = z1;

    return ptmsg;
}



static geometry_msgs::Point getSaltTrajectory(double t) {
    t = t+t_f;
    if (t >= t_f) t=t_f;

    // 3 movements: windup, smash, reset


    double x0 = action_pos_init.x;
    double y0 = action_pos_init.y;
    double z0 = action_pos_init.z;

    double x1, y1, z1;                          // desired position

    z1 = beer_nh + z0 - z0 * sin(t/t_f*M_PI);
    x1 = x0;
    y1 = y0;


    geometry_msgs::Point ptmsg;
    ptmsg.x = x1;
    ptmsg.y = y1;
    ptmsg.z = z1;

    return ptmsg;
}

static bool isSanePosition(const geometry_msgs::Point& pt) {
    double theta = theta1(pt.y, pt.x);

    bool sane_axes = (pt.x > -.3  && pt.x < 1.2) &&
                     (pt.y > -.5  && pt.y < 1)  &&
                     (pt.z > -.05 && pt.z < .55); // Z probably should be positive
    bool sane_distance = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z) < 1.5; // reduce to 1.25?
    bool sane_theta = true; // theta < 2*M_PI/3 && theta > -M_PI/4;
    bool success = sane_axes && sane_distance && sane_theta;
    if (!success)
        ROS_WARN_STREAM("Insane pos: " << pt <<
                        "theta = " << theta <<
                        "sane_distance = " << sane_distance <<
                        "sane_axes = " << sane_axes <<
                        "sane thet = " << sane_theta);
    return success;
}

// Processes a request to move to some pose
bool MobilitySrv::processRequest(Mobility::Request& req, Mobility::Response& res) {
    // Parse the message into a joint state
    sensor_msgs::JointState joint_state = positionToJointAngles(req.target_loc);
    if (!isSanePosition(req.target_loc)
        || joint_state.position[1] == ANGLE_ERROR
        || joint_state.position[2] == ANGLE_ERROR)
    {
        res.target_reached = false;
        return false;
    }
    disable_collisions = req.disable_collisions;
    ROS_INFO("Collisions disabled? %s", (disable_collisions ? "yes" : "no"));

    joint_state.header.stamp = ros::Time::now();
    joint_state.name = joint_names;
    res.target_reached = true;

    if(req.close_gripper)
        joint_state.position[gripper_index] = -1;
    else
        joint_state.position[gripper_index] = 0;

    if (!req.pouring_beer)
        helper_p->setPourAngle(req.pour_angle);
    else
        pour_angle = req.pour_angle;

    // If use trajectory is false, it just sets the "time to move" to 0 seconds
    initTrajectory(joint_state, req.use_trajectory, req.move_time);

    if (!pouring) {
        pouring = req.pouring_beer;
        if (pouring) {
            ROS_INFO("Pouring is now true!");
            t_f = req.move_time;
            action_pos_init = req.target_loc;
            beer_nh = req.beer_nh;
            beer_gh = req.beer_gh;
        }
    }
    pouring = req.pouring_beer;

    if (!salting) {
        salting = req.salting_cup;
        if (salting) {
            ROS_INFO("Salting is now true!");
            t_f = req.move_time;
            beer_nh = req.beer_nh;
            action_pos_init = req.target_loc;
        }
    }
    salting = req.salting_cup;

    if (req.is_blocking) {
        res.target_reached = block(req.target_loc, req.move_time * 1.5);
    }

    joint_state_publisher.publish(joint_state);
    service_ready = true;

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mobility_server");
    ros::start();

    // Set up pub/sub

    ros::NodeHandle n;

    // Get topic names
    if (!n.getParam("current_pose_topic", current_pose_topic_name))
    {
      ROS_ERROR("target_gripper_state_topic param not specified");
      return -1;
    }
    fkin_pub = n.advertise<geometry_msgs::PoseStamped>(
            current_pose_topic_name, 10, &processFeedback);

    ros::Subscriber feedback_subscriber =
        n.subscribe(joint_state_feedback_name, 10,
                              &processFeedback);

    // Set up HEBI
    HebiHelper helper(n, "all", names, families);
    helper_p = &helper;


    ROS_INFO("Waiting for feedback");
    // Wait until we have acquired a feedback
    while (!feedback_ready && ros::ok()) {
        ros::spinOnce();
    }
    ROS_INFO("Feedback acquired");


    // Initialize the global variables before setting up the service.
    prev_time = ros::Time::now();
    for (int i = 0 ; i < 5 ; i++)
    {
        q[i]    = feedback_joint_state.position[i];
        qdot[i] = 0;

        a[i] = b[i] = c[i] = d[i] = 0.0;
    }
    // ROS_INFO_STREAM("feedback joints: " << feedback_joint_state);

    // feedback_joint_state.velocity.resize(num_joints);
    // feedback_joint_state.position.resize(num_joints);

    joint_state_publisher =
        n.advertise<sensor_msgs::JointState>(joint_state_name, 1);

    corrected_feedback_publisher =
        n.advertise<sensor_msgs::JointState>("/corrected_feedback", 1);

    // Set up the mobility service
    MobilitySrv mobility;
    ros::ServiceServer ss = n.advertiseService("mobility", &MobilitySrv::processRequest, &mobility);

    // if (!n.getParam("target_position_topic", target_subscriber_name))
    // {
    //   ROS_ERROR("target_gripper_state_topic param not specified");
    //   return -1;
    // }

    // ros::Subscriber target_subscriber =
    //     n.subscribe(target_subscriber_name, 10,
    //                           &processTargetState);

    ROS_INFO("About to loop");

    // TODO: add back in loop rate?
    while(ros::ok()) {
        if (service_ready)
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


