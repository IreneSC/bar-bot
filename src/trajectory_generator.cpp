/*
 **   trajectory_generator.cpp
 **
 **   Node that provides the trajectory generation
 **
 **   Service:   /movejoints
 **              /movetip
 **              /ismoving
 **
 **   Publish:   /joint_states   - this could be to the HEBI instead...
 **
 **   This file is modified from work by Günter Niemeyer.
 */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


/*
 **   Global Variables.  So the callbacks can pass information.
 */
// Inverse Kinematics Service
static ros::ServiceClient  *ikinClientPtr;

// Time and current position/velocity commands.
static double  t;
static double  q[5];
static double  qdot[5];

// Cubic spline parameters
static double  a[5], b[5], c[5], d[5];

// Max speeds.
static double  qdotmax[5] = {3.14, 3.14, 3.14, 3.14, 3.14};


/*
 **   Move To Computation
 */
double loadmove(double qfinal[5])
{
    int     i;
    double  tmove;        // Total move time
    double  tmp;

    // Pick a move time.  Note this is approximate.  We could compute
    // the absolute fastest time or pass as an argument.
    tmove = 0.25;
    for (i = 0 ; i < 5 ; i++)
    {
        tmp = 2.0 * fabs(qfinal[i] - q[i]) / qdotmax[i];
        if (tmp > tmove)
            tmove = tmp;
    }

    // Set the cubic spline parameters.
    for (i = 0 ; i < 5 ; i++)
    {
        a[i] = qfinal[i];
        b[i] = 0.0;
        c[i] = (3.0*(q[i]-qfinal[i])/tmove + qdot[i])/tmove;
        d[i] = (2.0*(q[i]-qfinal[i])/tmove + qdot[i])/tmove/tmove;
    }

    // Set the time, so the move starts t<0 and ends t=0.
    t = -tmove;
    return t;
}





/*
 **   Move Tip Callback
 */
bool movetipCallback(moveto::MoveTip::Request  &req,
        moveto::MoveTip::Response &res)
{
    // Call the inverse kinematics.
    moveto::IKin iKinSrv;
    iKinSrv.request.tip.x     = req.tip.x;
    iKinSrv.request.tip.y     = req.tip.y;
    iKinSrv.request.tip.z     = req.tip.z;
    iKinSrv.request.tip.pitch = req.tip.pitch;
    iKinSrv.request.tip.roll  = req.tip.roll;
    ikinClientPtr->call(iKinSrv);

    // Move to the joints.
    res.movetime = loadmove(&iKinSrv.response.joints.joint[0]);

    return true;
}


/*
 **   Move Joints Callback
 */
bool movejointsCallback(moveto::MoveJoints::Request  &req,
        moveto::MoveJoints::Response &res)
{
    // Move to the joints.
    res.movetime = loadmove(&req.joints.joint[0]);

    return true;
}


/*
 **   Main Code
 */
int main(int argc, char **argv)
{
    int  i;

    // Initialize ROS and create a node handle.
    ros::init(argc, argv, "moveto");
    ros::NodeHandle nh;

    // Prepare a 200Hz update rate.
    ros::Rate loop_rate(200);

    // Initialize the global variables before setting up the services.
    t = 0.0;
    for (i = 0 ; i < 5 ; i++)
    {
        q[i]    = 0.0;
        qdot[i] = 0.0;

        a[i] = b[i] = c[i] = d[i] = 0.0;
    }


    // Create a client to the IKin service.
    ros::ServiceClient ikinClient = nh.serviceClient<moveto::IKin>("/ikin");
    ikinClientPtr = &ikinClient;

    // Create the movejoints and movetip service.
    ros::ServiceServer movetipServer =
        nh.advertiseService("/movetip", movetipCallback);
    ros::ServiceServer movejointsServer =
        nh.advertiseService("/movejoints", movejointsCallback);

    // Create a publisher to send commands.
    ros::Publisher cmdPub =
        nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

    sensor_msgs::JointState cmdMsg;
    cmdMsg.name.push_back("134/1");
    cmdMsg.name.push_back("134/2");
    cmdMsg.name.push_back("134/3");
    cmdMsg.name.push_back("134/4");
    cmdMsg.name.push_back("134/5");
    cmdMsg.position.resize(5);
    cmdMsg.velocity.resize(5);


    // Run the servo loop until shutdown.
    double  dt = loop_rate.expectedCycleTime().toSec();
    ROS_INFO("MoveTo: Running the servo loop with dt %f", dt);
    while(ros::ok())
    {
        // Advance time, but hold at t=0 to stay at the final position.
        t += dt;
        if (t > 0.0)
            t = 0.0;

        // Compute the new position and velocity commands.
        for (i = 0 ; i < 5 ; i++)
        {
            q[i]    = a[i]+t*(b[i]+t*(c[i]+t*d[i]));
            qdot[i] = b[i]+t*(2.0*c[i]+t*3.0*d[i]);

            cmdMsg.position[i] = q[i];
            cmdMsg.velocity[i] = qdot[i];
        }

        // Publish.
        cmdMsg.header.stamp = ros::Time::now();
        cmdPub.publish(cmdMsg);

        // Wait for next cycle.
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
