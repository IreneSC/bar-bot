#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <cmath>
#include "HebiHelper.hpp"

using namespace hebiros;


int main(int argc, char **argv)
{
    // Initialize the basic ROS node, run at 200Hz.
    // ros::init(argc, argv, "grabber_node");
    // HebiHelper helper((std::string)"all", std::vector<std::string>({"Arm"}),
    //     std::vector<std::string>({"manipulator"}));
}
