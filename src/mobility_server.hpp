#ifndef MOBILITY_SERVER_HPP
#define MOBILITY_SERVER_HPP

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include "kinematics.hpp"
#include "tf/transform_datatypes.h"
#include "hebi_helper.hpp"
#include "bar_bot/Mobility.h" // srv file

using namespace bar_bot;

class MobilitySrv {
public:
    bool processRequest(Mobility::Request& req, Mobility::Response& res);
};

#endif /* MOBILITY_SERVER_HPP */
