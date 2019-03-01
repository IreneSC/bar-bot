#ifndef __HEBI_HELPER__
#define __HEBI_HELPER__
#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "kinematics.hpp"

using namespace hebiros;
class HebiHelper{

private:
    // ros::NodeHandle n;
    sensor_msgs::JointState feedback;       // The actuator feedback struccture
    bool gripper_open = true;                   // Gripper state
    // volatile int            feedbackvalid = 0;
    // volatile double         goalpos;        // The goal position
    // volatile int            valid;
    // volatile int            isValidPrev = 1;

    std::string group_name;
    std::vector<std::string> names;
    std::vector<std::string> families;

    ros::Subscriber goalSubscriber;
    ros::Subscriber validSubscriber;
    ros::Subscriber feedback_subscriber;
    ros::Subscriber gripper_subscriber;
    ros::Publisher command_publisher;

    // sensor_msgs::JointState command_msg;

    void setupGroup(ros::NodeHandle n);
    void feedbackCallback(const sensor_msgs::JointState::ConstPtr& data);
    void gripperCallback(const std_msgs::Bool::ConstPtr& msg);
    // void goalCallback(const std_msgs::Float64::ConstPtr& msg);
    // void validCallback(const std_msgs::Bool::ConstPtr& msg);

public:
    HebiHelper(ros::NodeHandle n,
        const std::string& group_name,
        const std::vector<std::string>& names,
        const std::vector<std::string>& families);

    const inline sensor_msgs::JointState getFeedback() { return feedback; }

    // const inline int isValid() { return valid; }

    void goToJointState(sensor_msgs::JointState joints);

};

#endif /** #ifndef __HEBI_HELPER__ **/
