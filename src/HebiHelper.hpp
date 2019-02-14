#ifndef __HEBI_HELPER__
#define __HEBI_HELPER__
#include "ros/ros.h"
#include "hebiros/EntryListSrv.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "hebiros/SizeSrv.h"
class HebiHelper{
using namespace hebiros;

private:
    ros::NodeHandle n;
    sensor_msgs::JointState feedback;       // The actuator feedback struccture
    volatile int            feedbackvalid = 0;
    volatile double         goalpos;        // The goal position
    volatile int 			valid;
    volatile int 		    isValidPrev = 1;

    std::string group_name;
    std::vector<std::string> names;
    std::vector<std::string> families;

    ros::Subscriber goalSubscriber;
    ros::Subscriber validSubscriber;
    ros::Subscriber feedback_subscriber;
    ros::Publisher command_publisher;

    sensor_msgs::JointState command_msg;

    void setupGroup();
    void feedbackCallback(const sensor_msgs::JointState::ConstPtr& data);
    void goalCallback(const std_msgs::Float64::ConstPtr& msg);
    void validCallback(const std_msgs::Bool::ConstPtr& msg);

public:
    HebiHelper(std::string group_name,
        std::vector<std::string> names, std::vector<std::string> families);



};



#endif /** #ifndef __HEBI_HELPER__ **/
