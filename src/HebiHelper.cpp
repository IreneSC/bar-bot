#include "HebiHelper.hpp"

HebiHelper::HebiHelper(std::string group_name,
    std::vector<std::string> names, std::vector<std::string> families) :
    group_name(group_name), names(names), families(families){
        setupGroup();
    // Create a subscriber to listen for a goal.
    goalSubscriber = n.subscribe("/goal", 100, &HebiHelper::goalCallback, this);
    validSubscriber = n.subscribe("/valid", 100, &HebiHelper::validCallback, this);

    // Create a subscriber to receive feedback from the actuator group.
    feedback_subscriber = n.subscribe("/hebiros/"+group_name+"/feedback/joint_state",
        100, &HebiHelper::feedbackCallback,this);

    // Create a publisher to send commands to the actuator group.
    command_publisher= n.advertise<sensor_msgs::JointState>
        ("/hebiros/"+group_name+"/command/joint_state", 100);

    command_msg.name.push_back("Arm/tapedispenser");//TODO: take this as input
    command_msg.position.resize(1);
    command_msg.velocity.resize(1);
    command_msg.effort.resize(1);

    ros::Rate loop_rate(200);
    // Wait until we have some feedback from the actuator.
    ROS_INFO("Waiting for initial feedback");
    while (!feedbackvalid)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }


}

void HebiHelper::setupGroup(){
    // Ask the Hebi node to list the modules.  Create a client to their
    // service, instantiate a service class, and call.  This has no
    // input or output arguments.
    ros::ServiceClient entry_list_client = n.serviceClient<EntryListSrv>("/hebiros/entry_list");
    EntryListSrv entry_list_srv;
    entry_list_client.call(entry_list_srv);

    // Create a new "group of actuators".  This has input arguments,
    // which are the names of the actuators.
    ros::ServiceClient add_group_client = n.serviceClient<AddGroupFromNamesSrv>("/hebiros/add_group_from_names");
    AddGroupFromNamesSrv add_group_srv;
    add_group_srv.request.group_name = group_name;
    add_group_srv.request.names = names;
    add_group_srv.request.families = families;
    // Repeatedly call the service until it succeeds.
    while(!add_group_client.call(add_group_srv)) ;

    // Check the size of this group.  This has an output argument.
    ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");
    SizeSrv size_srv;
    size_client.call(size_srv);
    ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);
}

void HebiHelper::feedbackCallback(const sensor_msgs::JointState::ConstPtr& data)
{
  feedback = *data;
  feedbackvalid = 1;
}


/*
**   Goal Subscriber Callback
*/
void HebiHelper::goalCallback(const std_msgs::Float64::ConstPtr& msg)
{
  goalpos = msg->data;
}

/*
**   Valid goal Subscriber Callback
*/
void HebiHelper::validCallback(const std_msgs::Bool::ConstPtr& msg)
{
  isValidPrev = valid;
  valid = msg->data;
}
