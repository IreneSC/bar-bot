#include "hebi_helper.hpp"

#define FLIP_PITCHES



HebiHelper::HebiHelper(ros::NodeHandle n,
        const std::string& group_name,
        const std::vector<std::string>& names,
        const std::vector<std::string>& families) :
    group_name(group_name), names(names), families(families)
{
    setupGroup(n);
    // Create a subscriber to listen for a goal.
    // goalSubscriber = n.subscribe("/goal", 100, &HebiHelper::goalCallback, this);
    // validSubscriber = n.subscribe("/valid", 100, &HebiHelper::validCallback, this);

    // Create a subscriber to receive feedback from the actuator group.
    // feedback_subscriber = n.subscribe("/hebiros/"+group_name+"/feedback/joint_state",
    //         100, &HebiHelper::feedbackCallback,this);

    // Create a publisher to send commands to the actuator group.
    command_publisher= n.advertise<sensor_msgs::JointState>
        ("/hebiros/"+group_name+"/command/joint_state", 100);

    // Read in gripper topic name
    std::string target_gripper_state_topic;
    if (!n.getParam("target_gripper_state_topic", target_gripper_state_topic))
    {
      ROS_ERROR("target_gripper_state_topic param not specified");
      return;
    }

    // Create a subscriber to recieve command to manipulate gripper
    // gripper_subscriber = n.subscribe(target_gripper_state_topic,
    //         100, &HebiHelper::gripperCallback,this);

    // command_msg.name = names;
    // command_msg.position.resize(names.size());
    // command_msg.velocity.resize(names.size());
    // command_msg.effort.resize(names.size());

    for (auto i = 0; i < names.size(); i++) {
        this->names[i] = families[i] + "/" + names[i];
    }
    // Wait until we have some feedback from the actuator.
    // ROS_INFO("Waiting for initial feedback");
    // while (!feedbackvalid)
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    // ROS_INFO("Obtained initial feedback");
}

void HebiHelper::setupGroup(ros::NodeHandle n){
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
    ROS_INFO("Waiting until added successfully");
    // Repeatedly call the service until it succeeds.
    while(!add_group_client.call(add_group_srv)){};
    ROS_INFO("Added successfully");

    // Check the size of this group.  This has an output argument.
    ros::ServiceClient size_client = n.serviceClient<SizeSrv>("/hebiros/"+group_name+"/size");
    SizeSrv size_srv;
    size_client.call(size_srv);
    ROS_INFO("%s has been created and has size %d", group_name.c_str(), size_srv.response.size);
}

// void HebiHelper::feedbackCallback(const sensor_msgs::JointState::ConstPtr& data)
// {
//     feedback = *data;
//     feedbackvalid = 1;
// }

// Returns previous value
// double HebiHelper::setGripperAngle(double angle) {
//     double prev = gripper_angle;
//     gripper_angle = angle;
//     return prev;
// }

// Returns previous value
double HebiHelper::setPourAngle(double angle) {
    double prev = pour_angle;
    pour_angle = angle;
    return prev;
}



// NOTE: This method copies, and thus does not modify, its argument
void HebiHelper::goToJointState(sensor_msgs::JointState joints)
{
    // ROS_INFO_STREAM("publishing joint state: " << joints);
#ifdef FLIP_PITCHES
    joints.position[1] = -joints.position[1];
    joints.position[2] = -joints.position[2];
    if (joints.velocity.size() > 2) {
        joints.velocity[1] = -joints.velocity[1];
        joints.velocity[2] = -joints.velocity[2];
    }
    if (joints.effort.size() > 2) {
        joints.effort[1] = joints.effort[1];
        joints.effort[2] = joints.effort[2];
    }
#endif
    joints.position[4] = pour_angle;

    joints.name = names;
    command_publisher.publish(joints);
    // ROS_INFO("After publish");
}
