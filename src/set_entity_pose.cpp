
#include <ignition/transport.hh>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/pose.pb.h>


#include <ros/ros.h>

#include <robot_rodeo_gym/SetEntityPose.h>

#include <std_srvs/Empty.h>

#define NODE_NAME "SetGazeboEntityPose"

std::shared_ptr<ignition::transport::Node> ign_node;

std::string world_name;

bool set_entity_srv_callback(robot_rodeo_gym::SetEntityPose::Request &request, robot_rodeo_gym::SetEntityPose::Response &response) {
    bool success = false;

    ignition::msgs::Pose msg;

    ignition::msgs::Boolean resp;

    msg.set_name(request.pose.name);
    msg.set_id(request.pose.id);
    msg.mutable_position()->set_x(request.pose.pose.position.x);
    msg.mutable_position()->set_y(request.pose.pose.position.y);
    msg.mutable_position()->set_z(request.pose.pose.position.z);

    msg.mutable_orientation()->set_x(request.pose.pose.orientation.x);
    msg.mutable_orientation()->set_y(request.pose.pose.orientation.y);
    msg.mutable_orientation()->set_z(request.pose.pose.orientation.z);
    msg.mutable_orientation()->set_w(request.pose.pose.orientation.w);

    std::stringstream ss;
    ss << "/world/" << world_name << "/set_pose";

    ign_node->Request(ss.str(), msg, 500, resp, success);

    if(!success) {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "Calling service failed.");
    }

    response.success = success;


    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    auto nh = std::make_shared<ros::NodeHandle>("~");

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinner initialization");
    auto spinner = std::make_shared<ros::AsyncSpinner>(2);
    spinner->start();

    ign_node = std::make_shared<ignition::transport::Node>();

    if(!nh->getParam("world_name", world_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/world_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    ros::ServiceServer set_entity_pose_srv = nh->advertiseService("set_entity_pose", &set_entity_srv_callback);

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Service listening at path: " << set_entity_pose_srv.getService());

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Waiting for shutdown...");

    ros::waitForShutdown();

    std::cout << "Quitting.." << std::endl;
}
