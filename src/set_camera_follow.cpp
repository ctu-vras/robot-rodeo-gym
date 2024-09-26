
#include <ignition/transport.hh>
#include <ignition/msgs/stringmsg.pb.h>


#include <ros/ros.h>

#include <robot_rodeo_gym/SetEntityPose.h>

#include <std_srvs/Empty.h>
#include <robot_rodeo_gym/SetCameraFollow.h>

#define NODE_NAME "SetGazeboCameraFollow"

std::shared_ptr<ignition::transport::Node> ign_node;

std::string world_name;

bool set_camera_follow_srv_callback(robot_rodeo_gym::SetCameraFollow::Request &request, robot_rodeo_gym::SetCameraFollow::Response &response) {
    bool success = true;

    ignition::msgs::Boolean resp;

    {
        ignition::msgs::StringMsg msg;

        msg.set_data(request.robot_name.data);

        std::stringstream ss;
        ss << "/gui/follow";

        bool ok;
        ign_node->Request(ss.str(), msg, 500, resp, ok);

        if(!ok) {
            success = false;
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "Calling service " << ss.str() << " failed.");
        } else {
            ROS_INFO_STREAM_NAMED(NODE_NAME, "Calling service " << ss.str() << " successful.");
        }
    }

    {
        ignition::msgs::Vector3d msg;

        std::stringstream ss;
        ss << "/gui/follow/offset";

        msg.set_x(request.offset.x);
        msg.set_y(request.offset.y);
        msg.set_z(request.offset.z);

        bool ok;
        ign_node->Request(ss.str(), msg, 500, resp, ok);

        if(!ok) {
            success = false;
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "Calling service " << ss.str() << " failed.");
        } else {
            ROS_INFO_STREAM_NAMED(NODE_NAME, "Calling service " << ss.str() << " successful.");
        }
    }

    response.success = success;


    return true;
}

bool stop_camera_follow_srv_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    ignition::msgs::Boolean resp;

    {
        ignition::msgs::StringMsg msg;

        msg.set_data("");

        std::stringstream ss;
        ss << "/gui/follow";

        bool ok;
        ign_node->Request(ss.str(), msg, 500, resp, ok);

        if(!ok) {
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "Calling service " << ss.str() << " failed.");
        } else {
            ROS_INFO_STREAM_NAMED(NODE_NAME, "Calling service " << ss.str() << " successful.");
        }
    }

    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    auto nh = std::make_shared<ros::NodeHandle>("~");

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinner initialization");
    auto spinner = std::make_shared<ros::AsyncSpinner>(2);
    spinner->start();

    ign_node = std::make_shared<ignition::transport::Node>();

    ros::ServiceServer set_entity_pose_srv = nh->advertiseService("set_camera_follow", &set_camera_follow_srv_callback);
    ros::ServiceServer stop_entity_pose_srv = nh->advertiseService("stop_camera_follow", &stop_camera_follow_srv_callback);

    //ROS_INFO_STREAM_NAMED(NODE_NAME, "Service listening at path: " << set_entity_pose_srv.getService());

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Waiting for shutdown...");

    ros::waitForShutdown();

    std::cout << "Quitting.." << std::endl;
}
