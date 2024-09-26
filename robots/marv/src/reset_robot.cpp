
#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <robot_rodeo_gym/SetEntityPose.h>
#include <Eigen/Eigen>

#define NODE_NAME "ResetRobot"


std::string front_left_pos_cmd_topic_name, front_right_pos_cmd_topic_name, rear_left_pos_cmd_topic_name, rear_right_pos_cmd_topic_name;
std::string robot_name;
int robot_id;

robot_rodeo_gym::SetEntityPose::Request req;
double front_left_pos_zero = 0;
double front_right_pos_zero = 0;
double rear_left_pos_zero = 0;
double rear_right_pos_zero = 0;

bool reset_robot_srv_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    bool success = true;
/*
    {
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
*/

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    auto nh = std::make_shared<ros::NodeHandle>("~");

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinner initialization");
    auto spinner = std::make_shared<ros::AsyncSpinner>(2);
    spinner->start();

    if(!nh->getParam("front_left_pos_cmd_topic_name", front_left_pos_cmd_topic_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/front_left_pos_cmd_topic_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("front_right_pos_cmd_topic_name", front_right_pos_cmd_topic_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/front_right_pos_cmd_topic_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("rear_left_pos_cmd_topic_name", rear_left_pos_cmd_topic_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/rear_left_pos_cmd_topic_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("rear_right_pos_cmd_topic_name", rear_right_pos_cmd_topic_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/rear_right_pos_cmd_topic_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    if(!nh->getParam("front_left_pos_zero", front_left_pos_zero)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/front_left_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("front_right_pos_zero", front_right_pos_zero)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/front_right_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("rear_left_pos_zero", rear_left_pos_zero)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/rear_left_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("rear_right_pos_zero", rear_right_pos_zero)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/rear_right_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    if(!nh->getParam("robot_name", req.pose.name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/robot_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("robot_id", robot_id)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/robot_id is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    req.pose.id = robot_id;

    if(!nh->getParam("origin/position/x", req.pose.pose.position.x)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/position/x is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("origin/position/y", req.pose.pose.position.y)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/position/y is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("origin/position/z", req.pose.pose.position.z)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/position/z is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }


    if(!nh->getParam("origin/orientation/x", req.pose.pose.orientation.x)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/orientation/x is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("origin/orientation/y", req.pose.pose.orientation.y)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/orientation/y is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("origin/orientation/z", req.pose.pose.orientation.z)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/orientation/z is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("origin/orientation/w", req.pose.pose.orientation.w)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/origin/orientation/w is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    ros::ServiceServer reset_robot_srv = nh->advertiseService("reset_robot", &reset_robot_srv_callback);

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Waiting for shutdown...");

    ros::waitForShutdown();

    std::cout << "Quitting.." << std::endl;
}
