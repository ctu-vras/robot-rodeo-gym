
#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <robot_rodeo_gym/SetEntityPose.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64.h>

#define NODE_NAME "ResetRobot"


std::string front_left_pos_cmd_topic_name, front_right_pos_cmd_topic_name, rear_left_pos_cmd_topic_name, rear_right_pos_cmd_topic_name;
std::string robot_name;
std::string set_entity_pose_service_name;
int robot_id;

robot_rodeo_gym::SetEntityPose::Request req;

std_msgs::Float64 front_left_pos_cmd_msg, front_right_pos_cmd_msg, rear_left_pos_cmd_msg, rear_right_pos_cmd_msg;
std::shared_ptr<ros::Publisher> front_left_pos_cmd, front_right_pos_cmd, rear_left_pos_cmd, rear_right_pos_cmd;

bool reset_robot_srv_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    bool success = true;

    front_left_pos_cmd->publish(front_left_pos_cmd_msg);
    front_right_pos_cmd->publish(front_right_pos_cmd_msg);
    rear_left_pos_cmd->publish(rear_left_pos_cmd_msg);
    rear_right_pos_cmd->publish(rear_right_pos_cmd_msg);

    sleep(4);

    robot_rodeo_gym::SetEntityPose::Response resp;
    {
        bool ok = ros::service::call(set_entity_pose_service_name, req, resp);

        if(!ok) {
            success = false;
            ROS_ERROR_STREAM_NAMED(NODE_NAME, "Calling service " << set_entity_pose_service_name << " failed.");
        } else {
            ROS_INFO_STREAM_NAMED(NODE_NAME, "Calling service " << set_entity_pose_service_name << " successful.");
        }
    }

    return success;
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

    if(!nh->getParam("front_left_pos_zero", front_left_pos_cmd_msg.data)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/front_left_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("front_right_pos_zero", front_right_pos_cmd_msg.data)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/front_right_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("rear_left_pos_zero", rear_left_pos_cmd_msg.data)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/rear_left_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("rear_right_pos_zero", rear_right_pos_cmd_msg.data)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/rear_right_pos_zero is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    if(!nh->getParam("set_entity_pose_service_name", set_entity_pose_service_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/set_entity_pose_service_name is missing.";
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

    front_left_pos_cmd = std::make_shared<ros::Publisher>(nh->advertise<std_msgs::Float64>(front_left_pos_cmd_topic_name, 1));
    front_right_pos_cmd = std::make_shared<ros::Publisher>(nh->advertise<std_msgs::Float64>(front_right_pos_cmd_topic_name, 1));
    rear_left_pos_cmd = std::make_shared<ros::Publisher>(nh->advertise<std_msgs::Float64>(rear_left_pos_cmd_topic_name, 1));
    rear_right_pos_cmd = std::make_shared<ros::Publisher>(nh->advertise<std_msgs::Float64>(rear_right_pos_cmd_topic_name, 1));

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Waiting for shutdown...");

    ros::waitForShutdown();

    std::cout << "Quitting.." << std::endl;
}
