#include <ignition/transport.hh>
#include <ignition/msgs/pose_v.pb.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#define NODE_NAME "GroundTruthPublisher"

class GroundTruthPublisher {
    ignition::transport::Node ign_node;
    ros::NodeHandle public_ros_node;
    ros::NodeHandle private_ros_node;
    std::string world_name;
    std::string robot_name;
    std::string frame_name;
    ros::Rate publish_rate = ros::Rate(1);

    tf2_ros::TransformBroadcaster tf_broadcaster;
    ros::Publisher odom_publisher;
    geometry_msgs::TransformStamped tf;
    ros::Time last_timestamp = ros::Time::now();
    nav_msgs::Odometry odom;
    bool tf_received = false;

    std::mutex mtx;

public:
    GroundTruthPublisher() : private_ros_node("~") {
        if(!private_ros_node.getParam("world_name", world_name)) {
            std::stringstream ss;
            ss << "Param " << private_ros_node.getNamespace() << "world_name is missing.";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
        if(!private_ros_node.getParam("robot_name", robot_name)) {
            std::stringstream ss;
            ss << "Param " << private_ros_node.getNamespace() << "robot_name is missing.";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }
        if(!private_ros_node.getParam("frame_name", frame_name)) {
            std::stringstream ss;
            ss << "Param " << private_ros_node.getNamespace() << "frame_name is missing.";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }

        double rate;
        if(!private_ros_node.getParam("publish_rate", rate)) {
            std::stringstream ss;
            ss << "Param " << private_ros_node.getNamespace() << "world_name is missing.";
            ROS_ERROR_STREAM(ss.str());
            throw std::runtime_error(ss.str());
        }

        publish_rate = ros::Rate(rate);

        odom_publisher = public_ros_node.advertise<nav_msgs::Odometry>("ground_truth_odom", 10);

        ign_node.Subscribe("/world/" + world_name + "/pose/info", &GroundTruthPublisher::OnPoseInfo, this);

        ROS_INFO_STREAM_NAMED(NODE_NAME, "Node initialized");
        ROS_INFO_STREAM_NAMED(NODE_NAME, "World name: " << world_name);
        ROS_INFO_STREAM_NAMED(NODE_NAME, "Robot name: " << robot_name);
        ROS_INFO_STREAM_NAMED(NODE_NAME, "Frame name: " << frame_name);
        ROS_INFO_STREAM_NAMED(NODE_NAME, "Publish rate: " << rate);
    }

    void OnPoseInfo(const ignition::msgs::Pose_V &msg) {
        const auto stamp = msg.header().stamp();

        for (const auto &pose: msg.pose()) {
            if (pose.name() == robot_name) {
                std::lock_guard<std::mutex> lock(mtx);

                tf_received = true;

                tf.header.stamp.sec = stamp.sec();
                tf.header.stamp.nsec = stamp.nsec();
                tf.header.frame_id = "world";
                tf.child_frame_id = frame_name;
                tf.transform.translation.x = pose.position().x();
                tf.transform.translation.y = pose.position().y();
                tf.transform.translation.z = pose.position().z();
                tf.transform.rotation.x = pose.orientation().x();
                tf.transform.rotation.y = pose.orientation().y();
                tf.transform.rotation.z = pose.orientation().z();
                tf.transform.rotation.w = pose.orientation().w();

                odom.header = tf.header;
                odom.child_frame_id = tf.child_frame_id;
                odom.pose.pose.position.x = tf.transform.translation.x;
                odom.pose.pose.position.y = tf.transform.translation.y;
                odom.pose.pose.position.z = tf.transform.translation.z;
                odom.pose.pose.orientation.x = tf.transform.rotation.x;
                odom.pose.pose.orientation.y = tf.transform.rotation.y;
                odom.pose.pose.orientation.z = tf.transform.rotation.z;
                odom.pose.pose.orientation.w = tf.transform.rotation.w;
            }
        }
    }

    void Loop() {
        while(ros::ok()) {
            if(tf_received) {
                std::lock_guard<std::mutex> lock(mtx);
                tf.header.stamp = ros::Time::now();
                odom.header.stamp = ros::Time::now();

                if(tf.header.stamp != last_timestamp) {
                    tf_broadcaster.sendTransform(tf);
                    last_timestamp = tf.header.stamp;
                } else {
                    ROS_ERROR_STREAM_NAMED("Ground truth publisher", "Publishing ground truth with same timestamp!!!!! " << tf.header.stamp);
                }

                odom_publisher.publish(odom);
                //ROS_INFO_STREAM_NAMED(NODE_NAME, "publishing");
            } else {
                ROS_WARN_STREAM_NAMED(NODE_NAME, "No ground truth transformation received from gazebo");
            }

            try {
                publish_rate.sleep();
            } catch (...) {
                ROS_INFO_STREAM_NAMED(NODE_NAME, "Quitting...");
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ground_truth_publisher");

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinner initialization");
    auto spinner = std::make_shared<ros::AsyncSpinner>(2);
    spinner->start();

    GroundTruthPublisher pub;
    pub.Loop();

    std::cout << "Quitting..." << std::endl;
}
