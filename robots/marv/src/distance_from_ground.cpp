//
// Created by valda on 30.8.24.
//

#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_pcl/grid_map_pcl.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>




#define NODE_NAME ("DistanceFromGround")

std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64>> distance_pub;
std::shared_ptr<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>> points_pub;
tf2_ros::Buffer tf_buffer;

bool get_transform(const std::string &target_frame, const std::string &source_frame, Eigen::Affine3d &transform) {
    try {
        auto fl = tf_buffer.lookupTransform( target_frame, source_frame, ros::Time(0));
        tf::transformMsgToEigen(fl.transform, transform);

        return true;
    }
    catch (tf2::TransformException &ex) {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "Get transform error. Target frame: " << target_frame << ", source frame: " << source_frame << ", exception: " << ex.what() << ". Returning identity transform.");
        transform = Eigen::Affine3d::Identity();

        return false;
    }
}

void height_map_callback(grid_map_msgs::GridMapConstPtr grid_map) {

    grid_map::GridMap gm;
    grid_map::GridMapRosConverter::fromMessage(*grid_map, gm);

    grid_map::Position pos;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;

    double step_size = 0.05;

    auto l = gm.getLength();

    Eigen::Affine3d transform, os, offset;

    if(!get_transform("world", "base_link", transform) ||
       !get_transform("os_sensor", "world", os)) {
        ROS_WARN_NAMED(NODE_NAME, "Skipping because obtaining transform failed.");
        return;
    }

    double min_dist = -std::numeric_limits<double>::max();

    int i = 0;

    for(int x = 0; x < 10; x++) {
        for(int y = 0; y < 8; y++) {
            offset.translation().x() = (x - 4.5)*step_size;
            offset.translation().y() = (y - 3.5)*step_size;
            Eigen::Affine3d coor = transform*offset;
            pos.x() = coor.translation().x();
            pos.y() = coor.translation().y();

            try {
                double z = gm.atPosition("elevation_inpainted", pos);

                Eigen::Affine3d p;
                p.translation().x() = pos.x();
                p.translation().y() = pos.y();
                p.translation().z() = z;

                p = os * p;

                if(!std::isnan(p.translation().z())) {

                    if(p.translation().z() > min_dist) {
                        min_dist = p.translation().z();
                    }

                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = 0.05;
                    marker.scale.y = 0.05;
                    marker.scale.z = 0.05;

                    marker.id = markers.markers.size();
                    marker.header.seq = grid_map->info.header.seq;
                    marker.header.stamp = ros::Time::now();
                    marker.header.frame_id = "os_sensor";

                    marker.color.a = 1;
                    marker.pose.orientation.w = 1;

                    marker.pose.position.x = p.translation().x();
                    marker.pose.position.y = p.translation().y();
                    marker.pose.position.z = p.translation().z();

                    markers.markers.push_back(marker);
                }
            }
            catch (std::out_of_range exception) {
                ROS_INFO_STREAM_NAMED(NODE_NAME, "Exception: " << i+1 << "/" << 10 * 8 << ". Ex: " << exception.what() << ". Position x: " << pos.x() << ", y: " << pos.y());
            }

            i++;
        }
    }

    if(distance_pub->trylock()) {
        distance_pub->msg_.data = min_dist;

        distance_pub->unlockAndPublish();
    }

    if(points_pub->trylock() && markers.markers.size() > 0) {
        points_pub->msg_.markers.clear();
        points_pub->msg_.markers.resize(markers.markers.size());
        std::copy(markers.markers.begin(), markers.markers.end(), points_pub->msg_.markers.begin());

        points_pub->unlockAndPublish();
    }
}

int main(int argc, char** argv) {

    std::cout <<  "Starting" << std::endl;
    ros::init(argc, argv, NODE_NAME);

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinner initialization");
    auto spinner = std::make_shared<ros::AsyncSpinner>(2);
    spinner->start();

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Node handler initialization");
    auto nh = std::make_shared<ros::NodeHandle>("~");

    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer);

    distance_pub = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::Float64>>(*nh, "/distance_from_ground", 1);
    points_pub = std::make_shared<realtime_tools::RealtimePublisher<visualization_msgs::MarkerArray>>(*nh, "/distance_from_ground_points", 1);

    ros::Subscriber elevation_map_sub = nh->subscribe("/elevation_mapping/elevation_map_raw", 1000, height_map_callback);

    ros::waitForShutdown();
    std::cout <<  "Quitting" << std::endl;
}