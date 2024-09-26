//
// Created by valda on 24.4.24.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#define NODE_NAME ("CropPointCloud")

std::shared_ptr<ros::Publisher> pub;

pcl::CropBox<pcl::PCLPointCloud2> crop_box;

void markers_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_in) {

    auto* cloud_ptr = new pcl::PCLPointCloud2();
    pcl_conversions::toPCL(*cloud_in, *cloud_ptr);
    pcl::PCLPointCloud2::ConstPtr cptr(cloud_ptr);


    crop_box.setInputCloud(cptr);
    pcl::PCLPointCloud2 cloud_out;
    crop_box.filter(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromPCLPointCloud2(cloud_out, cloud_xyz);

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud_xyz, msg);
    pub->publish(msg);
}

int main(int argc, char** argv) {

    std::cout << "Starting" << std::endl;
    ros::init(argc, argv, NODE_NAME);

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Node handler initialization");
    ros::NodeHandle nh("~");

    if(argc != 9) {
        ROS_ERROR_STREAM_NAMED(NODE_NAME, "Invalid count of arguments. Is: " << argc << ", should: 9");
        return -1;
    }

    std::string sub_topic_name(argv[1]);
    ROS_INFO_STREAM_NAMED(NODE_NAME, "Subscribing to topic: " << sub_topic_name);
    auto sub =  nh.subscribe(sub_topic_name, 1000, markers_callback);

    double x_min = std::atof(argv[3]);
    double y_min = std::atof(argv[4]);
    double z_min = std::atof(argv[5]);

    double x_max = std::atof(argv[6]);
    double y_max = std::atof(argv[7]);
    double z_max = std::atof(argv[8]);
    ROS_INFO_STREAM_NAMED(NODE_NAME, "Cropping - x min: " << x_min << ", x max: "
                                                           << x_max << ", y min: " << y_min << ", y max: " << y_max << ", z min: " << z_min << ", z max: " << z_max);

    Eigen::Vector4d min{x_min, y_min, z_min, 1}, max{x_max, y_max, z_max, 1};
    crop_box.setMin(min.cast<float>());
    crop_box.setMax(max.cast<float>());

    std::string pub_topic_name(argv[2]);
    ROS_INFO_STREAM_NAMED(NODE_NAME, "Republishing to topic: " << pub_topic_name);
    pub = std::make_shared<ros::Publisher>(nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1));

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinning...");
    ros::spin();

    std::cout << "Exiting..." << std::endl;
}