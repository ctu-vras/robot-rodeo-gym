//
// Created by valda on 9/24/24.
//

#ifndef ROBOT_RODEO_GYM_RQT_SCORE_H
#define ROBOT_RODEO_GYM_RQT_SCORE_H


#include <QWidget>
#include <QMessageBox>
#include <QStringListModel>
#include <QStringList>

#include <rqt_gui_cpp/plugin.h>
#include <pluginlib/class_list_macros.h>
#include <ui_rqt_score.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>

namespace robot_rodeo_gym {

    class RqtScore
            : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT

        struct Obstacle {
            int id;
            ros::Time start;
            ros::Time end;
            std::vector<sensor_msgs::Imu> imu;
            std::vector<sensor_msgs::Joy> joy;
            std::vector<std_msgs::Float64> distance_from_ground;
        };


    public:
        RqtScore();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();


    private:
        std::string node_name;

        Ui::RqtScore ui_;
        QWidget* widget_;

        std::shared_ptr<ros::Timer> refresh_timer;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::AsyncSpinner> async_spinner;

        std::shared_ptr<ros::Subscriber> imu_sub, joy_sub, distance_from_ground_sub, obstacle_id_sub;
        std::mutex data_mtx;
        std_msgs::Int64 obstacle_id_data;

        bool started = false;
        ros::Time start_time = ros::Time::now();

        std::map<int, Obstacle> obstacles;

        void setup_ros();

        void refresh(const ros::TimerEvent& event);

        ros::Duration timeout;
        ros::Time imu_timestamp = ros::Time::now();
        ros::Time joy_timestamp = ros::Time::now();
        ros::Time distance_from_ground_timestamp = ros::Time::now();
        ros::Time obstacle_id_timestamp = ros::Time::now();

        void imu_callback(const sensor_msgs::Imu &imu);
        void joy_callback(const sensor_msgs::Joy &joy);
        void distance_from_ground_callback(const std_msgs::Float64 &ground_distance);
        void obstacle_id_callback(const std_msgs::Int64 &obstacle_id);

        void start();
        void stop();
        void clear();

        double sigmoid(double x, double x_min);

        double normalize_distance(double x);

        std::vector<double> find_max_bins(std::vector<double> data, int bins_count);
    };

}

#endif //ROBOT_RODEO_GYM_RQT_SCORE_H