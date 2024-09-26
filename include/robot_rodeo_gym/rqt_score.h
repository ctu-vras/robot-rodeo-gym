//
// Created by valda on 9/24/24.
//

#ifndef ROBOT_RODEO_GYM_RQT_SCORE_H
#define ROBOT_RODEO_GYM_RQT_SCORE_H

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <ui_rqt_score.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>

namespace robot_rodeo_gym {

    class RqtScore
            : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        RqtScore();
        virtual void initPlugin(qt_gui_cpp::PluginContext& context);
        virtual void shutdownPlugin();


    private:
        Ui::RqtScore ui_;
        QWidget* widget_;

        std::shared_ptr<ros::Timer> refresh_timer;
        std::shared_ptr<ros::NodeHandle> nh;
        std::shared_ptr<ros::AsyncSpinner> async_spinner;
        std::shared_ptr<ros::Subscriber> imu_sub;
        std::shared_ptr<ros::Subscriber> ground_distance_sub;
        std::shared_ptr<ros::Subscriber> joy_sub;
        void setup_ros();

        void refresh(const ros::TimerEvent& event);

        ros::Time imu_timestamp = ros::Time::now();
        void imu_callback(const sensor_msgs::Imu &imu);

        ros::Time ground_distance_timestamp = ros::Time::now();
        void ground_distance_callback(const std_msgs::Float64 &ground_distance);

        ros::Time joy_timestamp = ros::Time::now();
        void joy_callback(const sensor_msgs::Joy &joy);
    };

}

#endif //ROBOT_RODEO_GYM_RQT_SCORE_H