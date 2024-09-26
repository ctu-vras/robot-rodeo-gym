#include <robot_rodeo_gym/rqt_score.h>

using namespace robot_rodeo_gym;

#define NODE_NAME "RobotRodeoGym"

RqtScore::RqtScore()
        : rqt_gui_cpp::Plugin()
        , widget_(0)
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName(QString("RqtScore"));
}

void RqtScore::initPlugin(qt_gui_cpp::PluginContext& context)
{
    setup_ros();

    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    connect(ui_.start_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));
}

void RqtScore::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void RqtScore::setup_ros() {
    ROS_INFO_STREAM_NAMED(NODE_NAME, "Spinner initialization");
    async_spinner = std::make_shared<ros::AsyncSpinner>(2);
    async_spinner->start();

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Node handler initialization");
    nh = std::make_shared<ros::NodeHandle>("~");

    std::string imu_topic_name{"/imu/data"}, ground_distance_topic_name{"/distance_from_ground"}, joy_topic_name{"/joy"};

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Subscribing to imu topic: " << imu_topic_name);
    imu_sub = std::make_shared<ros::Subscriber>(nh->subscribe(imu_topic_name, 1, &RqtScore::imu_callback, this));

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Subscribing to ground distance topic: " << ground_distance_topic_name);
    ground_distance_sub = std::make_shared<ros::Subscriber>(nh->subscribe(ground_distance_topic_name, 1, &RqtScore::ground_distance_callback, this));

    ROS_INFO_STREAM_NAMED(NODE_NAME, "Subscribing to joy topic: " << joy_topic_name);
    joy_sub = std::make_shared<ros::Subscriber>(nh->subscribe(joy_topic_name, 1, &RqtScore::joy_callback, this));


    refresh_timer = std::make_shared<ros::Timer>(nh->createTimer(ros::Duration(0.1), &RqtScore::refresh, this));
}

void RqtScore::refresh(const ros::TimerEvent& event) {
    auto now = ros::Time::now();
    ros::Duration timeout(0.3);

    auto imu_timeout = now > imu_timestamp + timeout;
    auto ground_distance_timeout = now > ground_distance_timestamp + timeout;
    auto joy_timeout = now > joy_timestamp + timeout;

    ui_.imu_data_ok_checkbox->setCheckState(imu_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);
    ui_.ground_distance_data_ok_checkbox->setCheckState(ground_distance_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);
    ui_.joy_data_ok_checkbox->setCheckState(joy_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);

    QCoreApplication::processEvents();
}

void RqtScore::imu_callback(const sensor_msgs::Imu &imu) {
    imu_timestamp = ros::Time::now();

    ROS_INFO_STREAM("Received imu");
}

void RqtScore::ground_distance_callback(const std_msgs::Float64 &ground_distance) {
    ground_distance_timestamp = ros::Time::now();

    ROS_INFO_STREAM("Received ground distance");
}

void RqtScore::joy_callback(const sensor_msgs::Joy &joy) {
    joy_timestamp = ros::Time::now();

    ROS_INFO_STREAM("Received joy");
}

PLUGINLIB_EXPORT_CLASS(robot_rodeo_gym::RqtScore, rqt_gui_cpp::Plugin)