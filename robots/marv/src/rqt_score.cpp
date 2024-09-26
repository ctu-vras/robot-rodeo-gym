#include "robot_rodeo_gym/rqt_score.h"

using namespace robot_rodeo_gym;

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
    node_name = context.argv()[0].toStdString();
    std::cout << "Setting node name: " << node_name << std::endl;

    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    ui_.imu_topic_checkbox->setFocusPolicy(Qt::NoFocus);
    ui_.imu_topic_checkbox->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui_.joy_topic_checkbox->setFocusPolicy(Qt::NoFocus);
    ui_.joy_topic_checkbox->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui_.distance_from_ground_topic_checkbox->setFocusPolicy(Qt::NoFocus);
    ui_.distance_from_ground_topic_checkbox->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui_.obstacle_id_topic_checkbox->setFocusPolicy(Qt::NoFocus);
    ui_.obstacle_id_topic_checkbox->setAttribute(Qt::WA_TransparentForMouseEvents);
    ui_.obstacles_list->setFocusPolicy(Qt::NoFocus);
    ui_.obstacles_list->setAttribute(Qt::WA_TransparentForMouseEvents);

    setup_ros();

    connect(ui_.start_push_button, &QPushButton::pressed, this, &RqtScore::start);
    connect(ui_.stop_push_button, &QPushButton::pressed, this, &RqtScore::stop);
    connect(ui_.clear_push_button, &QPushButton::pressed, this, &RqtScore::clear);

}

void RqtScore::shutdownPlugin()
{
    if (imu_sub) imu_sub->shutdown();
    if (joy_sub) joy_sub->shutdown();
    if (distance_from_ground_sub) distance_from_ground_sub->shutdown();
    if (obstacle_id_sub) obstacle_id_sub->shutdown();
    if (refresh_timer) refresh_timer->stop();

    if (nh) nh->shutdown();
    if (async_spinner) async_spinner->stop();
}

void RqtScore::start() {
    std::lock_guard<std::mutex> lock(data_mtx);

    obstacles.clear();

    start_time = ros::Time::now();
    started = true;
}

void RqtScore::stop() {
    std::lock_guard<std::mutex> lock(data_mtx);

    started = false;
}

void RqtScore::clear() {
    std::lock_guard<std::mutex> lock(data_mtx);

    obstacles.clear();
    ui_.experiment_duration_lable->setText(QString::fromStdString("00:00:00"));
}

void RqtScore::setup_ros() {
    char** argv = nullptr;
    int argc = 0;

    ros::init(argc, argv, node_name);

    ROS_INFO_STREAM_NAMED(node_name, "Spinner initialization");
    async_spinner = std::make_shared<ros::AsyncSpinner>(2);
    async_spinner->start();

    ROS_INFO_STREAM_NAMED(node_name, "Node handler initialization");
    nh = std::make_shared<ros::NodeHandle>(node_name);

    ROS_INFO_STREAM_NAMED(node_name, "Node name: " << ros::this_node::getName() << "; namespace: " << nh->getNamespace());

    double topic_timeout_s = 0.3;
    if(!nh->getParam("topic_timeout", topic_timeout_s)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/imu_topic is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    timeout = ros::Duration(topic_timeout_s);

    std::string imu_topic_name, joy_topic_name, distance_from_ground_name, obstacle_id_name;

    if(!nh->getParam("imu_topic", imu_topic_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/imu_topic is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("joy_topic", joy_topic_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/joy_topic is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("distance_from_ground_topic", distance_from_ground_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/distance_from_ground_topic is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("obstacle_id_topic", obstacle_id_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/obstacle_id_topic is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }


    ROS_INFO_STREAM_NAMED(node_name, "Subscribing to imu topic: " << imu_topic_name);
    imu_sub = std::make_shared<ros::Subscriber>(nh->subscribe(imu_topic_name, 1, &RqtScore::imu_callback, this));
    ui_.imu_topic_name->setText(QString::fromStdString(imu_topic_name));

    ROS_INFO_STREAM_NAMED(node_name, "Subscribing to joy topic: " << joy_topic_name);
    joy_sub = std::make_shared<ros::Subscriber>(nh->subscribe(joy_topic_name, 1, &RqtScore::joy_callback, this));
    ui_.joy_topic_name->setText(QString::fromStdString(joy_topic_name));

    ROS_INFO_STREAM_NAMED(node_name, "Subscribing to distance from ground topic: " << distance_from_ground_name);
    distance_from_ground_sub = std::make_shared<ros::Subscriber>(nh->subscribe(distance_from_ground_name, 1, &RqtScore::distance_from_ground_callback, this));
    ui_.distancew_from_ground_topic_name->setText(QString::fromStdString(distance_from_ground_name));

    ROS_INFO_STREAM_NAMED(node_name, "Subscribing to obstacle id topic: " << obstacle_id_name);
    obstacle_id_sub = std::make_shared<ros::Subscriber>(nh->subscribe(obstacle_id_name, 1, &RqtScore::obstacle_id_callback, this));
    ui_.obstacle_id_topic_name->setText(QString::fromStdString(obstacle_id_name));

    refresh_timer = std::make_shared<ros::Timer>(nh->createTimer(ros::Duration(0.25), &RqtScore::refresh, this));
}

void RqtScore::refresh(const ros::TimerEvent& event) {
    auto now = ros::Time::now();
    bool started_data;
    bool imu_timeout, joy_timeout, distance_from_ground_timeout, obstacle_id_timeout;
    ros::Duration duration;

    QStringList data;

    { // Copy all data
        std::lock_guard<std::mutex> lock(data_mtx);

        started_data = started;
        duration = now - start_time;
        imu_timeout = now > imu_timestamp + timeout;
        joy_timeout = now > joy_timestamp + timeout;
        distance_from_ground_timeout = now > distance_from_ground_timestamp + timeout;
        obstacle_id_timeout = now > obstacle_id_timestamp + timeout;

        for (const auto &obstaclePair : obstacles) {
            const Obstacle &obstacle = obstaclePair.second;
            ros::Duration duration = obstacle.end - obstacle.start;
            std::stringstream ss;
            ss << "Obstacle " << obstaclePair.first << ": Duration: "
               << std::setw(2) << std::setfill('0') << ((duration.sec % 3600) / 60) << ":"
               << std::setw(2) << std::setfill('0') << (duration.sec % 60)
               << "; Joy: " << obstacle.joy.size() << "; Imu: " << obstacle.imu.size() << "; Distance: " << obstacle.distance_from_ground.size() << "; Score: 0";
            QString item = QString::fromStdString(ss.str());

            data << item;
        }
    }

    // Assuming 'listView' is a pointer to your QListView
    QMetaObject::invokeMethod(this, [this, data]() {
        ui_.obstacles_list->reset();
        auto model = new QStringListModel(this);
        model->setStringList(data);
        ui_.obstacles_list->setModel(model);
    }, Qt::AutoConnection);

    ui_.imu_topic_checkbox->setCheckState(imu_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);
    ui_.joy_topic_checkbox->setCheckState(joy_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);
    ui_.distance_from_ground_topic_checkbox->setCheckState(distance_from_ground_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);
    ui_.obstacle_id_topic_checkbox->setCheckState(obstacle_id_timeout ? Qt::CheckState::Unchecked : Qt::CheckState::Checked);

    if(!started_data) {
        if(!imu_timeout && !joy_timeout && !distance_from_ground_timeout && !obstacle_id_timeout) {
            ui_.start_push_button->setEnabled(true);
        } else {
            ui_.start_push_button->setEnabled(false);
        }

        ui_.stop_push_button->setEnabled(false);

        ui_.clear_push_button->setEnabled(!data.empty());
    } else {
        if(imu_timeout || joy_timeout || distance_from_ground_timeout || obstacle_id_timeout) {
            QMessageBox::information(nullptr, "Alert", "Missing topic data. Stopping...");
            {
                std::lock_guard<std::mutex> lock(data_mtx);
                started = false;
            }
        }

        ui_.start_push_button->setEnabled(false);
        ui_.stop_push_button->setEnabled(true);


        double seconds = duration.toSec();
        int hours = static_cast<int>(seconds / 3600);
        int minutes = static_cast<int>(fmod(seconds, 3600) / 60);
        int seconds_int = static_cast<int>(fmod(seconds, 60));

        std::stringstream ss;
        ss << std::setw(2) << std::setfill('0') << hours << ":"
           << std::setw(2) << std::setfill('0') << minutes << ":"
           << std::setw(2) << std::setfill('0') << seconds_int;
        ui_.experiment_duration_lable->setText(QString::fromStdString(ss.str()));

        ui_.clear_push_button->setEnabled(false);
    }

    QCoreApplication::processEvents();
}

double RqtScore::sigmoid(double x, double x_min) {
    return 2.0*(1.0/(1.0+std::exp(-x/x_min)) - 0.5);
}

/*
double RqtScore::normalize_distance(double x) {
    return max((x - normal_distance) / (min_distance - normal_distance), 0);
}

std::vector<double> RqtScore::find_max_time_in_percentage(std::vector<double> data, int bins_count) {
    std::vector<double> bins;
    bins.resize(bins_count);

    int l = data.size();
    double bin_size = static_cast<double>(l) / bins_count;

    int id_offset = std::floor(bin_size);

    for(int i = 0; i < bins_count; i++) {
        int start_id = i * id_offset;
        int end_id = std::min((i + 1) * id_offset, l - 1);
        bins[i] = *std::max_element(data.begin() + start_id, data.end() + end_id);
    }

    return bins
}*/

void RqtScore::imu_callback(const sensor_msgs::Imu &imu) {
    std::lock_guard<std::mutex> lock(data_mtx);
    imu_timestamp = ros::Time::now();

    if(started) {
        int active_obstacle = obstacle_id_data.data;
        obstacles[active_obstacle].end = ros::Time::now();
        obstacles[active_obstacle].imu.push_back(imu);
    }
}

void RqtScore::joy_callback(const sensor_msgs::Joy &joy) {
    std::lock_guard<std::mutex> lock(data_mtx);
    joy_timestamp = ros::Time::now();

    if(started) {
        int active_obstacle = obstacle_id_data.data;
        obstacles[active_obstacle].end = ros::Time::now();
        obstacles[active_obstacle].joy.push_back(joy);
    }
}

void RqtScore::distance_from_ground_callback(const std_msgs::Float64 &distance) {
    std::lock_guard<std::mutex> lock(data_mtx);
    distance_from_ground_timestamp = ros::Time::now();



    if(started) {
        int active_obstacle = obstacle_id_data.data;
        obstacles[active_obstacle].end = ros::Time::now();
        obstacles[active_obstacle].distance_from_ground.push_back(distance);
    }
}

void RqtScore::obstacle_id_callback(const std_msgs::Int64 &obstacle_id) {
    std::lock_guard<std::mutex> lock(data_mtx);
    obstacle_id_timestamp = ros::Time::now();

    obstacle_id_data = obstacle_id;

    if(started) {

        int active_obstacle = obstacle_id_data.data;
        if (obstacles.find(active_obstacle) == obstacles.end()) { // Key is not present in the map
            obstacles[active_obstacle] = Obstacle();
            obstacles[active_obstacle].id = active_obstacle;
            obstacles[active_obstacle].start = ros::Time::now();
        }

        obstacles[active_obstacle].end = ros::Time::now();
    }
}

PLUGINLIB_EXPORT_CLASS(robot_rodeo_gym::RqtScore, rqt_gui_cpp::Plugin)