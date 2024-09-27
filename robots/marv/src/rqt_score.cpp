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

    connect(ui_.start_follow_robot, &QPushButton::pressed, this, &RqtScore::start_follow);
    connect(ui_.stop_follow_robot, &QPushButton::pressed, this, &RqtScore::stop_follow);
    connect(ui_.reset_robot_push_button, &QPushButton::pressed, this, &RqtScore::reset_robot);
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

void RqtScore::start_follow() {
    robot_rodeo_gym::SetCameraFollow::Response response;

    auto ok = ros::service::call(start_camera_follow_service_name, follow_request, response);

    if(!ok) {
        ROS_ERROR_STREAM_NAMED(node_name, "Calling service " << start_camera_follow_service_name << " failed.");
    } else {
        ROS_INFO_STREAM_NAMED(node_name, "Calling service " << start_camera_follow_service_name << " successful.");
    }
}

void RqtScore::stop_follow() {
    std_srvs::Empty empty;

    auto ok = ros::service::call(stop_camera_follow_service_name, empty);

    if(!ok) {
        ROS_ERROR_STREAM_NAMED(node_name, "Calling service " << stop_camera_follow_service_name << " failed.");
    } else {
        ROS_INFO_STREAM_NAMED(node_name, "Calling service " << stop_camera_follow_service_name << " successful.");
    }
}

void RqtScore::reset_robot() {
    std_srvs::Empty empty;

    auto ok = ros::service::call(reset_robot_service_name, empty);

    if(!ok) {
        ROS_ERROR_STREAM_NAMED(node_name, "Calling service " << reset_robot_service_name << " failed.");
    } else {
        ROS_INFO_STREAM_NAMED(node_name, "Calling service " << reset_robot_service_name << " successful.");
    }
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

    if(!nh->getParam("robot_name", robot_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/robot_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    ui_.robot_name->setText(QString::fromStdString(robot_name));
    follow_request.robot_name.data = robot_name;

    if(!nh->getParam("arena_name", arena_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/arena_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    ui_.arena_name->setText(QString::fromStdString(arena_name));

    if(!nh->getParam("reset_robot_service_name", reset_robot_service_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/reset_robot_service_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

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

    if(!nh->getParam("start_camera_follow_service_name", start_camera_follow_service_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/start_camera_follow_service_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("stop_camera_follow_service_name", stop_camera_follow_service_name)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/stop_camera_follow_service_name is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("follow_offset/x", follow_request.offset.x)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/follow_offset/x is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("follow_offset/y", follow_request.offset.y)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/follow_offset/y is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("follow_offset/z", follow_request.offset.z)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/follow_offset/z is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    if(!nh->getParam("follow_offset/z", follow_request.offset.z)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/follow_offset/z is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }


    if(!nh->getParam("limits/min_distance", min_distance)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/limits/min_distance is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("limits/normal_distance", normal_distance)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/limits/normal_distance is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }
    if(!nh->getParam("limits/max_acc", max_acc)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/limits/max_acc is missing.";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    if(!nh->getParam("bins_count", bins_count)) {
        std::stringstream ss;
        ss << "Param " << nh->getNamespace() << "/bins_count is missing.";
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

        std::vector<double> lin_acc_x_max, lin_acc_y_max, lin_acc_z_max, shock, distance_max;
        lin_acc_x_max.resize(bins_count);
        lin_acc_y_max.resize(bins_count);
        lin_acc_z_max.resize(bins_count);
        distance_max.resize(bins_count);
        shock.resize(bins_count);

        ros::Duration total;

        std::vector<double> total_quality;

        for (const auto &obstacle_p : obstacles) {
            const Obstacle &obstacle = obstacle_p.second;
            if(obstacle.id < 0) continue; // skip

            ros::Duration duration = obstacle.end - obstacle.start;
            total += duration;
            std::stringstream ss;

            bool first = true;
            double operations = 0;
            double previous_time = 0.0;

            for (const auto& msg : obstacle.joy) {
                double t = msg.header.stamp.toSec(); // More straightforward way to get time in seconds
                if (first) {
                    first = false;
                    previous_time = t;
                }

                double dt = t - previous_time;
                previous_time = t;

                double sum = process_joy(msg);
                operations += sum * dt; // Assuming "obstacle" is the key you want to update
            }

            double normalized_shock = 0;
            double normalized_distance = 1;

            if(obstacle.imu.size() > bins_count) {
                find_max_bins(obstacle.imu_acc_x, lin_acc_x_max);
                find_max_bins(obstacle.imu_acc_y, lin_acc_y_max);
                find_max_bins(obstacle.imu_acc_z, lin_acc_z_max);

                for(int i = 0; i < bins_count; i++) {
                    shock[i] = std::sqrt(std::pow(lin_acc_x_max[i], 2) + std::pow(lin_acc_y_max[i], 2) + std::pow(lin_acc_z_max[i], 2));
                }

                normalized_shock = 1.0 - sigmoid(mean(shock), max_acc); // Invert logic
            }

            /*
             * TODO fix distance
            if(obstacle.distance_from_ground.size() > bins_count) {
                find_max_bins(obstacle.distance_from_ground, distance_max);

                normalized_distance = normalize_distance(mean(distance_max));
            }
             */


            //double quality = (normalized_distance + normalized_shock) / 2.0;
            double quality = normalized_shock;
            total_quality.push_back(quality);

            ss << "Obstacle " << obstacle.id << ": Duration: "
               << std::setw(2) << std::setfill('0') << ((duration.sec % 3600) / 60) << ":"
               << std::setw(2) << std::setfill('0') << (duration.sec % 60)
               //<< "; Operation: " << static_cast<int>(operations) << "; Quality: " << std::setprecision(2) << quality;
                    << "; Operation: " << static_cast<int>(operations) << "; Quality: " << static_cast<int>(quality * 100);

            QString item = QString::fromStdString(ss.str());

            data << item;
        }

        double seconds = total.toSec();
        int hours = static_cast<int>(seconds / 3600);
        int minutes = static_cast<int>(fmod(seconds, 3600) / 60);
        int seconds_int = static_cast<int>(fmod(seconds, 60));

        std::stringstream ss;
        ss << std::setw(2) << std::setfill('0') << hours << ":"
           << std::setw(2) << std::setfill('0') << minutes << ":"
           << std::setw(2) << std::setfill('0') << seconds_int;
        ui_.experiment_duration_lable->setText(QString::fromStdString(ss.str()));

        std::stringstream total_quality_ss;
        //total_quality_ss << std::setprecision(2) << mean(total_quality) << std::endl;
        total_quality_ss << static_cast<int>(100 * mean(total_quality)) << std::endl;
        ui_.total_score->setText(QString::fromStdString(total_quality_ss.str()));
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

        ui_.clear_push_button->setEnabled(false);
    }

    QCoreApplication::processEvents();
}

double RqtScore::sigmoid(double x, double x_min) {
    return 2.0*(1.0/(1.0+std::exp(-x/x_min)) - 0.5);
}

double RqtScore::mean(const std::vector<double> &vec) {
    double sum = 0.0;
    for (const auto& value : vec) {
        sum += value;
    }
    if(vec.empty()) return sum;

    return sum / vec.size();
}

double RqtScore::normalize_distance(double x) {
    return std::max((x - normal_distance) / (min_distance - normal_distance), 0.0);
}

void RqtScore::find_max_bins(const std::vector<double> &data, std::vector<double> &bins) {
    int l = data.size();
    double bin_size = static_cast<double>(l) / bins_count;

    int id_offset = std::floor(bin_size);

    for(int i = 0; i < bins_count; i++) {
        int start_id = i * id_offset;
        int end_id = std::min((i + 1) * id_offset, l - 1);
        if(end_id >= data.size()) end_id = data.size() - 1;

        bins[i] = *std::max_element(data.begin() + start_id, data.begin() + end_id);
    }
}

double RqtScore::process_joy(const sensor_msgs::Joy &joy) {
    double sum = 0.0;
    for (int i = 0; i < 7; i++) {
        double a = joy.axes[i];
        if (i == 2 || i == 5) {
            a = (a < 0.0)? 1.0 : 0.0;
        }
        a = std::abs(a);
        if (a > 0.0) {
            a = 1.0;
        }
        if (i == 3) { // Skip
            continue;
        }
        sum += a;
    }

    for (int i = 0; i < 10; i++) {
        double a = static_cast<double>(joy.buttons[i]);
        if (i > 5) { // allow only "A", "B", "X", "Y", "left_one_L1", "right_one_R1"
            a = 0.0;
        }
        sum += a;
    }
    return sum;
}

void RqtScore::imu_callback(const sensor_msgs::Imu &imu) {
    std::lock_guard<std::mutex> lock(data_mtx);
    imu_timestamp = ros::Time::now();

    if(started && obstacle_id_data > 0) {
        int active_obstacle = obstacle_id_data;
        obstacles[active_obstacle].end = ros::Time::now();
        obstacles[active_obstacle].imu.push_back(imu);

        obstacles[active_obstacle].imu_acc_x.push_back(imu.linear_acceleration.x);
        obstacles[active_obstacle].imu_acc_z.push_back(imu.linear_acceleration.y);
        obstacles[active_obstacle].imu_acc_y.push_back(imu.linear_acceleration.z);
    }
}

void RqtScore::joy_callback(const sensor_msgs::Joy &joy) {
    std::lock_guard<std::mutex> lock(data_mtx);
    joy_timestamp = ros::Time::now();

    if(started && obstacle_id_data > 0) {
        int active_obstacle = obstacle_id_data;
        obstacles[active_obstacle].end = ros::Time::now();
        obstacles[active_obstacle].joy.push_back(joy);
    }
}

void RqtScore::distance_from_ground_callback(const std_msgs::Float64 &distance) {
    std::lock_guard<std::mutex> lock(data_mtx);
    distance_from_ground_timestamp = ros::Time::now();

    if(started && obstacle_id_data > 0) {
        int active_obstacle = obstacle_id_data;
        obstacles[active_obstacle].end = ros::Time::now();
        obstacles[active_obstacle].distance_from_ground.push_back(distance.data);
    }
}

void RqtScore::obstacle_id_callback(const std_msgs::Int64 &obstacle_id) {
    std::lock_guard<std::mutex> lock(data_mtx);
    obstacle_id_timestamp = ros::Time::now();

    obstacle_id_data = obstacle_id.data;

    if(started && obstacle_id_data > 0) {

        int active_obstacle = obstacle_id_data;
        if (obstacles.find(active_obstacle) == obstacles.end()) { // Key is not present in the map
            obstacles[active_obstacle] = Obstacle();
            obstacles[active_obstacle].id = active_obstacle;
            obstacles[active_obstacle].start = ros::Time::now();
        }

        obstacles[active_obstacle].end = ros::Time::now();
    }
}

PLUGINLIB_EXPORT_CLASS(robot_rodeo_gym::RqtScore, rqt_gui_cpp::Plugin)