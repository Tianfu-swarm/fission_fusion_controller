#include "system_init.h"

void fissionFusion::handleProximityAvoidance(const sensor_msgs::msg::PointCloud2 msg)
{
    geometry_msgs::msg::Twist cmd_vel;

    // 累积障碍物方向的矢量
    float accum_x = 0.0;
    float accum_y = 0.0;

    // 遍历点云数据，step 为每个点占用的字节数（16 字节）
    for (size_t i = 0; i < msg.data.size(); i += 16) // 使用 msg.data 访问数据字段
    {
        // 获取每个点的 x, y, z 坐标
        const float *ptr_x = reinterpret_cast<const float *>(&msg.data[i]);
        const float *ptr_y = reinterpret_cast<const float *>(&msg.data[i + 4]);
        const float *ptr_z = reinterpret_cast<const float *>(&msg.data[i + 8]);

        float x = *ptr_x;
        float y = *ptr_y;
        float z = *ptr_z;

        // 仅处理前方范围内的点（假设 x > 0 表示前方）
        if (x > 0.0)
        {
            float weight = 1.0 / (std::hypot(x, y + 1e-5) * std::hypot(x, y + 1e-5)); // 距离越近权重越大
            accum_x += weight * x;
            accum_y += weight * y;
        }
    }

    // 计算障碍物方向矢量的角度
    float angle = std::atan2(accum_y, accum_x);                        // 累积矢量方向
    float distance = std::sqrt(accum_x * accum_x + accum_y * accum_y); // 累积矢量的长度

    // RCLCPP_INFO(this->get_logger(), "distance is :,%f", distance);
    // 控制逻辑
    if (distance > 0.0) // 如果累积矢量表示有障碍物
    {                   // RCLCPP_INFO(this->get_logger(), "angle is :,%f", angle);
        isAbstacle = true;
        // 创建随机设备和生成器
        std::random_device rd;
        std::mt19937 gen(rd());

        // 定义随机数分布范围 [0, 2]
        std::uniform_real_distribution<> dis(0.0, 2.0);

        // 生成随机速度
        double random_speed = dis(gen);
        if (distance < 15) // 如果障碍物非常近
        {
            cmd_vel.linear.x = abs(angle) < 0.5 ? -2 : 2;                         // 后退
            cmd_vel.angular.z = angle > 0 ? -3 * random_speed : 3 * random_speed; // 根据角度调整后退方向
        }
        else
        {
            // 如果角度在允许范围内，直行
            // if (std::abs(angle) > 1) // 假设大于 0.05 弧度的角度为直行范围
            // {
            //     cmd_vel.linear.x = 10;
            //     cmd_vel.angular.z = 0.0;
            //     // RCLCPP_INFO(this->get_logger(), "go straight...");
            // }
            // else
            // {

            // 如果角度不在直行范围内，根据角度转向
            if (angle < 0.0)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0 - random_speed; // 左转
                // RCLCPP_INFO(this->get_logger(), "turn left...");
            }
            else
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = random_speed; // 右转
                // RCLCPP_INFO(this->get_logger(), "turn right...");
            }
            // }
        }
    }
    else
    {
        // 没有检测到障碍物
        isAbstacle = false;
        return;
    }

    // 发布控制指令
    cmd_vel_publisher_->publish(cmd_vel);
}

void fissionFusion::avoidance()
{
    fissionFusion::handleProximityAvoidance(proximity_point);
}

void fissionFusion::configure(const std::string &yaml_file)
{
    try
    {
        YAML::Node yaml = YAML::LoadFile(yaml_file);

        // PD Parameters
        control_loop_duration = yaml["pd_parameters"]["control_loop_duration"].as<double>();
        Kp_distance = yaml["pd_parameters"]["Kp_distance"].as<double>();
        Kd_distance = yaml["pd_parameters"]["Kd_distance"].as<double>();
        Kp_angle = yaml["pd_parameters"]["Kp_angle"].as<double>();
        Kd_angle = yaml["pd_parameters"]["Kd_angle"].as<double>();
        max_velocity = yaml["pd_parameters"]["max_velocity"].as<double>();
        max_omega = yaml["pd_parameters"]["max_omega"].as<double>();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading configuration: " << e.what() << std::endl;
    }
}

void fissionFusion::update_subscriptions()
{
    auto topic_names_and_types = this->get_topic_names_and_types();
    for (const auto &topic : topic_names_and_types)
    {
        const auto &topic_name = topic.first;

        // 忽略无发布者的话题
        if (this->count_publishers(topic_name) == 0)
        {
            continue;
        }

        if (topic_name.find("/pose") != std::string::npos &&
            subscriptions_.find(topic_name) == subscriptions_.end())
        {
            std::string robot_id = topic_name.substr(0, topic_name.rfind("/pose"));

            auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_name,
                10,
                [this, robot_id](geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    all_pose_callback(robot_id, msg);
                });

            subscriptions_[topic_name] = sub;
            poses_[robot_id] = geometry_msgs::msg::PoseStamped();
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize the ROS 2 system

    auto node = std::make_shared<fissionFusion>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
