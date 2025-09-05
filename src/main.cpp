#include "system_init.h"

void fissionFusion::handleProximityAvoidance(const sensor_msgs::msg::PointCloud2 msg)
{
    geometry_msgs::msg::Twist cmd_vel;

    float min_r = 1e5;
    float near_x = 0.0f, near_y = 0.0f;

    for (size_t i = 0; i < msg.data.size(); i += 16)
    {
        const float *px = reinterpret_cast<const float *>(&msg.data[i + 0]);
        const float *py = reinterpret_cast<const float *>(&msg.data[i + 4]);
        // const float* pz = reinterpret_cast<const float*>(&msg.data[i + 8]);

        float x = *px, y = *py;

        if (!std::isfinite(x) || !std::isfinite(y))
            continue;

        float r = std::hypot(x, y);
        if (r < min_r && r > 0)
        {
            min_r = r;
            near_x = x;
            near_y = y;
        }
    }

    // 计算障碍物方向矢量的角度
    float angle = std::atan2(near_y, near_x);                      // 累积矢量方向
    float distance = std::sqrt(near_x * near_x + near_y * near_y); // 累积矢量的长度

    // 控制逻辑
    if (distance > 1e-5)
    {
        // RCLCPP_INFO(this->get_logger(), "angle is :,%f", angle);
        isAbstacle = true;
        // 创建随机设备和生成器
        std::random_device rd;
        std::mt19937 gen(rd());

        // 定义随机数分布范围 [0, 2]
        std::uniform_real_distribution<> dis(0.0, 2.0);

        // 生成随机速度
        double random_speed = dis(gen);
        if (distance < 0.0002) // 如果障碍物非常近
        {
            cmd_vel.linear.x = abs(angle) < 0.5 ? -2 : 2;                         // 后退
            cmd_vel.angular.z = angle > 0 ? -3 * random_speed : 3 * random_speed; // 根据角度调整后退方向
        }
        else
        {
            if (std::abs(angle) > 2)
            {
                cmd_vel.linear.x = 1.0;
                cmd_vel.angular.z = 0.0;
            }
            else // 如果角度不在直行范围内，根据角度转向
            {
                if (angle < 0.0) // 障碍在右侧
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = random_speed - 2 * angle; // 左转 z>0
                }
                else // 障碍在左侧
                {
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = -random_speed - 2 * angle; // 右转 z<0
                }
            }
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
