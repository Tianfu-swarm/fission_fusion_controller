#include "system_init.h"
/*
 *  SFFM.cpp
 *  Stable Fusion-Fission Model
 *  Created on: 16 Feb 2025
 *  Author: Tianfu Zhang
 *  Email: tianfu.zhang@uni-konstanz.de
 */
void fissionFusion::sffm_controler_step()
{ // set up deferent desired_subgroup_size
    std::regex re("/bot(\\d+)");
    std::smatch match;

    int bot_id = -1; // 默认值，表示未匹配到

    if (std::regex_search(current_namespace, match, re))
    {
        bot_id = std::stoi(match[1]);
    }

    // pub rab
    Pub_rab();

    // boot time
    if ((this->get_clock()->now() - boot_time) < boot_wait_time)
    {
        current_state = RANDOM_WALK;
        // execute_state_behavior(current_state);
        srand(static_cast<unsigned int>(this->get_clock()->now().nanoseconds())); // 初始化随机种子

        int rand_int = rand() % 100;

        // 计算 jitter
        jitter_time = rand_int * 0.01;

        safe_publish_trigger();

        return;
    }

    double size;
    if (current_state != STAY)
    {
        estimated_group_size = 1;
    }
    else
    {
        size = extrema_propagation();
        if (size > 0 && estimated_group_size != size) // 只有估计了一个新值时才添加
        {
            estimated_group_size = size;
        }
    }

    estimated_group_size = smoothed_estimate_with_window(estimated_group_size);

    // double ground_truth = sffm_detect_group_size("/bot0");
    double actual_group_size = std::round(estimated_group_size);
    write_buffer << current_namespace << ","
                 << std::fixed << this->get_clock()->now().seconds() << ","
                 << estimated_group_size << ","
                 << actual_group_size << ","
                 //  << ground_truth << ","
                 << stability_window << ","
                 << current_pose.pose.position.x << ","
                 << current_pose.pose.position.y << ","
                 << K << "\n";

    // 每 5 秒写入一次文件
    rclcpp::Time now = this->get_clock()->now();
    std::string full_file_path = results_file_path + current_namespace + ".csv";
    std::filesystem::path dir_path = std::filesystem::path(full_file_path).parent_path();

    if (!std::filesystem::exists(dir_path))
    {
        std::filesystem::create_directories(dir_path);
    }

    if ((now - last_flush_time).seconds() > 5)
    {
        std::ofstream file(full_file_path, std::ios::app);
        if (file.is_open())
        {
            file << write_buffer.str();
            file.close(); // 自动 flush + 释放资源
            write_buffer.str("");
            write_buffer.clear();
            last_flush_time = now;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can't open file: %s", full_file_path.c_str());
        }
    }

    current_state = update_state(current_state);

    execute_state_behavior(current_state);

    safe_publish_trigger();
}

void fissionFusion::execute_state_behavior(robot_state state)
{
    switch (state)
    {
    case RANDOM_WALK:
    {
        // random_walk
        target_transform.header.frame_id.clear();

        auto [v, omega] = random_walk(
            /*mean_v=*/1.0, /*std_v=*/0.5,
            /*mean_ω=*/0.0, /*std_ω=*/0.5);
        geometry_msgs::msg::Twist twist_msg;

        geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning();

        if (local_planning_target.child_frame_id == "avoid_target")
        {
            std::pair<double, double> results = pd_control_to_target(local_planning_target);
            twist_msg.linear.x = results.first;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = results.second;
        }
        else
        {
            twist_msg.linear.x = v;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = omega;
        }

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }

    case FUSION:
    {

        // fusion
        fissionFusion::refresh_target_transform();
        std::pair<double, double> control_command;
        // 如果没有有效目标，返回 0 控制量
        if (target_transform.header.frame_id == "none" ||
            target_transform.header.frame_id == "non-follower" ||
            target_transform.header.frame_id.empty())
        {
            control_command.first = 0.0;
            control_command.second = 0.0;
        }
        else
        {
            control_command = pd_control_to_target(target_transform);
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = control_command.first;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = control_command.second;

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }

    case FISSION:
    { // random_walk
        target_transform.header.frame_id.clear();

        auto [v, omega] = random_walk(
            /*mean_v=*/1.0, /*std_v=*/0.5,
            /*mean_ω=*/0.0, /*std_ω=*/0.5);
        geometry_msgs::msg::Twist twist_msg;

        geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning();

        if (local_planning_target.child_frame_id == "avoid_target")
        {
            std::pair<double, double> results = pd_control_to_target(local_planning_target);
            twist_msg.linear.x = results.first;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = results.second;
        }
        else
        {
            twist_msg.linear.x = v;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = omega;
        }

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;

        // // fission
        // target_transform.header.frame_id.clear();

        // geometry_msgs::msg::Twist twist_msg;
        // geometry_msgs::msg::TransformStamped GroupGLJTarget = computeGroupGLJTarget(); // group glj is for keep far with a group
        // geometry_msgs::msg::TransformStamped GLJTarget = computeGLJTarget();           // when stay is to gather group,when fission is to gather all fissioner

        // double weight_fission = 0.3; // 权重1（fission 目标）
        // double weight_group = 0.7;   // 权重2（group 目标）

        // double weight_sum = weight_fission + weight_group;
        // if (weight_sum > 1e-6)
        // { // 防止除以0
        //     weight_fission /= weight_sum;
        //     weight_group /= weight_sum;
        // }

        // // 加权合并方向
        // double combined_x = weight_group * GLJTarget.transform.translation.x + weight_fission * GroupGLJTarget.transform.translation.x;
        // double combined_y = weight_group * GLJTarget.transform.translation.y + weight_fission * GroupGLJTarget.transform.translation.y;

        // // 构造目标 TransformStamped
        // geometry_msgs::msg::TransformStamped combined_target;
        // combined_target.header.stamp = rclcpp::Clock().now();
        // combined_target.header.frame_id = "base_link";
        // combined_target.transform.translation.x = combined_x;
        // combined_target.transform.translation.y = combined_y;
        // combined_target.transform.translation.z = 0.0;

        // combined_target.transform.rotation.x = 0.0;
        // combined_target.transform.rotation.y = 0.0;
        // combined_target.transform.rotation.z = 0.0;
        // combined_target.transform.rotation.w = 1.0;

        // if (fission_sign == 1 && original_group_child_frame_id_list.size() > 1) // 存在原始的group，存在同类的fissioner
        // {
        //     // std::cout << "combined_target is " << combined_target.transform.translation.x << ", " << combined_target.transform.translation.y << std::endl;
        //     // std::cout << "GLJTarget       is " << GLJTarget.transform.translation.x << ", " << GLJTarget.transform.translation.y << std::endl;
        //     // std::cout << "GroupGLJTarget  is " << GroupGLJTarget.transform.translation.x << ", " << GroupGLJTarget.transform.translation.y << std::endl;

        //     std::pair<double, double> control_command = pd_control_to_target(combined_target);

        //     twist_msg.linear.x = control_command.first;
        //     twist_msg.linear.y = 0.0;
        //     twist_msg.linear.z = 0.0;

        //     twist_msg.angular.x = 0.0;
        //     twist_msg.angular.y = 0.0;
        //     twist_msg.angular.z = control_command.second;
        // }
        // else
        // {
        //     geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning();
        //     if (local_planning_target.child_frame_id == "avoid_target")
        //     {
        //         std::pair<double, double> results = pd_control_to_target(local_planning_target);
        //         twist_msg.linear.x = results.first;
        //         twist_msg.linear.y = 0.0;
        //         twist_msg.linear.z = 0.0;

        //         twist_msg.angular.x = 0.0;
        //         twist_msg.angular.y = 0.0;
        //         twist_msg.angular.z = results.second;
        //     }
        //     else
        //     {
        //         auto [v, omega] = random_walk(
        //             /*mean_v=*/1.0, /*std_v=*/0.5,
        //             /*mean_ω=*/0.0, /*std_ω=*/0.5);
        //         twist_msg.linear.x = v;
        //         twist_msg.linear.y = 0.0;
        //         twist_msg.linear.z = 0.0;

        //         twist_msg.angular.x = 0.0;
        //         twist_msg.angular.y = 0.0;
        //         twist_msg.angular.z = omega;
        //     }
        // }

        // if (isAbstacle == false)
        // {
        //     cmd_vel_publisher_->publish(twist_msg);
        // }
        // break;
    }

    case STAY:
    {
        target_transform.header.frame_id.clear();

        std::pair<double, double> control_command;

        if (std::find(base_ground_data.data.begin(), base_ground_data.data.end(), 0.0) == base_ground_data.data.end())
        {
            // 全白
            geometry_msgs::msg::TransformStamped LookBlackTarget = look_for_black();
            control_command = pd_control_to_target(LookBlackTarget);

            std::cout << "look for black" << std::endl;
            std::cout << "LookBlackTarget.x = " << LookBlackTarget.transform.translation.x
                      << " LookBlackTarget.y = " << LookBlackTarget.transform.translation.y
                      << std::endl;

            std::cout << "control_command.x = " << control_command.first
                      << " control_command.y = " << control_command.second
                      << std::endl;
        }
        else
        {

            geometry_msgs::msg::TransformStamped GLJTarget = computeGLJTarget();
            geometry_msgs::msg::TransformStamped BlackTarget = stay_in_black();

            double w_glj = 0.0;
            double w_black = 0.3;

            geometry_msgs::msg::TransformStamped CombinedTarget;
            CombinedTarget.header.stamp = rclcpp::Clock().now();
            CombinedTarget.header.frame_id = "base_link";
            CombinedTarget.child_frame_id = "combined_target";

            // 平移加权
            double gx = GLJTarget.transform.translation.x;
            double gy = GLJTarget.transform.translation.y;
            double bx = BlackTarget.transform.translation.x;
            double by = BlackTarget.transform.translation.y;

            double tx = w_glj * gx + w_black * bx;
            double ty = w_glj * gy + w_black * by;

            CombinedTarget.transform.translation.x = tx;
            CombinedTarget.transform.translation.y = ty;
            CombinedTarget.transform.translation.z = 0.0;

            // 旋转直接设为单位四元数（默认朝向）
            CombinedTarget.transform.rotation.x = 0.0;
            CombinedTarget.transform.rotation.y = 0.0;
            CombinedTarget.transform.rotation.z = 0.0;
            CombinedTarget.transform.rotation.w = 1.0;

            control_command = pd_control_to_target(CombinedTarget);

            // std::cout << "stay on more black" << std::endl;
            // std::cout << "CombinedTarget.x = " << CombinedTarget.transform.translation.x
            //           << " CombinedTarget.y = " << CombinedTarget.transform.translation.y
            //           << std::endl;
            // std::cout << "CombinedTarget_command.x = " << control_command.first
            //           << " CombinedTarget_command.y = " << control_command.second
            //           << std::endl;
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = control_command.first;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = control_command.second;

        if (isAbstacle == false)
        {
            cmd_vel_publisher_->publish(twist_msg);
        }
        break;
    }
    default:
    {
        RCLCPP_ERROR(this->get_logger(), "No state, default RANDOM_WALK");
        current_state = RANDOM_WALK;
    }
    }
}

fissionFusion::robot_state fissionFusion::update_state(robot_state current_robot_state)
{
    switch (current_robot_state)
    {
    case RANDOM_WALK:
    {
        std::pair<double, double> follow_result = sffm_estimate_posibility_range(desired_subgroup_size,
                                                                                 arena_area,
                                                                                 n_groupsize);
        if (std::find(base_ground_data.data.begin(),
                      base_ground_data.data.end(),
                      0.0) != base_ground_data.data.end())
        {

            std::cout << "arrived black, return stay" << std::endl;

            initial_group_size = 1;
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
            return STAY; // 到达black
        }

        double follow_posibility = 1;
        double follow_radius = follow_range;

        sffm_choose_follow_target(follow_posibility, follow_radius);

        if (target_transform.header.frame_id == "none") // not found target
        {
            return RANDOM_WALK;
        }
        else if (target_transform.header.frame_id == "non-follower")
        {
            // non-follower
            initial_group_size = 1; //
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
            std::cout << "from random to stay" << std::endl;
            return STAY;
        }
        else
        {
            // RCLCPP_INFO(this->get_logger(), "from RANDOM_WALK to FUSION");
            return FUSION;
        }

        break;
    }

    case FUSION:
    {
        // 1. 检查目标是否丢失
        if (target_transform.header.frame_id == "none" ||
            target_transform.header.frame_id == "non-follower" ||
            target_transform.header.frame_id.empty())
        {
            return RANDOM_WALK;
        }

        // 2. 计算目标距离
        double dx = target_transform.transform.translation.x;
        double dy = target_transform.transform.translation.y;
        double delta_distance = std::sqrt(dx * dx + dy * dy);

        // 3. 到达目标 → STAY
        if (delta_distance < 0.5)
        {
            auto data = Extract_Rab_Data(target_id);

            // 需要能访问索引 1，所以至少 size == frame length；同时排除 NaN/Inf 和占位 -1
            if (data.size() == frame_length && std::isfinite(data[1]) && data[1] != -1)
            {
                initial_group_size = data[1];
            }
            else
            {
                return FUSION;
            }

            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(
                static_cast<double>(Waiting_time_scale_factor * initial_group_size)); // 加自己
            target_transform.child_frame_id.clear();
            // RCLCPP_INFO(this->get_logger(), "from FUSION to SATY");
            return STAY;
        }
        else if (isConCommunication) // 连续通信
        {
            // 提取目标 ID
            static const std::regex re(R"(bot(\d+)/)");
            std::smatch m;
            target_id = -1;
            if (std::regex_search(target_transform.child_frame_id, m, re) && m.size() >= 2)
            {
                target_id = std::stoi(m[1].str());
            }
            else
            {
                return RANDOM_WALK; // 提取失败，无法做通信判断
            }

            // 获取目标群体大小
            double target_group_size;
            if (Extract_Rab_Data(target_id)[1] != -1)
            {
                target_group_size = Extract_Rab_Data(target_id)[1];
            }
            else
            {
                std::cout << "extra failed" << std ::endl;
                return FUSION;
            }

            if (target_group_size == -1)
            {
                // std::cout << "con com return random not get target" << std::endl;
                return RANDOM_WALK; // 数据丢失或目标异常
            }

            // 通信策略：决定是继续融合、分裂还是放弃
            if (target_group_size < (desired_subgroup_size + groupsize_tolerance))
            {
                return FUSION; // 群体未满，继续融合
            }
            else
            {
                // std::cout << "con com return random" << std::endl;
                return RANDOM_WALK;
            }
        }
        else
        {
            // std::cout << "target frame id is " << target_transform.header.frame_id << std::endl;
            // 提取目标 ID
            static const std::regex re(R"(bot(\d+)/)");
            std::smatch m;
            target_id = -1;
            if (std::regex_search(target_transform.child_frame_id, m, re) && m.size() >= 2)
            {
                target_id = std::stoi(m[1].str());
            }
            else
            {
                return RANDOM_WALK; // 提取失败，无法做通信判断
            }

            double id_fission_sign;
            if (Extract_Rab_Data(target_id)[3] != -1)
            {
                id_fission_sign = Extract_Rab_Data(target_id)[3];
            }
            else
            {
                return FUSION;
            }

            if (id_fission_sign == 1)
            {
                return RANDOM_WALK;
            }
            else
            {
                return FUSION;
            }
        }

        break;
    }

    case FISSION:
    {
        stabilization_timer_started = false;
        initial_decision = UNDECIDED;

        if (fission_transform.child_frame_id.empty())
        {
            double min_distance = 1e6;
            for (const auto &tf : rab_tf.transforms)
            {
                double x = tf.transform.translation.x;
                double y = tf.transform.translation.y;
                double distance = std::sqrt(x * x + y * y);
                if (distance < min_distance)
                {
                    min_distance = distance;
                    fission_transform.child_frame_id = tf.child_frame_id;
                }
            }
        }
        else
        {
            bool found = false;

            if (!rab_tf.transforms.empty())
            {
                for (const auto &tf : rab_tf.transforms)
                {
                    if (tf.child_frame_id == fission_transform.child_frame_id)
                    {
                        fission_transform = tf;
                        found = true;
                        rab_tf.transforms.clear();
                        break;
                    }
                }

                if (!found)
                {
                    fission_transform.header.frame_id.clear();
                    return RANDOM_WALK;
                }
            }

            double x = fission_transform.transform.translation.x;
            double y = fission_transform.transform.translation.y;
            double distance = std::sqrt(x * x + y * y);
            std::cout << "distance to fission target " << fission_transform.child_frame_id
                      << " is " << distance << std::endl;
            if (distance > 1.2 * follow_range)
            {
                fission_transform.child_frame_id.clear();
                return RANDOM_WALK;
            }
        }
        return FISSION;
        // // fission前，首先选择要发fission的group是哪个
        // if (original_group_child_frame_id_list.empty())
        // {
        //     std::regex pattern(R"(bot([0-9]+)/base_link)");
        //     for (const auto &tf : rab_tf.transforms)
        //     {
        //         double x = tf.transform.translation.x;
        //         double y = tf.transform.translation.y;
        //         double distance = std::sqrt(x * x + y * y);

        //         if (distance > follow_range / 2)
        //         {
        //             continue;
        //         }

        //         if (tf.header.frame_id == tf.child_frame_id)
        //         {
        //             continue;
        //         }

        //         const std::string &frame_id = tf.child_frame_id;
        //         std::smatch match;

        //         if (std::regex_match(frame_id, match, pattern))
        //         {
        //             int id = std::stoi(match[1]);
        //             double id_fission_sign;
        //             auto target_data = Extract_Rab_Data(id);
        //             if (target_data.size() < frame_length)
        //             {
        //                 continue; // 不足4个数据，无法判断
        //             }

        //             id_fission_sign = target_data[3];

        //             if (id_fission_sign == 1.0)
        //             {
        //                 continue;
        //             }

        //             if (std::find(original_group_child_frame_id_list.begin(), original_group_child_frame_id_list.end(), id) == original_group_child_frame_id_list.end())
        //             {
        //                 original_group_child_frame_id_list.push_back(id);
        //             }
        //         }
        //     }
        // }

        // // 检查之后list还是0则返回RANDOM_WALK
        // if (original_group_child_frame_id_list.size() == 0)
        // {
        //     rclcpp::Time now = this->get_clock()->now();
        //     // std::cout << "WAIT FISSION -> RANDOM_WALK [No Original Group] [" << now.seconds() << "], jitter time is " << jitter_time << std::endl;
        //     if (now - fission_start_time >= rclcpp::Duration::from_seconds(jitter_time))
        //     {
        //         std::cout << "FISSION -> RANDOM_WALK [No Original Group] [" << now.seconds() << "]" << std::endl;
        //         return RANDOM_WALK;
        //     }

        //     // 还没到超时，继续留在 FISSION 重试
        //     return FISSION;
        // }

        // double sum_x = 0.0, sum_y = 0.0;
        // int count = 0;

        // for (const auto &target_id : original_group_child_frame_id_list)
        // {
        //     std::string child_frame_id = "bot" + std::to_string(target_id) + "/base_link";
        //     bool found = false;

        //     for (const auto &tf : rab_tf.transforms)
        //     {

        //         if (tf.child_frame_id == child_frame_id)
        //         {
        //             sum_x += tf.transform.translation.x;
        //             sum_y += tf.transform.translation.y;
        //             count++;
        //             found = true;
        //             break;
        //         }
        //     }

        //     if (!found)
        //     {
        //         std::cout << "[Warning] TF not found for " << child_frame_id << ", skipping." << std::endl;
        //         return FISSION;
        //     }
        // }

        // double center_dist = 0;
        // if (count > 0)
        // {
        //     double center_x = sum_x / count;
        //     double center_y = sum_y / count;
        //     center_dist = std::sqrt(center_x * center_x + center_y * center_y);
        // }

        // double distance_threshold = 1.1 * follow_range;
        // if (center_dist > distance_threshold)
        // {
        //     initial_group_size = 1;
        //     stay_start_time = this->get_clock()->now();
        //     wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
        //     std::cout << "FISSION to STAY.[" << stay_start_time.seconds() << "]" << std::endl;
        //     return STAY;
        // }

        // double fission_timeout_threshold = 2 * distance_threshold;

        // if (this->get_clock()->now() - fission_start_time > rclcpp::Duration::from_seconds(fission_timeout_threshold))
        // {
        //     std::cout << "FISSION timeout, switching to RANDOM_WALK." << std::endl;
        //     return RANDOM_WALK;
        // }

        // // std::cout << "distance to original group center is " << center_dist << " threshold distance is " << distance_threshold << std::endl;

        // return FISSION;
    }

    case STAY:
    {

        double actual_group_size = std::round(estimated_group_size);

        if (groupsize_tolerance > 0)
        {
            // —— 已作出决策：统一延迟窗口，等待 1s 再返回决策（防止抖动干扰） ——
            if (initial_decision != UNDECIDED)
            {
                auto now = this->get_clock()->now();
                double wait_seconds = (initial_decision == STAY) ? 1.5 : 1.0; // STAY 等 1.5 秒，FISSION 等 1 秒

                if (now - decision_start_time >= rclcpp::Duration::from_seconds(wait_seconds))
                {
                    robot_state decision = initial_decision;
                    initial_decision = UNDECIDED;

                    if (decision == STAY)
                    {
                        std::cout << "time now " << now.seconds() << " decision time is " << decision_start_time.seconds() << " return STAY" << std::endl;
                    }
                    if (decision == FISSION)
                    {
                        fission_start_time = now; // 用 now 保证时间一致
                        std::cout << "time now " << now.seconds() << " decision time is " << decision_start_time.seconds() << " return FISSION" << std::endl;
                    }
                    return decision; // 返回决策
                }

                target_transform.child_frame_id.clear();
                return STAY; // 等待期间保持 STAY
            }

            // —— 未决策 && 超阈值：进入稳定期 ——
            if (actual_group_size >= (desired_subgroup_size + groupsize_tolerance))
            {
                if (!stabilization_timer_started)
                {
                    double duration_seconds = 0.5 * actual_group_size;
                    stabilization_duration = rclcpp::Duration::from_seconds(duration_seconds);
                    stabilization_start_time = this->get_clock()->now();
                    stabilization_timer_started = true;
                    std::cout << "enter stabilization duration, time is " << stabilization_start_time.seconds()
                              << ", current group size is " << actual_group_size
                              << " desired group size is " << desired_subgroup_size
                              << " groupsize_tolerance is " << groupsize_tolerance
                              << std::endl;
                }

                rclcpp::Time current_time = this->get_clock()->now();
                if ((current_time - stabilization_start_time) < stabilization_duration)
                {
                    // 稳定期未结束，保持 STAY 状态
                    target_transform.child_frame_id.clear();
                    return STAY;
                }

                // 稳定期结束后，进行正常判断逻辑
                std::pair<double, double> follow_result = sffm_estimate_posibility_range(desired_subgroup_size,
                                                                                         arena_area,
                                                                                         actual_group_size);

                double follow_posibility = 1 - ((actual_group_size - desired_subgroup_size) / actual_group_size);
                double follow_radius = 2;

                if (initial_decision == UNDECIDED) // 没有状态时才定义
                {
                    sffm_choose_follow_target(follow_posibility, follow_radius);
                    if (target_transform.header.frame_id == "non-follower")
                    {
                        original_group_child_frame_id_list.clear();
                        initial_decision = FISSION;
                        decision_start_time = this->get_clock()->now();
                        std::cout << "[DECISION] First decision: FISSION Time is " << decision_start_time.seconds() << std::endl;
                        return STAY; // 下一帧进入“已决策延迟窗口”
                    }
                    else
                    {
                        Maintain_state_start_time = this->get_clock()->now();
                        target_transform.child_frame_id.clear();
                        initial_decision = STAY;
                        decision_start_time = this->get_clock()->now();
                        std::cout << "[DECISION] First decision: STAY Time is " << decision_start_time.seconds() << std::endl;
                        return STAY;
                    }
                }
            }
            else
            {
                // std::cout << "estimated_group_size =  " << estimated_group_size << std::endl;
                // 群体未超阈值，重置稳定状态，避免卡在“稳定期未结束”
                stabilization_timer_started = false;
            }

            if (actual_group_size < (desired_subgroup_size + groupsize_tolerance) && actual_group_size >= desired_subgroup_size) // size介于期望大小区间内
            {
                target_transform.child_frame_id.clear();
                return STAY;
            }
        }
        else
        {

            if (actual_group_size < desired_subgroup_size)
            {
                if (this->get_clock()->now() - Maintain_state_start_time < rclcpp::Duration::from_seconds(1.0))
                {
                    target_transform.child_frame_id.clear();
                    return STAY;
                }

                if (actual_group_size != initial_group_size)
                { // 如果 group size 大了，更新等待时间
                    if (actual_group_size > initial_group_size)
                    {
                        // std::cout << "size larger,from " << initial_group_size << " to " << actual_group_size << std::endl;
                        initial_group_size = actual_group_size; // 更新 group size 记录
                        wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
                        stay_start_time = this->get_clock()->now(); // 重置等待
                    }
                    // 如果 group size小了，比较剩余时间与新时间，取小
                    if (actual_group_size < initial_group_size)
                    {
                        rclcpp::Time now = this->get_clock()->now();
                        rclcpp::Duration elapsed_time = now - stay_start_time;
                        rclcpp::Duration remaining_time = wait_time - elapsed_time;

                        rclcpp::Duration new_wait_time = rclcpp::Duration::from_seconds(
                            static_cast<double>(Waiting_time_scale_factor * actual_group_size));

                        if (new_wait_time < remaining_time)
                        {
                            wait_time = new_wait_time;
                            stay_start_time = now; // 重置起始时间
                        }

                        initial_group_size = actual_group_size;
                    }
                }

                // 检查当前时间是否超过 wait_time
                rclcpp::Time time_now = this->get_clock()->now();
                if ((time_now - stay_start_time).seconds() > wait_time.seconds())
                {
                    std::cout << "\033[1;32m" << "[Stay]->[Fission]" << "\033[0m"
                              << " Waiting time Run out,"
                              << "start time = " << stay_start_time.seconds()
                              << ", wait time = " << wait_time.seconds()
                              << " initial_group_size:" << initial_group_size
                              << std::endl;

                    fission_start_time = this->get_clock()->now();
                    original_group_child_frame_id_list.clear();
                    return FISSION;
                }
                else
                {
                    target_transform.child_frame_id.clear();
                    return STAY;
                }
            }
            else
            {
                std::pair<double, double> follow_result = sffm_estimate_posibility_range(desired_subgroup_size,
                                                                                         arena_area,
                                                                                         actual_group_size);

                double follow_posibility = 1 - ((actual_group_size - desired_subgroup_size) / actual_group_size);
                double follow_radius = 2;

                sffm_choose_follow_target(follow_posibility, follow_radius);
                if (target_transform.header.frame_id == "non-follower")
                {
                    original_group_child_frame_id_list.clear();
                    fission_start_time = this->get_clock()->now();
                    return FISSION; //
                }
                else
                {
                    target_transform.child_frame_id.clear();
                    Maintain_state_start_time = this->get_clock()->now();
                    return STAY;
                }
            }
        }
        std::cout << "current size is " << actual_group_size << std::endl;
        return STAY;
    }

    default:
        return RANDOM_WALK;
    }
}

void fissionFusion::sffm_choose_follow_target(double posibility, double follow_radius)
{
    target_transform.header.frame_id = "none";

    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    if (dist01(rng) > posibility)
    {
        target_transform.header.frame_id = "non-follower";
        return;
    }

    const double r2 = follow_radius * follow_radius;
    std::vector<const geometry_msgs::msg::TransformStamped *> candidates;
    candidates.reserve(tf_entries.size());

    for (const auto &e : tf_entries)
    {
        if (e.tf->child_frame_id == e.tf->header.frame_id)
            continue; // 自己
        if (e.is_on_gray)
            continue; // 跳过 fissioner
        if (e.d2 > r2)
            continue;

        if (isMinCommunication || isConCommunication)
        {
            if (e.id == -1)
                continue;
            const double *frame = get_rab_ptr(e.id);
            if (!frame)
                continue;
            if (frame[1] == -1)
                continue; // 无 group size

            double target_group_size = frame[1];
            if (target_group_size >= desired_subgroup_size + groupsize_tolerance)
                continue;

            double target_desired = frame[2]; // 对方 desired_subgroup_size
            if (target_desired != desired_subgroup_size)
            {
                continue; // 不考虑 desired 不同的机器人
            }
        }
        candidates.push_back(e.tf);
    }

    if (!candidates.empty())
    {
        std::uniform_int_distribution<size_t> pick(0, candidates.size() - 1);
        target_transform = *candidates[pick(rng)];
    }
    else
    {
        target_transform.header.frame_id = "none";
    }
}

double fissionFusion::sffm_detect_group_size(std::string target_namespace)
{
    std::set<std::string> visited; // 记录访问过的粒子
    std::queue<std::string> q;     // 用于 BFS 遍历群体
    double group_size = 0;         // 计数群体大小

    std::string current_key = target_namespace;

    // 2. BFS 扩展群体
    q.push(current_key);
    visited.insert(current_key);

    while (!q.empty())
    {
        std::string node = q.front();
        q.pop();
        group_size++;

        const auto &node_pose = poses_[node];

        // 遍历所有其他粒子
        for (const auto &[neighbor_id, neighbor_pose] : poses_)
        {
            if (visited.count(neighbor_id) == 0)
            { // 如果未访问
                double distance = calculate_distance(node_pose, neighbor_pose);
                if (distance <= group_size_distance_threshold)
                {
                    q.push(neighbor_id);
                    visited.insert(neighbor_id);
                }
            }
        }
    }

    return group_size;
}

double fissionFusion::calculate_distance(const geometry_msgs::msg::PoseStamped &p1,
                                         const geometry_msgs::msg::PoseStamped &p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// local path planning
geometry_msgs::msg::TransformStamped fissionFusion::local_path_planning(double inflation_distance)
{
    geometry_msgs::msg::TransformStamped target;
    target.header.frame_id = "base_link";
    target.child_frame_id = "none";
    target.transform.rotation.w = 1.0;

    const double offset_angle = 50.0 * M_PI / 180.0;
    const double factor = inflation_distance; // 避障距离

    double best_d2 = std::numeric_limits<double>::infinity();
    const TfEntry *nearest = nullptr;

    for (const auto &e : tf_entries)
    {
        if (e.tf->child_frame_id == e.tf->header.frame_id)
            continue; // 跳过自己
        if (e.d2 < best_d2)
        {
            best_d2 = e.d2;
            nearest = &e;
        }
    }

    if (nearest && best_d2 < factor * factor)
    {
        const double dx = nearest->dx, dy = nearest->dy;
        const double angle_to_other = std::atan2(dy, dx);

        const double tangent_left = angle_to_other + M_PI_2;
        const double tangent_right = angle_to_other - M_PI_2;

        const double left_x = std::cos(tangent_left), left_y = std::sin(tangent_left);
        const double right_x = std::cos(tangent_right), right_y = std::sin(tangent_right);

        const double cos_left = left_x; // 与前向 [1,0] 的点积
        const double cos_right = right_x;

        const double tangent_angle = (cos_left > cos_right)
                                         ? (tangent_left + offset_angle)
                                         : (tangent_right - offset_angle);

        const double avoid_step = 0.5;
        target.child_frame_id = "avoid_target";
        target.transform.translation.x = avoid_step * std::cos(tangent_angle);
        target.transform.translation.y = avoid_step * std::sin(tangent_angle);
    }
    else
    {
        target.transform.translation.x = 0.0;
        target.transform.translation.y = 0.0;
    }
    return target;
}

// 计算二项分布的概率质量函数 (PMF)
double fissionFusion::binomial_pmf(int n, int m, double p)
{
    if (m > n || m < 0)
        return 0.0;

    // 计算组合数 C(n, m)
    double C = 1.0;
    for (int i = 0; i < m; ++i)
    {
        C *= (n - i) / static_cast<double>(i + 1);
    }

    // 计算二项分布的概率
    return C * std::pow(p, m) * std::pow(1 - p, n - m);
}

// 计算对数正态分布的概率密度函数 (PDF)
double fissionFusion::lognormal_pdf(double x, double mu, double sigma)
{
    if (x <= 0)
        return 0.0;
    double exponent = -(std::pow(std::log(x) - mu, 2) / (2 * sigma * sigma));
    return (1.0 / (x * sigma * std::sqrt(2 * M_PI))) * std::exp(exponent);
}

// 计算最大似然估计 (MLE) 的 p, mu, sigma
std::tuple<double, double, double> fissionFusion::maximum_likelihood_estimation(int total_robots, int total_observed_groups)
{
    double best_p = 0.0, best_mu = 0.0, best_sigma = 0.0;
    double best_prob = 0.0;

    for (double p = 0.5; p <= 1.0; p += 0.01)
    {
        for (double mu = 0.0; mu <= 5.0; mu += 0.1)
        {
            for (double sigma = 0.1; sigma <= 2.0; sigma += 0.1)
            {
                double likelihood = 0.0;
                for (int subgroup1_groups = 1; subgroup1_groups < total_observed_groups; ++subgroup1_groups)
                {
                    double bin_prob = binomial_pmf(total_robots, subgroup1_groups, p);                    // 计算 subgroup_1 组的概率
                    double log_prob = lognormal_pdf(total_observed_groups - subgroup1_groups, mu, sigma); // 计算 subgroup_2 组的概率
                    likelihood += bin_prob * log_prob;
                }

                if (likelihood > best_prob)
                {
                    best_prob = likelihood;
                    best_p = p;
                    best_mu = mu;
                    best_sigma = sigma;
                }
            }
        }
    }
    return std::make_tuple(best_p, best_mu, best_sigma);
}

// 计算最优 r，使得 mu 和 sigma 最匹配 best_mu 和 best_sigma
double fissionFusion::estimate_range(double best_mu, double best_sigma, int num_robots, double arena_area)
{
    double best_r = 0.0;
    double min_error = std::numeric_limits<double>::max();

    for (double r = 0.01; r <= 1.0; r += 0.001)
    {
        double mu = 0.3448 + 0.3701 * std::log(1.0 / r) - 0.0872 * std::log(1.0 / arena_area) + 0.1831 * std::log(num_robots);
        double sigma = 0.1741 - 0.0322 * std::log(1.0 / r) + 0.0067 * std::log(1.0 / arena_area) + 0.0563 * std::log(num_robots);

        double error = std::abs(mu - best_mu) + std::abs(sigma - best_sigma);

        if (error < min_error)
        {
            min_error = error;
            best_r = r;
        }
    }
    return best_r;
}

std::pair<double, double> fissionFusion::sffm_estimate_posibility_range(double expected_subgroupsize,
                                                                        double arena_area,
                                                                        double nums_robots)
{

    double estimate_posibility = 0;

    double best_mu = 0.0, best_sigma = 0.0;

    double nums_group = nums_group / expected_subgroupsize;

    std::make_tuple(estimate_posibility, best_mu, best_sigma) = maximum_likelihood_estimation(nums_robots, nums_group);

    double estimate_follow_range = estimate_range(best_mu, best_sigma, nums_robots, arena_area);

    // 用后删除
    estimate_posibility = desired_subgroup_size / nums_robots;
    estimate_follow_range = follow_range;

    return std::make_pair(estimate_posibility, estimate_follow_range);
}

std::pair<double, double> fissionFusion::pd_control_to_target(geometry_msgs::msg::TransformStamped pose)
{

    // 目标在机器人坐标系下的位置（base_link → target）
    double dx = pose.transform.translation.x;
    double dy = pose.transform.translation.y;

    double distance = std::sqrt(dx * dx + dy * dy);
    double angle_to_target = std::atan2(dy, dx); // 因为机器人面朝 x 轴方向

    // 假设当前机器人朝向为 0，则：
    double angle_error = angle_to_target;

    // 归一化角度 [-pi, pi]
    if (angle_error > M_PI)
        angle_error -= 2 * M_PI;
    else if (angle_error < -M_PI)
        angle_error += 2 * M_PI;

    rclcpp::Time now = this->get_clock()->now();
    control_loop_duration = (now - pd_control_last_time).seconds();
    // control_loop_duration = 0.01;
    const double min_dt = 1e-3;
    if (control_loop_duration < min_dt)
    {
        control_loop_duration = min_dt;
    }
    pd_control_last_time = now;

    // 控制器参数
    double dt = control_loop_duration;

    double distance_error_rate = (distance - prev_distance_error) / dt;
    double angle_error_rate = (angle_error - prev_angle_error) / dt;

    double v = Kp_distance * distance + Kd_distance * distance_error_rate;
    double omega = Kp_angle * angle_error + Kd_angle * angle_error_rate;

    // 转角优先策略
    if (std::fabs(angle_error) > M_PI / 4.0)
    {
        v = 0.0; // 转向优先，暂停前进
    }

    // 停止条件
    if (distance < 0.05)
    {
        v = 0.0;
        omega = 0.0;
    }

    // 限速处理
    v = std::clamp(v, -max_velocity, max_velocity);
    omega = std::clamp(omega, -max_omega, max_omega);

    prev_distance_error = distance;
    prev_angle_error = angle_error;

    return {v, omega};
}

std::pair<double, double> fissionFusion::random_walk(double mean_v,
                                                     double std_v,
                                                     double mean_omega,
                                                     double std_omega)
{
    // static generator 保证只初始化一次
    static std::random_device rd;
    static std::mt19937 gen(rd());

    // 分布用传入的参数
    std::normal_distribution<double> dist_v(mean_v, std_v);
    std::normal_distribution<double> dist_omega(mean_omega, std_omega);

    // 采样并 clamp
    double v = std::max(0.0, dist_v(gen));
    double omega = dist_omega(gen);

    return {v, omega};
}

geometry_msgs::msg::TransformStamped fissionFusion::computeGLJTarget(double stable_distance, double force)
{
    const int max_neighbors = 5; // K
    double sigma = stable_distance, epsilon = 0.3;
    const double n = 4.0, m = 2.0;
    double effective_range = follow_range;
    double max_force = force;

    double fx_total = 0.0, fy_total = 0.0;

    if (fission_sign == 1)
    {
        std::cout << "+++ new group" << std::endl;
        sigma = 0.5;
        epsilon = 1.0;
        effective_range = 10.0 * sigma;

        const auto &rabData = rab_data.data;
        for (size_t i = 0; i + frame_length <= rabData.size(); i += frame_length)
        {
            if (rabData[i + frame_length - 1] != 1)
                continue; // 只看 fissioner

            int id_number = static_cast<int>(std::round(rabData[i]));
            std::string child_frame_id = "bot" + std::to_string(id_number) + "/base_link";

            int __id = -1;
            if (!parse_ns_id_fast(child_frame_id, __id))
                continue;

            auto it = tf_index_by_id.find(__id);
            if (it == tf_index_by_id.end())
                continue;

            const auto &e = tf_entries[it->second];

            const double d2 = e.d2;
            if (d2 < 1e-10)
                continue;
            if (effective_range > 0 && d2 >= effective_range * effective_range)
                continue;

            const double d = std::sqrt(d2);
            double f_mag;
            if (n == 4.0 && m == 2.0)
            {
                const double inv_d = 1.0 / d;
                const double inv_d2 = inv_d * inv_d;
                const double inv_d4 = inv_d2 * inv_d2;
                const double sigma2 = sigma * sigma;
                const double sigma4 = sigma2 * sigma2;
                f_mag = -epsilon * (sigma4 * inv_d4 * inv_d - sigma2 * inv_d2 * inv_d);
            }
            else
            {
                f_mag = -epsilon * (std::pow(sigma / d, n) - std::pow(sigma / d, m)) / d;
            }
            fx_total += f_mag * (e.dx / d);
            fy_total += f_mag * (e.dy / d);
        }
    }
    else
    {
        struct Node
        {
            double d2, dx, dy;
        };
        const int K = std::max(0, max_neighbors);
        std::vector<Node> heap;
        heap.reserve(K);

        for (const auto &e : tf_entries)
        {
            if (e.d2 < 1e-8)
                continue;
            if (effective_range > 0 && e.d2 >= effective_range * effective_range)
                continue;
            if (e.is_on_gray)
                continue;

            if ((int)heap.size() < K)
            {
                heap.push_back({e.d2, e.dx, e.dy});
                if ((int)heap.size() == K)
                    std::make_heap(heap.begin(), heap.end(),
                                   [](const Node &a, const Node &b)
                                   { return a.d2 < b.d2; });
            }
            else if (K > 0 && e.d2 < heap.front().d2)
            {
                std::pop_heap(heap.begin(), heap.end(),
                              [](const Node &a, const Node &b)
                              { return a.d2 < b.d2; });
                heap.back() = {e.d2, e.dx, e.dy};
                std::push_heap(heap.begin(), heap.end(),
                               [](const Node &a, const Node &b)
                               { return a.d2 < b.d2; });
            }
        }

        if (n == 4.0 && m == 2.0)
        {
            const double sigma2 = sigma * sigma;
            const double sigma4 = sigma2 * sigma2;
            for (const auto &node : heap)
            {
                const double d = std::sqrt(node.d2);
                const double inv_d = 1.0 / d;
                const double inv_d2 = inv_d * inv_d;
                const double inv_d4 = inv_d2 * inv_d2;
                const double f_mag = -epsilon * (sigma4 * inv_d4 * inv_d - sigma2 * inv_d2 * inv_d);
                fx_total += f_mag * (node.dx * inv_d);
                fy_total += f_mag * (node.dy * inv_d);
            }
        }
        else
        {
            for (const auto &node : heap)
            {
                const double d = std::sqrt(node.d2);
                const double f_mag = -epsilon * (std::pow(sigma / d, n) - std::pow(sigma / d, m)) / d;
                fx_total += f_mag * (node.dx / d);
                fy_total += f_mag * (node.dy / d);
            }
        }
    }

    const double mag = std::hypot(fx_total, fy_total);
    if (mag > max_force)
    {
        const double s = max_force / mag;
        fx_total *= s;
        fy_total *= s;
    }
    if (mag < 1e-3)
    {
        fx_total = 0.0;
        fy_total = 0.0;
    }

    geometry_msgs::msg::TransformStamped GLJTarget;
    GLJTarget.header.stamp = this->get_clock()->now();
    GLJTarget.header.frame_id = "base_link";
    GLJTarget.transform.translation.x = fx_total;
    GLJTarget.transform.translation.y = fy_total;
    GLJTarget.transform.translation.z = 0.0;
    GLJTarget.transform.rotation.w = 1.0;
    return GLJTarget;
}

geometry_msgs::msg::TransformStamped fissionFusion::computeGroupGLJTarget()
{
    const double A = 0.8;                    // 力的最大幅值
    const double r0 = 1.8 * follow_range;    // 理想距离（平衡点）
    const double max_force = 1.0;            // 限制最大力
    const double effective_range = 2.0 * r0; // sin 函数有效范围
    const double min_distance = 1e-4;        // 防止除以 0

    double sum_x = 0.0;
    double sum_y = 0.0;
    int valid_count = 0;

    std::vector<int> remove_id_list;

    for (int id_number : original_group_child_frame_id_list)
    {

        auto rab_data_frame = Extract_Rab_Data(id_number);

        bool id_fission_sign = rab_data_frame[3];

        if (id_fission_sign == 1)
        {
            remove_id_list.push_back(id_number);
            continue;
        }

        std::string child_frame_id = "bot" + std::to_string(id_number) + "/base_link";

        for (const auto &tf : rab_tf.transforms)
        {
            if (tf.child_frame_id == child_frame_id)
            {
                sum_x += tf.transform.translation.x;
                sum_y += tf.transform.translation.y;
                valid_count++;
                break;
            }
        }
    }

    for (int remove_id : remove_id_list)
    {
        original_group_child_frame_id_list.erase(
            std::remove(original_group_child_frame_id_list.begin(), original_group_child_frame_id_list.end(), remove_id),
            original_group_child_frame_id_list.end());
    }

    double fx_total = 0.0;
    double fy_total = 0.0;

    if (valid_count > 0)
    {
        double center_x = sum_x / valid_count;
        double center_y = sum_y / valid_count;
        double d = std::hypot(center_x, center_y);

        if (d > min_distance && d <= effective_range)
        {
            double force = -A * std::sin(M_PI * d / r0); // 你的指定形式 f(r) = A·sin(π·r/r₀)

            fx_total = force * center_x / d;
            fy_total = force * center_y / d;
        }
    }

    // 限制最大力
    double mag = std::hypot(fx_total, fy_total);
    if (mag > max_force)
    {
        fx_total = fx_total / mag * max_force;
        fy_total = fy_total / mag * max_force;
    }

    // 抖动过滤
    if (mag < 0.001)
    {
        fx_total = 0.0;
        fy_total = 0.0;
    }

    // 构造消息输出
    geometry_msgs::msg::TransformStamped GroupSinForceTarget;
    GroupSinForceTarget.header.stamp = rclcpp::Clock().now();
    GroupSinForceTarget.header.frame_id = "base_link";
    GroupSinForceTarget.transform.translation.x = fx_total;
    GroupSinForceTarget.transform.translation.y = fy_total;
    GroupSinForceTarget.transform.translation.z = 0.0;
    GroupSinForceTarget.transform.rotation.x = 0.0;
    GroupSinForceTarget.transform.rotation.y = 0.0;
    GroupSinForceTarget.transform.rotation.z = 0.0;
    GroupSinForceTarget.transform.rotation.w = 1.0;

    return GroupSinForceTarget;
}

std::vector<double> fissionFusion::Extract_Rab_Data(int id_number)
{
    std::vector<double> out;

    // 先用零拷贝索引
    auto it = rab_offset.find(id_number);
    if (it != rab_offset.end())
    {
        const auto &d = rab_data.data;
        size_t off = it->second;
        if (off + rab_stride <= d.size())
        {
            out.reserve(rab_stride);
            out.insert(out.end(), d.begin() + off, d.begin() + off + rab_stride);
            return out;
        }
    }

    // 回退：保持旧行为（线性扫）
    const auto &d = rab_data.data;
    if (frame_length < 2)
        return out;
    for (size_t i = 0; i + frame_length <= d.size(); i += frame_length)
    {
        int id = static_cast<int>(std::round(d[i]));
        if (id == id_number)
        {
            out.reserve(frame_length);
            for (int j = 0; j < frame_length; ++j)
                out.push_back(d[i + j]);
            return out;
        }
    }
    return out;
}

void fissionFusion::refresh_target_transform()
{
    static int lost_count = 0;

    if (target_transform.header.frame_id == "none" ||
        target_transform.header.frame_id == "non-follower" ||
        target_transform.header.frame_id.empty())
    {
        lost_count = 0;
        return;
    }

    int __id = -1;
    if (parse_ns_id_fast(target_transform.child_frame_id, __id))
    {
        auto it = tf_index_by_id.find(__id);
        if (it != tf_index_by_id.end())
        {
            target_transform = *tf_entries[it->second].tf;
            lost_count = 0; // 找到了，重置丢失计数
            return;
        }
    }

    // 未找到：累积丢失
    if (++lost_count >= 10)
    {
        target_transform.header.frame_id = "none";
        lost_count = 0;
        std::cout << "lost target! re-search" << std::endl;
    }
}

void fissionFusion::Pub_rab()
{
    int number_id = -1;
    if (!parse_ns_id_fast(current_namespace, number_id))
    {
        std::cerr << "Failed to extract numeric ID from namespace: " << current_namespace << std::endl;
        number_id = -1;
    }

    if (current_state == FISSION)
    {
        fission_sign = (groupsize_tolerance > 0) ? 1.0 : 0.0;
    }
    else
    {
        fission_sign = 0.0;
    }

    std_msgs::msg::Float64MultiArray rab_actuator;
    rab_actuator.data.push_back(number_id);
    rab_actuator.data.push_back(std::round(estimated_group_size));
    rab_actuator.data.push_back(desired_subgroup_size);
    rab_actuator.data.push_back(fission_sign);

    frame_length = rab_actuator.data.size();
    rab_actuator_publisher_->publish(rab_actuator);
}

double fissionFusion::smoothed_estimate_with_window(double new_estimate)
{
    // 静态变量用于跨调用保持历史状态
    static std::deque<double> history;
    const size_t W = 2;       // 滑动窗口大小
    const double decay = 0.9; // 时间衰减因子（0.8~0.95 建议）

    // 添加新估计值
    if (history.size() >= W)
        history.pop_front(); // 移除最旧
    history.push_back(new_estimate);

    // 计算加权平均
    double weighted_sum = 0.0, weight_total = 0.0;
    for (size_t i = 0; i < history.size(); ++i)
    {
        // 越新的估计越大权重（最后一个是最大）
        double weight = std::pow(decay, history.size() - i - 1);
        weighted_sum += weight * history[i];
        weight_total += weight;
    }

    return weighted_sum / weight_total;
}

void fissionFusion::safe_publish_trigger()
{
    if (!trigger_signal_publisher_)
    {
        RCLCPP_ERROR(this->get_logger(), "[safe_publish_trigger] Publisher not initialized.");
        return;
    }

    const int max_attempts = 50;
    int attempts = 0;

    // 等待至少一个订阅者连接（最多等待5秒）
    while (trigger_signal_publisher_->get_subscription_count() == 0 && rclcpp::ok() && attempts < max_attempts)
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for subscriber to connect to /trigger_signal...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        attempts++;
    }

    if (trigger_signal_publisher_->get_subscription_count() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "No subscribers connected to /trigger_signal after waiting.");
        return;
    }

    std_msgs::msg::Header trigger;
    trigger.frame_id = "trigger_signal";
    trigger.stamp = this->get_clock()->now();

    try
    {
        trigger_signal_publisher_->publish(trigger);

        {
            std::lock_guard<std::mutex> lock(trigger_list_mutex_);
            trigger_list_.push_back(trigger);

            if (trigger_list_.size() > 10)
                trigger_list_.pop_front();
        }

        RCLCPP_DEBUG(this->get_logger(), "Published /trigger_signal at %.9f",
                     trigger.stamp.sec + trigger.stamp.nanosec * 1e-9);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish /trigger_signal: %s", e.what());
    }
}

bool fissionFusion::parse_ns_id_fast(const std::string &ns, int &id)
{
    // 解析 "/bot123" 或 "bot123" → 123
    size_t b = ns.find("bot");
    if (b == std::string::npos)
        return false;
    size_t s = b + 3, e = s;
    while (e < ns.size() && std::isdigit(static_cast<unsigned char>(ns[e])))
        ++e;
    if (e == s)
        return false;
    id = std::stoi(ns.substr(s, e - s));
    return true;
}

void fissionFusion::build_frame_indices()
{
    // 0) 定义 rab 帧布局索引（根据你的实际数据改）
    constexpr size_t RAB_IDX_ID = 0;      // id
    constexpr size_t RAB_IDX_UNUSED1 = 1; // 占位/其他
    constexpr size_t RAB_IDX_UNUSED2 = 2; // 占位/其他
    constexpr size_t RAB_IDX_ON_GRAY = 3; // 1.0 表示 is_on_gray
    constexpr size_t RAB_IDX_IS_FULL = 4; // 1.0 表示 is_full
    constexpr size_t RAB_MIN_FIELDS = 5;  // 读到 is_full 至少需要 5 个槽位

    // 1) rab_offset：建立 id -> 起始偏移
    rab_offset.clear();
    rab_stride = frame_length;

    const auto &d = rab_data.data;
    if (rab_stride >= 2 && d.size() >= static_cast<size_t>(rab_stride))
    {
        rab_offset.reserve(d.size() / rab_stride);
        for (size_t i = 0; i + rab_stride <= d.size(); i += rab_stride)
        {
            int id = static_cast<int>(std::round(d[i + RAB_IDX_ID]));
            rab_offset.emplace(id, i);
        }
    }

    // 2) TF 条目 + 按 id 的索引
    tf_entries.clear();
    tf_entries.reserve(rab_tf.transforms.size());

    tf_index_by_id.clear();
    tf_index_by_id.reserve(rab_tf.transforms.size());

    for (size_t idx = 0; idx < rab_tf.transforms.size(); ++idx)
    {
        const auto &tf = rab_tf.transforms[idx];

        int id = -1;
        parse_ns_id_fast(tf.child_frame_id, id);

        bool fiss = false;    // is_on_gray
        bool is_full = false; // 新字段

        if (id != -1)
        {
            auto it = rab_offset.find(id);
            if (it != rab_offset.end())
            {
                size_t off = it->second;
                // 需要至少 RAB_MIN_FIELDS 个槽位才能同时读取 on_gray 和 is_full
                if (off + RAB_MIN_FIELDS <= d.size())
                {
                    fiss = (d[off + RAB_IDX_ON_GRAY] == 1.0);
                    is_full = (d[off + RAB_IDX_IS_FULL] == 1.0);
                }
                else if (off + (RAB_IDX_ON_GRAY + 1) <= d.size())
                {
                    // 兼容旧包：若数据还没携带 is_full，就只读 on_gray
                    fiss = (d[off + RAB_IDX_ON_GRAY] == 1.0);
                }
            }
        }

        const double dx = tf.transform.translation.x;
        const double dy = tf.transform.translation.y;
        const double d2 = dx * dx + dy * dy;

        // 注意聚合初始化的顺序要与 TfEntry 声明一致
        tf_entries.push_back(TfEntry{&tf, id, fiss, dx, dy, d2, is_full});

        if (id != -1)
        {
            // 同一 id 多条 TF 时，这里保留第一条；如需最近，可比较 d2 再覆盖
            tf_index_by_id.emplace(id, idx);
        }
    }
}

const double *fissionFusion::get_rab_ptr(int id) const
{
    auto it = rab_offset.find(id);
    if (it == rab_offset.end())
        return nullptr;
    const auto &d = rab_data.data;
    size_t off = it->second;
    if (off + rab_stride > d.size())
        return nullptr;
    return d.data() + off; // 长度为 rab_stride
}
