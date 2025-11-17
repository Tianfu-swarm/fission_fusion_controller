#include "system_init.h"
#include <filesystem>

void fissionFusion::convergence_controller_step()
{
    // std::ofstream null_stream("/dev/null");      // Linux/Unix
    // std::streambuf *old_buf = std::cout.rdbuf(); // 保存原来的缓冲区
    // std::cout.rdbuf(null_stream.rdbuf());        // 重定向到空
    // 恢复
    // std::cout.rdbuf(old_buf);

    // boot time
    if ((this->get_clock()->now() - boot_time) < boot_wait_time)
    {
        current_state = RANDOM_WALK;
        // execute_state_behavior(current_state);
        srand(static_cast<unsigned int>(this->get_clock()->now().nanoseconds())); // 初始化随机种子

        int rand_int = rand() % 100;

        // 计算 jitter
        jitter_time = rand_int * 0.01;

        int id;
        if (parse_ns_id_fast(current_namespace, id))
        {
            if (id >= 0 && id < 3)
            {
                desired_subgroup_size = 3;
            }
            else if (id >= 3 && id < 7)
            {
                desired_subgroup_size = 4;
            }
            else if (id >= 7 && id < 12)
            {
                desired_subgroup_size = 5;
            }
        }

        safe_publish_trigger();

        return;
    }

    // pub rab
    Pub_rab();

    // 建索引（每帧一次）
    build_frame_indices();

    // estimate size
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

    double actual_group_size = std::round(estimated_group_size);
    write_buffer << current_namespace << ","
                 << std::fixed << this->get_clock()->now().seconds() << ","
                 << estimated_group_size << ","
                 << actual_group_size << ","
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

    current_state = update_state_convergence(current_state);

    execute_state_behavior_convergence(current_state);

    safe_publish_trigger();
}

fissionFusion::robot_state fissionFusion::update_state_convergence(robot_state current_robot_state)
{
    switch (current_robot_state)
    {
    case RANDOM_WALK:
    {
        auto follow_result = sffm_estimate_posibility_range(desired_subgroup_size, arena_area, n_groupsize);
        double follow_posibility = follow_result.first;
        double follow_radius = follow_result.second;

        static bool first_desion = true;
        if (first_desion)
        {
            first_desion = false;
        }
        else
        {
            follow_posibility = 1;
            follow_radius = follow_range;
        }

        sffm_choose_follow_target(follow_posibility, follow_radius);

        if (target_transform.header.frame_id == "none")
            return RANDOM_WALK;
        if (target_transform.header.frame_id == "non-follower")
        {
            initial_group_size = 1;
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
            std::cout << "from random to stay" << std::endl;
            return STAY;
        }

        std::cout << "from random to FUSION" << std::endl;
        return FUSION;
    }

    case FUSION:
    {
        if (target_transform.header.frame_id == "none" ||
            target_transform.header.frame_id == "non-follower" ||
            target_transform.header.frame_id.empty())
        {
            return RANDOM_WALK;
        }

        double dx = target_transform.transform.translation.x;
        double dy = target_transform.transform.translation.y;
        double delta_distance = std::sqrt(dx * dx + dy * dy);

        if (delta_distance < 0.5)
        {
            initial_group_size = 2;
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
            target_transform.child_frame_id.clear();
            std::cout << "from FUSION to SATY" << std::endl;
            return STAY;
        }

        if (isConCommunication)
        {
            // 用 parse_ns_id_fast + rab_index
            target_id = -1;
            if (!parse_ns_id_fast(target_transform.child_frame_id, target_id))
            {
                std::cout << "not get target id, return random walk" << std::endl;
                return RANDOM_WALK;
            }

            double target_group_size = -1;
            const double *rab = get_rab_ptr(target_id);
            if (rab && rab[1] != -1)
            {
                target_group_size = rab[1];
            }
            else
            {
                std::cout << "extra target group size failed, keep fusion" << std::endl;
                return FUSION;
            }

            if (target_group_size < (desired_subgroup_size + groupsize_tolerance))
            {
                return FUSION;
            }
            else
            {
                std::cout << "「ConCommunication」target group size is larger, return random walk, target_group_size: "
                          << target_group_size
                          << " larger than desired_subgroup_size + groupsize_tolerance is "
                          << (desired_subgroup_size + groupsize_tolerance) << std::endl;
                return RANDOM_WALK;
            }
        }

        return FUSION;
    }

    case FISSION:
    {
        // ✅ 改：使用 tf_entries 寻最近 & tf_index 更新
        if (fission_transform.child_frame_id.empty())
        {
            std::cout << "choose fission target" << std::endl;
            if (tf_entries.empty())
            {
                std::cout << "rab no message" << std::endl;
                return FISSION;
            }

            // 最近邻（按 d²）
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
            if (nearest)
            {
                fission_transform.child_frame_id = nearest->tf->child_frame_id;
                std::cout << "fission target distance is: " << std::sqrt(best_d2) << std::endl;
            }
            else
            {
                return FISSION;
            }
        }

        bool found = false;
        {
            int __fid = -1;
            if (parse_ns_id_fast(fission_transform.child_frame_id, __fid))
            {
                auto it = tf_index_by_id.find(__fid);
                if (it != tf_index_by_id.end())
                {
                    fission_transform = *tf_entries[it->second].tf;
                    found = true;
                }
            }
        }

        if (!found)
        {
            static int lost_target_time = 0;
            if (++lost_target_time < 5)
                return FISSION;
            lost_target_time = 0;
            std::cout << "lost fission target, return random " << fission_transform.child_frame_id << std::endl;
            fission_transform.header.frame_id.clear();
            return RANDOM_WALK;
        }

        const double x = fission_transform.transform.translation.x;
        const double y = fission_transform.transform.translation.y;
        const double distance = std::sqrt(x * x + y * y);

        if (distance > 5)
        {
            std::cout << "from fission to random walk, far to target: " << distance << std::endl;
            fission_transform.child_frame_id.clear();
            return RANDOM_WALK;
        }
        else
        {
            return FISSION;
        }
    }

    case STAY:
    {
        double actual_group_size = std::round(estimated_group_size);
        if (this->get_clock()->now() - Maintain_state_start_time < rclcpp::Duration::from_seconds(2.0))
        {
            target_transform.child_frame_id.clear();
            return STAY;
        }

        if (actual_group_size < desired_subgroup_size + groupsize_tolerance)
        {
            if (actual_group_size != initial_group_size)
            {
                if (actual_group_size > initial_group_size)
                {
                    initial_group_size = actual_group_size;
                    wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
                    stay_start_time = this->get_clock()->now();
                }
                else
                {
                    rclcpp::Time now = this->get_clock()->now();
                    rclcpp::Duration elapsed = now - stay_start_time;
                    rclcpp::Duration remaining = wait_time - elapsed;

                    rclcpp::Duration new_wait = rclcpp::Duration::from_seconds(
                        static_cast<double>(Waiting_time_scale_factor * actual_group_size));

                    if (new_wait < remaining)
                    {
                        wait_time = new_wait;
                        stay_start_time = now;
                    }
                    initial_group_size = actual_group_size;
                }
            }

            rclcpp::Time time_now = this->get_clock()->now();
            if ((time_now - stay_start_time).seconds() > wait_time.seconds())
            {
                std::cout << "\033[1;32m" << "[Stay]->[Fission]" << "\033[0m"
                          << " Waiting time Run out,"
                          << "start time = " << stay_start_time.seconds()
                          << ", wait time = " << wait_time.seconds()
                          << " initial_group_size:" << initial_group_size
                          << std::endl;
                fission_transform.child_frame_id.clear();
                return FISSION;
            }
            else
            {
                target_transform.child_frame_id.clear();
                return STAY;
            }
        }
        else if (actual_group_size > desired_subgroup_size + groupsize_tolerance)
        {
            auto follow_result = sffm_estimate_posibility_range(desired_subgroup_size, arena_area, actual_group_size);
            double follow_posibility = 1 - ((actual_group_size - desired_subgroup_size) / actual_group_size);
            double follow_radius = 2;

            sffm_choose_follow_target(follow_posibility, follow_radius);
            if (target_transform.header.frame_id == "non-follower")
            {
                std::cout << "[larger] from stay to fission, larger than desired size: " << actual_group_size << std::endl;
                fission_transform.child_frame_id.clear();
                return FISSION;
            }
            else
            {
                target_transform.child_frame_id.clear();
                Maintain_state_start_time = this->get_clock()->now();
                std::cout << "from stay to stay, size: " << estimated_group_size << ", time: " << this->get_clock()->now().seconds() << std::endl;
                return STAY;
            }
        }
        else
        {
            target_transform.child_frame_id.clear();
            return STAY;
        }
    }

    default:
        return RANDOM_WALK;
    }
}

void fissionFusion::execute_state_behavior_convergence(robot_state state)
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
            twist_msg.linear.x = results.first / 2;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = results.second / 2;
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
            twist_msg.linear.x = results.first / 2;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = results.second / 2;
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

    case STAY:
    {
        target_transform.header.frame_id.clear();

        std::pair<double, double> control_command;

        geometry_msgs::msg::TransformStamped GLJTarget = computeGLJTarget();

        control_command = pd_control_to_target(GLJTarget);

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