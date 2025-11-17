#include "system_init.h"
#include <filesystem>

void fissionFusion::cps_task_step()
{
    // std::ofstream null_stream("/dev/null");      // Linux/Unix
    // std::streambuf *old_buf = std::cout.rdbuf(); // 保存原来的缓冲区
    // std::cout.rdbuf(null_stream.rdbuf());        // 重定向到空
    // 恢复
    // std::cout.rdbuf(old_buf);
    // 在类成员或者文件作用域保存原始缓冲区
    // std::streambuf *default_cout_buf = std::cout.rdbuf();
    // static std::ofstream null_stream("/dev/null"); // 永远开着，避免悬空

    // // 在逻辑里切换
    // if (current_namespace == "/bot0")
    // {
    //     // bot0 打印正常
    //     std::cout.rdbuf(default_cout_buf);
    // }
    // else
    // {
    //     // 其他机器人静音
    //     std::cout.rdbuf(null_stream.rdbuf());
    // }

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

    // pub rab
    Pub_rab_cps();

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

    current_state = update_state_cps(current_state);

    execute_state_behavior_cps(current_state);

    update_desired_subgroup_size_from_gray();

    safe_publish_trigger();
}

void fissionFusion::execute_state_behavior_cps(robot_state state)
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

        geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning(0.6);

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
        fissionFusion::refresh_target_transform();

        std::pair<double, double> control_command{0.0, 0.0};

        // 无有效目标：直接停
        if (target_transform.header.frame_id == "none" ||
            target_transform.header.frame_id == "non-follower" ||
            target_transform.header.frame_id.empty())
        {
            control_command.first = 0.0;
            control_command.second = 0.0;
        }
        else
        {
            // 与目标的相对距离（base_link）
            const double tx = target_transform.transform.translation.x;
            const double ty = target_transform.transform.translation.y;
            const double dist = std::hypot(tx, ty);

            if (dist <= 2.0)
            {
                // 近距：只用目标点
                control_command = pd_control_to_target(target_transform);
            }
            else
            {
                // 远距：确定性的固定点加权
                // 固定权重（可按需调整）：更相信本地规划
                constexpr double ALPHA_LOCAL = 0.7; // Local 占 70%
                constexpr double ALPHA_DIR = 1.0 - ALPHA_LOCAL;

                geometry_msgs::msg::TransformStamped LocalTarget = local_path_planning(0.6);

                bool local_valid = !(LocalTarget.child_frame_id.empty() || LocalTarget.child_frame_id == "none");

                geometry_msgs::msg::TransformStamped CombinedTarget;
                CombinedTarget.header.stamp = rclcpp::Clock().now();
                CombinedTarget.header.frame_id = "base_link";
                CombinedTarget.child_frame_id = "combined_target";

                if (local_valid)
                {
                    const double lx = LocalTarget.transform.translation.x;
                    const double ly = LocalTarget.transform.translation.y;

                    // 平移点加权： (1-α)*Direct + α*Local
                    CombinedTarget.transform.translation.x = ALPHA_DIR * tx + ALPHA_LOCAL * lx;
                    CombinedTarget.transform.translation.y = ALPHA_DIR * ty + ALPHA_LOCAL * ly;
                }
                else
                {
                    // 本地不可用则退化为直接目标
                    CombinedTarget.transform.translation.x = tx;
                    CombinedTarget.transform.translation.y = ty;
                }

                CombinedTarget.transform.translation.z = 0.0;
                // 旋转单位四元数（与 STAY 一致）
                CombinedTarget.transform.rotation.x = 0.0;
                CombinedTarget.transform.rotation.y = 0.0;
                CombinedTarget.transform.rotation.z = 0.0;
                CombinedTarget.transform.rotation.w = 1.0;

                // 对加权“点”做 PD
                control_command = pd_control_to_target(CombinedTarget);
            }
        }

        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = control_command.first;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = control_command.second;

        if (!isAbstacle)
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

        geometry_msgs::msg::TransformStamped local_planning_target = local_path_planning(0.6);

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

        const bool has_gray = std::any_of(motor_ground_data.data.begin(), motor_ground_data.data.end(),
                                          [](double x)
                                          { return x < 1.0 - 1e-6; });

        if (!has_gray)
        {

            // 全白：Spring + LookGray 加权
            geometry_msgs::msg::TransformStamped SpringTarget = computeSpringTarget(spring_distance, spring_distance / 2); // 保持与邻居距离
            geometry_msgs::msg::TransformStamped LookGrayTarget = look_for_gray(0.3);                                      // 沿包络/圆搜灰色

            double w_spring = 0.0; // ← 可调：对齐/疏密的权重
            double w_look = 0.9;   // ← 可调：搜索黑色的权重

            geometry_msgs::msg::TransformStamped CombinedTarget;
            CombinedTarget.header.stamp = rclcpp::Clock().now();
            CombinedTarget.header.frame_id = "base_link";
            CombinedTarget.child_frame_id = "combined_target";

            // 平移加权
            double gx = SpringTarget.transform.translation.x;
            double gy = SpringTarget.transform.translation.y;
            double lx = LookGrayTarget.transform.translation.x;
            double ly = LookGrayTarget.transform.translation.y;

            double tx = w_spring * gx + w_look * lx;
            double ty = w_spring * gy + w_look * ly;

            CombinedTarget.transform.translation.x = tx;
            CombinedTarget.transform.translation.y = ty;
            CombinedTarget.transform.translation.z = 0.0;

            // 旋转设为单位四元数
            CombinedTarget.transform.rotation.x = 0.0;
            CombinedTarget.transform.rotation.y = 0.0;
            CombinedTarget.transform.rotation.z = 0.0;
            CombinedTarget.transform.rotation.w = 1.0;

            control_command = pd_control_to_target(CombinedTarget);
        }
        else
        {
            geometry_msgs::msg::TransformStamped SpringTarget =
                computeSpringTarget(spring_distance, spring_distance / 2); // keep distance with neighbor
            geometry_msgs::msg::TransformStamped GrayTarget =
                stay_in_gray(); // stay on gray

            // 判断 gray 是否有效：child_frame_id != "none"
            const bool use_gray = !(GrayTarget.child_frame_id.empty() ||
                                    GrayTarget.child_frame_id == "none");

            // 选择目标（不再加权合并）
            const geometry_msgs::msg::TransformStamped &ChosenTarget =
                use_gray ? GrayTarget : SpringTarget;

            // 下发控制（直接用选中的目标）
            control_command = pd_control_to_target(ChosenTarget);
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

fissionFusion::robot_state fissionFusion::update_state_cps(robot_state current_robot_state)
{
    switch (current_robot_state)
    {
    case RANDOM_WALK:
    {
        auto follow_result = sffm_estimate_posibility_range(desired_subgroup_size, arena_area, n_groupsize);
        double follow_posibility = follow_result.first;
        double follow_radius = follow_result.second;

        if (std::any_of(motor_ground_data.data.begin(), motor_ground_data.data.end(),
                        [](double x)
                        { return x < 1.0 - 1e-6; })) // < 1 认为有灰度
        {
            std::cout << "arrived gray, return stay" << std::endl;
            initial_group_size = 1;
            stay_start_time = this->get_clock()->now();
            wait_time = rclcpp::Duration::from_seconds(static_cast<double>(Waiting_time_scale_factor * initial_group_size));
            return STAY; // 到达灰度（非白）
        }

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

        cps_task_choose_follow_target(follow_posibility, follow_radius);

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
            // ✅ 改：用 parse_ns_id_fast + rab_index
            target_id = -1;
            if (!parse_ns_id_fast(target_transform.child_frame_id, target_id))
            {
                std::cout << "not get target id, return random walk" << std::endl;
                return RANDOM_WALK;
            }

            double target_group_size = -1;
            double is_on_gray = -1;
            double is_full = -1;
            const double *rab = get_rab_ptr(target_id);
            if (rab && rab[1] != -1 && rab[3] != -1 && rab[4] != -1)
            {
                target_group_size = rab[1];
                is_on_gray = rab[3];
                is_full = rab[4];
            }
            else
            {
                std::cout << "extra target group size failed, keep fusion" << std::endl;
                return FUSION;
            }

            if (target_group_size < (desired_subgroup_size + groupsize_tolerance) && is_on_gray >= 0.5 && is_full <= 0.5)
            { //
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
        // 使用 tf_entries 寻最近 & tf_index 更新
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

        if (distance > 2)
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
        if (this->get_clock()->now() - Maintain_state_start_time < rclcpp::Duration::from_seconds(1.0))
        {
            target_transform.child_frame_id.clear();
            return STAY;
        }

        // if (!is_majority_same_type(2.0))
        // {
        //     std::cout << "fisison cause is not majority" << std::endl;
        //     fission_transform.child_frame_id.clear();
        //     return FISSION;
        // }

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

            cps_task_choose_follow_target(follow_posibility, follow_radius);
            if (target_transform.header.frame_id == "non-follower")
            {
                std::cout << "[larger] from stay to fission, larger than desired size: " << desired_subgroup_size << " current size is: " << estimated_group_size << std::endl;
                fission_transform.child_frame_id.clear();
                return FISSION;
            }
            else
            {
                target_transform.child_frame_id.clear();
                Maintain_state_start_time = this->get_clock()->now();
                std::cout << "from stay to stay,desired size is: " << desired_subgroup_size << " current size: " << estimated_group_size << ", time: " << this->get_clock()->now().seconds() << std::endl;
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

void fissionFusion::cps_task_choose_follow_target(double posibility, double follow_radius)
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

        if (e.d2 > r2)
            continue;

        if (isMinCommunication)
        {
            if (e.id == -1)
                continue;

            const double *frame = get_rab_ptr(e.id);
            if (!frame)
                continue;

            // 规模超容差
            if (frame[1] == -1 || frame[1] >= desired_subgroup_size + groupsize_tolerance)
                continue;

            double is_on_gray = -1;
            is_on_gray = frame[3];
            if (is_on_gray == -1 || is_on_gray < 0.5) // 不在gray上
                continue;

            double is_full = -1;
            is_full = frame[4];
            if (is_full == -1 || is_full > 0.5) // 目标组满了
                continue;
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

bool fissionFusion::is_majority_same_type(double radius)
{
    const double r2 = radius * radius;

    int same = 0;  // 与我 desired_subgroup_size 相同的数量
    int total = 0; // 统计的总数（邻居 + 自己）

    for (const auto &e : tf_entries)
    {
        // 跳过自己
        if (e.tf->child_frame_id == e.tf->header.frame_id)
            continue;
        // 半径过滤
        if (e.d2 > r2)
            continue;

        // 取邻居广播
        if (e.id == -1)
            continue;
        const double *frame = get_rab_ptr(e.id);
        if (!frame)
            continue;

        // frame[2]：邻居的期望子群规模/类型；-1 或 非有限 跳过
        if (!std::isfinite(frame[2]) || frame[2] == -1)
            continue;

        int neighbor_type = static_cast<int>(std::lround(frame[2]));
        total++;
        if (neighbor_type == desired_subgroup_size)
            same++;
    }

    // 自己也算一票（自己的类型即 desired_subgroup_size）
    total++;
    same++;

    // 多数：严格大于一半；若 total==0（极端不可能）返回 false
    return (total > 0) && (same * 2 > total);
}

void fissionFusion::Pub_rab_cps()
{
    int number_id = -1;
    if (!parse_ns_id_fast(current_namespace, number_id))
    {
        std::cerr << "Failed to extract numeric ID from namespace: " << current_namespace << std::endl;
        number_id = -1;
    }

    // 判断 motor_ground_data 是否有灰度
    bool has_gray = false;
    {
        const auto &v = motor_ground_data.data;
        for (double val : v)
        {
            if (val < 1.0 - 1e-6) // 小于1.0即认为有灰
            {
                has_gray = true;
                break;
            }
        }
    }

    // 判断是否在工作且组已满，且有灰度
    double is_on_gray;
    if (current_state == STAY && has_gray)
    {
        is_on_gray = 1.0;
    }
    else
    {
        is_on_gray = 0.0;
    }

    double is_full = 0.0;
    if (std::round(estimated_group_size) >= desired_subgroup_size + groupsize_tolerance)
    {
        is_full = 1.0;
    }
    else
    {
        is_full = 0.0;
    }

    std_msgs::msg::Float64MultiArray rab_actuator;
    rab_actuator.data.push_back(number_id);
    rab_actuator.data.push_back(std::round(estimated_group_size));
    rab_actuator.data.push_back(desired_subgroup_size);
    rab_actuator.data.push_back(is_on_gray);
    rab_actuator.data.push_back(is_full);

    frame_length = rab_actuator.data.size();
    rab_actuator_publisher_->publish(rab_actuator);
}

void fissionFusion::update_desired_subgroup_size_from_gray()
{
    if (motor_ground_data.data.size() >= 4)
    {

        // 计算平均灰度
        double sum = 0.0;
        for (double x : motor_ground_data.data)
            sum += x;
        double avg = sum / motor_ground_data.data.size();

        // 取最小值
        double min_val = std::numeric_limits<double>::infinity();
        for (double x : motor_ground_data.data)
            if (x < min_val)
                min_val = x;

        double gray_value = min_val;
        // 根据平均灰度设置 desired_subgroup_size
        if (gray_value < 0.3)
        {
            desired_subgroup_size = 5;
            spring_distance = 0.6;
        }
        else if (gray_value >= 0.3 && gray_value <= 0.7)
        {
            desired_subgroup_size = 7;
            spring_distance = 0.55;
        }
        else if (gray_value >= 0.7 && gray_value < 1)
        {
            desired_subgroup_size = 9;
            spring_distance = 0.42;
        }
        else
        {
            // 其他情況，如平均值靠近白 (>0.7)，你觉得设置多少比较合适
            desired_subgroup_size = 100;
        }

        // // 打印四个灰度值
        // std::cout << "Motor ground readings: ";
        // for (size_t i = 0; i < motor_ground_data.data.size(); ++i)
        // {
        //     std::cout << motor_ground_data.data[i];
        //     if (i + 1 < motor_ground_data.data.size())
        //         std::cout << ", ";
        // }
        // std::cout << std::endl;

        // std::cout << "gray_value: " << gray_value
        //           << " → desired_subgroup_size = " << desired_subgroup_size
        //           << std::endl;
    }
}

geometry_msgs::msg::TransformStamped fissionFusion::computeSpringTarget(double stable_distance, double force)
{
    // 参数
    const double effective_range = 1.1; // 若 <=0 则不限制
    const double k = 1.0;               // 弹簧系数，可按需要调整
    const double max_force = force;

    // 查找最近邻
    double best_d2 = std::numeric_limits<double>::infinity();
    double best_dx = 0.0, best_dy = 0.0;

    for (const auto &e : tf_entries)
    {
        if (e.d2 < 1e-8)
            continue; // 排除自身或极近
        if (effective_range > 0 && e.d2 >= effective_range * effective_range)
            continue;
        if (e.is_on_gray < 1.0 - 1e-6)
            continue;

        if (e.d2 < best_d2)
        {
            best_d2 = e.d2;
            best_dx = e.dx;
            best_dy = e.dy;
        }
    }

    double fx_total = 0.0, fy_total = 0.0;

    if (std::isfinite(best_d2))
    {
        const double d = std::sqrt(best_d2);
        if (d > 1e-8)
        {
            // Hooke：F = k*(d - d0) * u
            const double u_x = best_dx / d;
            const double u_y = best_dy / d;
            const double f_mag = k * (d - stable_distance);

            fx_total = f_mag * u_x;
            fy_total = f_mag * u_y;
        }
    }

    // 力限幅
    const double mag = std::hypot(fx_total, fy_total);
    if (mag > max_force && mag > 1e-9)
    {
        const double s = max_force / mag;
        fx_total *= s;
        fy_total *= s;
    }

    // 死区
    if (mag < 1e-3)
    {
        fx_total = 0.0;
        fy_total = 0.0;
    }

    // 输出
    geometry_msgs::msg::TransformStamped SpringTarget;
    SpringTarget.header.stamp = this->get_clock()->now();
    SpringTarget.header.frame_id = "base_link";
    SpringTarget.transform.translation.x = fx_total;
    SpringTarget.transform.translation.y = fy_total;
    SpringTarget.transform.translation.z = 0.0;
    SpringTarget.transform.rotation.w = 1.0;
    return SpringTarget;
}
