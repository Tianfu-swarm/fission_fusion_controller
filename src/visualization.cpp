#include "system_init.h"

void fissionFusion::publish_path()
{
    if (current_pose.header.frame_id.empty())
    {
        return;
    }

    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = current_pose.header.frame_id;

    path_msg.poses.push_back(current_pose);

    path_publisher_->publish(path_msg);
}

void fissionFusion::publish_predict_path()
{
    double prediction_time = 3;
    double dt = 0.05;
    // 清空预测路径
    path_predict.poses.clear();
    path_predict.header = current_pose.header;

    // 初始化变量
    geometry_msgs::msg::PoseStamped pose = current_pose;
    double v = current_vel.linear.x;
    double ws = current_vel.angular.z;
    double elapsed_time = 0;

    // 从当前姿态提取初始角度（yaw）
    double yaw = tf2::getYaw(pose.pose.orientation); // 将初始角度设为当前朝向角

    while (elapsed_time < prediction_time)
    {
        double dx = 0, dy = 0, alpha = 0;
        double epsilon = 0.001; // 判断是否直线的阈值

        // 判断是否为直线运动
        if (fabs(ws) < epsilon)
        {
            dx = v * dt;
        }
        else
        {
            // 弧线运动计算
            alpha = angles::normalize_angle(ws * dt); // 角度增量
            double radius = v / ws;                   // 转弯半径
            dx = radius * sin(alpha);
            dy = radius * (1 - cos(alpha));
        }

        // 更新位置
        pose.pose.position.x += cos(yaw) * dx - sin(yaw) * dy;
        pose.pose.position.y += sin(yaw) * dx + cos(yaw) * dy;

        // 更新yaw角
        yaw = angles::normalize_angle(yaw + alpha);

        // 更新pose中的yaw角
        pose.pose.orientation.z = sin(yaw / 2);
        pose.pose.orientation.w = cos(yaw / 2);

        // 添加到路径中
        path_predict.poses.push_back(pose);

        // 更新时间
        elapsed_time += dt;
    }
    path_predict.header.frame_id = "map";
    predict_path_publisher_->publish(path_predict);
}
void fissionFusion::publish_odometry()
{
    odom.header.frame_id = "map";
    odom.pose.pose.position = current_pose.pose.position;
    odom.twist.twist.linear = current_vel.linear;
    odom.twist.twist.angular = current_vel.angular;
    odom_publisher_->publish(odom);
}

void fissionFusion::publish_transformed_follow_relation()
{
    if (target_transform.header.frame_id == "none" ||
        target_transform.header.frame_id == "non-follower" ||
        target_transform.header.frame_id.empty())
    {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        follow_relation_pub_->publish(marker);
        return;
    }

    // 1. 把 current_pose 转成 tf2::Transform
    tf2::Transform tf_current;
    tf2::fromMsg(current_pose.pose, tf_current);

    // 2. 把 target_transform.transform 转成 tf2::Transform
    tf2::Transform tf_target_relative;
    tf2::fromMsg(target_transform.transform, tf_target_relative);

    // 3. 组合变换：目标机器人在 map 下的位置 = 当前机器人的位姿 * 相对变换
    tf2::Transform tf_target_in_map = tf_current * tf_target_relative;

    // 4. 转换为 geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose target_pose;
    tf2::toMsg(tf_target_in_map, target_pose);

    // Create Arrow Marker (from current robot to target robot)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "arrows";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置箭头两端点：从当前机器人指向目标机器人
    geometry_msgs::msg::Point start, end;
    start.x = current_pose.pose.position.x;
    start.y = current_pose.pose.position.y;
    start.z = current_pose.pose.position.z;

    end.x = std::round(target_pose.position.x * 100.0) / 100.0;
    end.y = std::round(target_pose.position.y * 100.0) / 100.0;
    end.z = std::round(target_pose.position.z * 100.0) / 100.0;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // 箭头样式（scale.x 是杆宽，scale.y 是箭头头部直径）
    marker.scale.x = 0.08; // 杆宽
    marker.scale.y = 0.3;  // 头宽
    marker.scale.z = 0.0;  // 忽略

    // 原始颜色设置：青绿色
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    follow_relation_pub_->publish(marker);
}

void fissionFusion::visualization()
{
    // fissionFusion::publish_path();

    // fissionFusion::publish_predict_path();

    fissionFusion::publish_odometry();

    fissionFusion::publish_transformed_follow_relation();

    fissionFusion::avoidance();
}