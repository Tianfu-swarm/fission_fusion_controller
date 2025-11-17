#include "system_init.h"
/*Base ground sensor*/
/*      front
 *
 *        0      r
 * l    1   7    i
 * e  2       6  g
 * f    3   5    h
 * t      4      t
 *
 *       back*/

geometry_msgs::msg::TransformStamped fissionFusion::stay_in_black()
{
    geometry_msgs::msg::TransformStamped target_tf;
    target_tf.header.stamp = this->get_clock()->now();
    target_tf.header.frame_id = "base_link";
    target_tf.child_frame_id = "black_area"; // 默认有目标

    const auto &v = base_ground_data.data;
    if (v.size() < 8)
    {
        // 数据不足，返回 none
        target_tf.child_frame_id = "none";
        target_tf.transform.translation.x = 0.0;
        target_tf.transform.translation.y = 0.0;
        target_tf.transform.translation.z = 0.0;
        target_tf.transform.rotation.w = 1.0;
        return target_tf;
    }

    static const double ang[8] = {
        0.0, M_PI / 4.0, M_PI / 2.0, 3.0 * M_PI / 4.0,
        M_PI, -3.0 * M_PI / 4.0, -M_PI / 2.0, -M_PI / 4.0};

    double tx = 0.0, ty = 0.0;
    for (size_t i = 0; i < 8; ++i)
    {
        if (v[i] == 0.0) // 0 = 黑
        {
            tx += std::cos(ang[i]);
            ty += std::sin(ang[i]);
        }
    }

    if (std::abs(tx) < 1e-6 && std::abs(ty) < 1e-6)
    {
        // 没有黑色 → 返回 none
        target_tf.child_frame_id = "none";
        target_tf.transform.translation.x = 0.0;
        target_tf.transform.translation.y = 0.0;
        target_tf.transform.translation.z = 0.0;
        target_tf.transform.rotation.w = 1.0;
        return target_tf;
    }

    // 有黑色：生成目标点
    double norm = std::hypot(tx, ty);
    tx /= norm;
    ty /= norm;

    double scale = 0.2;
    target_tf.transform.translation.x = tx * scale;
    target_tf.transform.translation.y = ty * scale;
    target_tf.transform.translation.z = 0.0;

    double yaw = std::atan2(ty, tx);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    target_tf.transform.rotation.x = q.x();
    target_tf.transform.rotation.y = q.y();
    target_tf.transform.rotation.z = q.z();
    target_tf.transform.rotation.w = q.w();

    return target_tf;
}

geometry_msgs::msg::TransformStamped fissionFusion::look_for_black(double inflation_distance)
{
    geometry_msgs::msg::TransformStamped target;
    target.header.stamp = this->get_clock()->now();
    target.header.frame_id = "base_link";
    target.child_frame_id = "black_search_target";
    target.transform.rotation.w = 1.0;

    const double avoid_step = 0.5; // 步长，可调
    const double factor = inflation_distance;

    double best_d2 = std::numeric_limits<double>::infinity();
    const TfEntry *nearest = nullptr;

    // 找最近邻居（障碍）
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

        // 切线角度（径向 ± 90°）
        const double tangent_left = angle_to_other + M_PI_2;
        const double tangent_right = angle_to_other - M_PI_2;

        // 前向 (1,0)，选更朝前的那一条切线
        const double cos_left = std::cos(tangent_left);
        const double cos_right = std::cos(tangent_right);

        const double tangent_angle = (cos_left > cos_right) ? tangent_left : tangent_right;

        target.child_frame_id = "avoid_target";
        target.transform.translation.x = avoid_step * std::cos(tangent_angle);
        target.transform.translation.y = avoid_step * std::sin(tangent_angle);
    }
    else
    {
        // 没有障碍：给一个小的前向目标（比 0,0 更自然）
        target.transform.translation.x = avoid_step;
        target.transform.translation.y = 0.0;
    }

    target.transform.translation.z = 0.0;
    target.transform.rotation.x = 0.0;
    target.transform.rotation.y = 0.0;
    target.transform.rotation.z = 0.0;
    target.transform.rotation.w = 1.0;

    return target;
}

/*Motor ground sensor*/
/*       front
 *
 * l|w          r|w
 * e|h   1  0   i|h
 * f|e          g|e
 * t|e   2  3   h|e
 *  |l          t|l
 *
 *       back*/

// “待在灰度” — 如果探头中有非白的，返回相应方向目标；否则返回 none
geometry_msgs::msg::TransformStamped fissionFusion::stay_in_gray()
{
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "gray_area";
    tf.transform.rotation.w = 1.0;

    const auto &v = motor_ground_data.data;
    if (v.size() < 4)
    {
        tf.child_frame_id = "none";
        return tf;
    }

    const double EPS = 1e-6;
    auto is_gray = [&](double x)
    { return x < 1.0 - EPS; }; // <1 即灰
    auto is_white = [&](double x)
    { return !is_gray(x); }; // >=1 即白

    const bool g0 = is_gray(v[0]), g1 = is_gray(v[1]),
               g2 = is_gray(v[2]), g3 = is_gray(v[3]);

    // 四个都灰 → 到位，停
    if (g0 && g1 && g2 && g3)
    {
        tf.child_frame_id = "none";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        return tf;
    }

    std::cout << "stay in gray" << std::endl;
    // 还没全灰 → 前向推进 + 侧向纠偏
    const double STEP_FWD = 0.25; // 前进量（可调大些以“冲进去”）
    const double STEP_LAT = 0.12; // 侧向纠偏量（小一点防抖）

    // 统计左右“白”的数量：谁白得多，就往那一侧挪一点
    const int left_white = (is_white(v[1]) ? 1 : 0) + (is_white(v[2]) ? 1 : 0);
    const int right_white = (is_white(v[0]) ? 1 : 0) + (is_white(v[3]) ? 1 : 0);

    double dx = STEP_FWD;
    double dy = 0.0;
    if (left_white > right_white)
        dy = +STEP_LAT; // 左白多 → y+
    else if (right_white > left_white)
        dy = -STEP_LAT; // 右白多 → y-

    tf.transform.translation.x = dx;
    tf.transform.translation.y = dy;
    tf.transform.translation.z = 0.0;
    return tf;
}

// “寻找灰度” — 如果当前环境中所有探头几乎白，就朝前／朝某方向找灰度；否则，可以先 stay
geometry_msgs::msg::TransformStamped fissionFusion::look_for_gray(double inflation_distance)
{
    geometry_msgs::msg::TransformStamped target;
    target.header.stamp = this->get_clock()->now();
    target.header.frame_id = "base_link";
    target.child_frame_id = "gray_envelope_follow";
    target.transform.rotation.w = 1.0;

    // —— 参数（可按需微调）——
    static constexpr int HAND = +1;    // +1 左手(CCW)，-1 右手(CW)
    const double step_follow = 0.20;   // 绕行步长
    const double step_approach = 0.45; // 接近步长
    const double k_rad = 0.60;         // 半径误差校正增益
    const double band_follow = 0.05;   // 跟随带宽：|d - d0| ≤ band → 仅切向
    const double band_acquire = 0.20;  // 捕获带宽：d ≥ d0 + band_acquire → 进入接近模式
    const double eps = 1e-9;

    // 距离上限（≤0 表示不限）
    const double max_range = follow_range;
    const double max_r2 = (max_range > 0.0)
                              ? max_range * max_range
                              : std::numeric_limits<double>::infinity();

    // —— 1) 在 tf_entries 中挑参考对象：最近 & 在范围内 & frame[3]==1.0 → is_fissioner==true ——
    const TfEntry *ref = nullptr;
    double best_d2 = std::numeric_limits<double>::infinity();

    for (const auto &e : tf_entries)
    {
        if (e.d2 < 1e-8)
            continue; // 跳过自己/极近
        if (e.d2 > max_r2)
            continue; // 太远忽略
        if (!e.is_on_gray)
            continue; // 只要 frame[3]==1.0 的对象

        if (e.d2 < best_d2)
        {
            best_d2 = e.d2;
            ref = &e;
        }
    }

    // —— 2) 无参考对象：兜底前进 ——
    if (!ref || best_d2 < eps)
    {
        std::cout << "no neighbor on gray [" << this->get_clock()->now().seconds() << "]" << std::endl;
        target.transform.translation.x = 0.0;
        target.transform.translation.y = 0.0;
        target.transform.translation.z = 0.0;
        return target;
    }

    // —— 3) 二段式：远时接近，近时沿包络线 + 半径微调 ——
    const double d0 = inflation_distance;
    const double d = std::sqrt(best_d2);
    const double nx = ref->dx / d; // 单位法向（从本机指向参考）
    const double ny = ref->dy / d;

    // 固定手性：切向 = 法向旋转 ±90°
    const double tx = (HAND == +1) ? (-ny) : (ny);
    const double ty = (HAND == +1) ? (nx) : (-nx);

    if (d >= d0 + band_acquire)
    {
        // Acquire：纯径向靠近（不绕）
        target.transform.translation.x = (+nx) * step_approach;
        target.transform.translation.y = (+ny) * step_approach;
    }
    else if (std::fabs(d - d0) <= band_follow)
    {
        // === Follow：仅按切线推进（不加固定偏角），选更朝前的一侧 ===
        // 假设 ref->dx, ref->dy 是 “本机 -> 目标” 的向量；base_link 的前向为 +X
        const double vx = ref->dx;
        const double vy = ref->dy;

        // 指向目标的方位角
        const double angle_to_other = std::atan2(vy, vx);

        // 左/右切线方向（相对 base_link）
        const double tangent_left = angle_to_other + M_PI_2;
        const double tangent_right = angle_to_other - M_PI_2;

        // 与前向 [1,0] 的余弦（点积），谁更大谁更“向前”
        const double cos_left = std::cos(tangent_left);
        const double cos_right = std::cos(tangent_right);

        // 选择更朝前的切线方向；若相等，用 HAND 打破平局（+1 左手，-1 右手）
        double tangent_angle;
        if (cos_left > cos_right)
        {
            tangent_angle = tangent_left;
        }
        else if (cos_right > cos_left)
        {
            tangent_angle = tangent_right;
        }
        else
        {
            tangent_angle = (HAND == +1) ? tangent_left : tangent_right;
        }

        // 沿该切线走一步
        const double s = step_follow; // 直接复用你的跟随步长
        target.transform.translation.x = s * std::cos(tangent_angle);
        target.transform.translation.y = s * std::sin(tangent_angle);
    }
    else
    {
        // Follow：切向 + 半径微调（把半径拉回 d0）
        // 用 err = d0 - d 更直观：>0 表示太近需要外推，<0 表示太远需要内拉
        const double err = d0 - d;
        const double rx = +k_rad * err * nx; // err<0(太远) → rx 为负 → 朝内(−|…| * nx)？不，再看：nx 指向参考，err<0 ⇒ rx 为负倍数 * nx ⇒ 朝反向？为避免混淆，直接这样写：
        // 更清晰的写法：
        // const double rx = k_rad * (d0 - d) * nx;
        // const double ry = k_rad * (d0 - d) * ny;

        const double ry = +k_rad * err * ny;

        double sx = tx + rx;
        double sy = ty + ry;
        const double sn = std::hypot(sx, sy);
        if (sn > eps)
        {
            sx /= sn;
            sy /= sn;
        }
        else
        {
            sx = tx;
            sy = ty;
        }

        target.transform.translation.x = sx * step_follow;
        target.transform.translation.y = sy * step_follow;
    }

    std::cout << "look for gray area [" << this->get_clock()->now().seconds() << "]" << std::endl;
    target.transform.translation.z = 0.0;
    return target;
}
