#include "system_init.h"

geometry_msgs::msg::TransformStamped fissionFusion::stay_on_black()
{

    /*
        front

           0      r
    l    1   7    i
    e  2       6  g
    f    3   5    h
    t      4      t

        back
    */

    geometry_msgs::msg::TransformStamped target_tf;
    target_tf.header.stamp = rclcpp::Clock().now();
    target_tf.header.frame_id = "base_link";
    target_tf.child_frame_id = "black_area";

    const auto &v = base_ground_data.data;
    if (v.size() < 8)
    {
        // 没有数据 → 返回原点单位旋转
        target_tf.transform.translation.x = 0.0;
        target_tf.transform.translation.y = 0.0;
        target_tf.transform.translation.z = 0.0;
        target_tf.transform.rotation.x = 0.0;
        target_tf.transform.rotation.y = 0.0;
        target_tf.transform.rotation.z = 0.0;
        target_tf.transform.rotation.w = 1.0;
        return target_tf;
    }

    // 8 个传感器方向（弧度）
    static const double ang[8] = {
        0.0,               // 0 front
        M_PI / 4.0,        // 1 front-left
        M_PI / 2.0,        // 2 left
        3.0 * M_PI / 4.0,  // 3 back-left
        M_PI,              // 4 back
        -3.0 * M_PI / 4.0, // 5 back-right
        -M_PI / 2.0,       // 6 right
        -M_PI / 4.0        // 7 front-right
    };

    // 合成方向向量
    double tx = 0.0, ty = 0.0;
    for (size_t i = 0; i < 8; ++i)
    {
        if (v[i] == 0.0)
        { // 0=黑
            tx += std::cos(ang[i]);
            ty += std::sin(ang[i]);
        }
    }

    if (std::abs(tx) < 1e-6 && std::abs(ty) < 1e-6)
    {
        // 没有黑色 → 停在原点
        target_tf.transform.translation.x = 0.0;
        target_tf.transform.translation.y = 0.0;
        target_tf.transform.translation.z = 0.0;
        target_tf.transform.rotation.x = 0.0;
        target_tf.transform.rotation.y = 0.0;
        target_tf.transform.rotation.z = 0.0;
        target_tf.transform.rotation.w = 1.0;
        return target_tf;
    }

    // 归一化
    double norm = std::hypot(tx, ty);
    tx /= norm;
    ty /= norm;

    // 平移（可缩放：比如 0.2m 前方）
    double scale = 0.2; // 目标点距离
    target_tf.transform.translation.x = tx * scale;
    target_tf.transform.translation.y = ty * scale;
    target_tf.transform.translation.z = 0.0;

    // 方向 → 四元数
    double yaw = std::atan2(ty, tx);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    target_tf.transform.rotation.x = q.x();
    target_tf.transform.rotation.y = q.y();
    target_tf.transform.rotation.z = q.z();
    target_tf.transform.rotation.w = q.w();

    return target_tf;
}

geometry_msgs::msg::TransformStamped fissionFusion::look_for_black()
{
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = rclcpp::Clock().now();
    out.header.frame_id = "base_link";
    out.child_frame_id = "black_search_target";

    // 固定步长（米）：每次沿包络推进的弧长
    const double step_len = 0.1;
    const double R = 0.5; // 膨胀半径

    // 跨周期的弧长参数（沿顺时针方向走）
    static double s_state = 0.0;  // 对多边形/包络：沿边累计弧长
    static double th_state = 0.0; // 对圆（1点/2点退化）：角度参数

    auto norm = [](double x, double y)
    { return std::sqrt(x * x + y * y); };

    // 0) 收集候选点（距离 0.3 ~ follow_range）
    std::vector<std::pair<double, double>> pts;
    {
        const double rmin = 0.3;
        const double rmax = follow_range;
        for (const auto &tf : rab_tf.transforms)
        {
            const std::string &id = tf.child_frame_id;
            if (id.size() < 10)
                continue;
            if (id.rfind("/base_link") != id.size() - 10)
                continue;
            if (id.compare(0, 3, "bot") != 0)
                continue;
            double x = tf.transform.translation.x;
            double y = tf.transform.translation.y;
            double d = std::hypot(x, y);
            if (d >= rmin && d <= rmax)
                pts.emplace_back(x, y);
        }
    }

    // A) 无点：返回默认
    if (pts.empty())
    {
        out.transform.translation.x = 0.0;
        out.transform.translation.y = 0.0;
        out.transform.translation.z = 0.0;
        out.transform.rotation.x = 0.0;
        out.transform.rotation.y = 0.0;
        out.transform.rotation.z = 0.0;
        out.transform.rotation.w = 1.0;
        return out;
    }

    // B) 1点/2点：用圆（中心为该点或两点中点），按固定弧长推进
    if (pts.size() <= 2)
    {
        double cx, cy;
        if (pts.size() == 1)
        {
            cx = pts[0].first;
            cy = pts[0].second;
        }
        else
        {
            cx = 0.5 * (pts[0].first + pts[1].first);
            cy = 0.5 * (pts[0].second + pts[1].second);
        }

        // 弧长步进：Δθ = step_len / R，顺时针（减）
        th_state -= step_len / R;

        out.transform.translation.x = cx + R * std::cos(th_state);
        out.transform.translation.y = cy + R * std::sin(th_state);
        out.transform.translation.z = 0.0;
        out.transform.rotation.x = 0.0;
        out.transform.rotation.y = 0.0;
        out.transform.rotation.z = 0.0;
        out.transform.rotation.w = 1.0;
        return out;
    }

    // C) ≥3点：凸包 → 外偏移（包络） → 按弧长参数 s_state 前进 step_len（顺时针）

    // 1) 凸包（单调链，CCW）
    auto cross = [](const std::pair<double, double> &a,
                    const std::pair<double, double> &b,
                    const std::pair<double, double> &c)
    {
        return (b.first - a.first) * (c.second - a.second) - (b.second - a.second) * (c.first - a.first);
    };
    std::sort(pts.begin(), pts.end(),
              [](const auto &A, const auto &B)
              {
                  return (A.first < B.first) ||
                         (A.first == B.first && A.second < B.second);
              });
    std::vector<std::pair<double, double>> hull;
    hull.reserve(pts.size() * 2);
    for (const auto &p : pts)
    {
        while (hull.size() >= 2 &&
               cross(hull[hull.size() - 2], hull.back(), p) <= 0.0)
            hull.pop_back();
        hull.push_back(p);
    }
    const std::size_t t = hull.size();
    for (int i = (int)pts.size() - 2; i >= 0; --i)
    {
        const auto &p = pts[i];
        while (hull.size() > t &&
               cross(hull[hull.size() - 2], hull.back(), p) <= 0.0)
            hull.pop_back();
        hull.push_back(p);
    }
    if (!hull.empty())
        hull.pop_back();

    if (hull.size() < 3)
    {
        // 极端退化：回退到圆逻辑（用点集质心为圆心）
        double sx = 0, sy = 0;
        for (auto &p : pts)
        {
            sx += p.first;
            sy += p.second;
        }
        double cx = sx / pts.size(), cy = sy / pts.size();
        th_state -= step_len / R;
        out.transform.translation.x = cx + R * std::cos(th_state);
        out.transform.translation.y = cy + R * std::sin(th_state);
        out.transform.translation.z = 0.0;
        out.transform.rotation.x = 0.0;
        out.transform.rotation.y = 0.0;
        out.transform.rotation.z = 0.0;
        out.transform.rotation.w = 1.0;
        return out;
    }

    // 2) 外包络：每条边按外法线平移 R，相邻偏移线求交
    // 质心用于判断外侧
    std::pair<double, double> centroid{0.0, 0.0};
    for (const auto &p : hull)
    {
        centroid.first += p.first;
        centroid.second += p.second;
    }
    centroid.first /= hull.size();
    centroid.second /= hull.size();

    // 偏移线 (p_off, dir)
    std::vector<std::pair<double, double>> off;
    off.reserve(hull.size() * 2);
    for (std::size_t i = 0; i < hull.size(); ++i)
    {
        const auto &A = hull[i];
        const auto &B = hull[(i + 1) % hull.size()];
        double ex = B.first - A.first, ey = B.second - A.second;
        double el = norm(ex, ey);
        if (el < 1e-12)
            continue;
        double ux = ex / el, uy = ey / el; // 切向
        double nlx = -uy, nly = ux;        // 左法向（对 CCW 多边形通常指向外侧）
        auto Aoff = std::make_pair(A.first + nlx * R, A.second + nly * R);
        double d0 = norm(A.first - centroid.first, A.second - centroid.second);
        double d1 = norm(Aoff.first - centroid.first, Aoff.second - centroid.second);
        double nx = (d1 > d0) ? nlx : -nlx;
        double ny = (d1 > d0) ? nly : -nly;
        auto p_off = std::make_pair(A.first + nx * R, A.second + ny * R);
        off.push_back(p_off);
        off.push_back(std::make_pair(ux, uy));
    }

    std::vector<std::pair<double, double>> hull_offset;
    {
        const std::size_t m = off.size() / 2;
        if (m < 3)
        {
            hull_offset = hull; // 退化回退
        }
        else
        {
            hull_offset.reserve(m);
            auto line_intersection = [](const std::pair<double, double> &p,
                                        const std::pair<double, double> &dp,
                                        const std::pair<double, double> &q,
                                        const std::pair<double, double> &dq,
                                        std::pair<double, double> &outp) -> bool
            {
                double det = dp.first * dq.second - dp.second * dq.first;
                if (std::fabs(det) < 1e-12)
                    return false;
                double t = ((q.first - p.first) * dq.second - (q.second - p.second) * dq.first) / det;
                outp.first = p.first + t * dp.first;
                outp.second = p.second + t * dp.second;
                return true;
            };
            for (std::size_t i = 0; i < m; ++i)
            {
                auto p1 = off[2 * i], d1 = off[2 * i + 1];
                auto p2 = off[2 * ((i + 1) % m)], d2 = off[2 * ((i + 1) % m) + 1];
                std::pair<double, double> inter;
                if (!line_intersection(p1, d1, p2, d2, inter))
                    inter = p1; // 平行退化
                hull_offset.push_back(inter);
            }
        }
    }
    if (hull_offset.size() < 3)
        hull_offset = hull;

    // 3) 计算包络的周长（CW 遍历时用的边长度相同）
    double perimeter = 0.0;
    for (std::size_t i = 0; i < hull_offset.size(); ++i)
    {
        const auto &A = hull_offset[i];
        const auto &B = hull_offset[(i + 1) % hull_offset.size()];
        perimeter += norm(B.first - A.first, B.second - A.second);
    }
    if (perimeter < 1e-9)
    {
        // 极端退化：回退到圆
        double sx = 0, sy = 0;
        for (auto &p : pts)
        {
            sx += p.first;
            sy += p.second;
        }
        double cx = sx / pts.size(), cy = sy / pts.size();
        th_state -= step_len / R;
        out.transform.translation.x = cx + R * std::cos(th_state);
        out.transform.translation.y = cy + R * std::sin(th_state);
        out.transform.translation.z = 0.0;
        out.transform.rotation.x = 0.0;
        out.transform.rotation.y = 0.0;
        out.transform.rotation.z = 0.0;
        out.transform.rotation.w = 1.0;
        return out;
    }

    // 4) 固定弧长步进：s_state = (s_state + step_len) mod perimeter
    s_state += step_len;
    // 规范到 [0, perimeter)
    if (s_state >= perimeter)
        s_state = std::fmod(s_state, perimeter);
    if (s_state < 0.0)
        s_state += perimeter;

    // 5) 以某个“固定起点”参数化（选 hull_offset[0] 为0点），沿 **顺时针** 走 s_state
    //    CCW 顶点序列的顺时针边方向为 B->A（索引递减）
    double s = s_state;
    std::pair<double, double> P = hull_offset[0];
    // 从“边(0->1)的反向（1->0）”开始
    std::size_t i = hull_offset.size(); // 让第一条边是 (1->0)
    while (true)
    {
        std::size_t iB = (i) % hull_offset.size();                          // B
        std::size_t iA = (i - 1 + hull_offset.size()) % hull_offset.size(); // A
        const auto &B = hull_offset[iB];
        const auto &A = hull_offset[iA];
        double L = norm(A.first - B.first, A.second - B.second);
        if (L < 1e-12)
        {
            i = (i - 1 + hull_offset.size()) % hull_offset.size();
            continue;
        }
        if (s <= L)
        {
            // 在这条边上走 s
            double ux = (A.first - B.first) / L;
            double uy = (A.second - B.second) / L;
            P.first = B.first + ux * s;
            P.second = B.second + uy * s;
            break;
        }
        else
        {
            s -= L;
            i = (i - 1 + hull_offset.size()) % hull_offset.size(); // 下一条顺时针边
        }
    }

    // 6) 输出（旋转单位四元数）
    out.transform.translation.x = P.first;
    out.transform.translation.y = P.second;
    out.transform.translation.z = 0.0;
    out.transform.rotation.x = 0.0;
    out.transform.rotation.y = 0.0;
    out.transform.rotation.z = 0.0;
    out.transform.rotation.w = 1.0;
    return out;
}
