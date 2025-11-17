#include "system_init.h"

double fissionFusion::exponential_random()
{
    using clock = std::chrono::high_resolution_clock;

    // 线程本地 PRNG 与播种标记：无锁，首次调用时播种一次
    static thread_local std::mt19937 gen;
    static thread_local bool seeded = false;

    if (!seeded)
    {
        const uint64_t rt_ns = static_cast<uint64_t>(clock::now().time_since_epoch().count());
        const uint64_t pid = static_cast<uint64_t>(::getpid());

        // 用对象地址与命名空间哈希增加熵；这里能直接访问 this / current_namespace
        const uint64_t self = reinterpret_cast<uint64_t>(this);
        const uint64_t nsh = static_cast<uint64_t>(std::hash<std::string>{}(current_namespace));

        std::seed_seq seed{
            static_cast<uint32_t>(rt_ns), static_cast<uint32_t>(rt_ns >> 32),
            static_cast<uint32_t>(pid), static_cast<uint32_t>(pid >> 32),
            static_cast<uint32_t>(self), static_cast<uint32_t>(self >> 32),
            static_cast<uint32_t>(nsh), static_cast<uint32_t>(nsh >> 32)};
        gen.seed(seed);
        seeded = true;
    }

    static thread_local std::exponential_distribution<double> dist(1.0);
    return dist(gen);
}

// Initialize the local random vector x with K values drawn from Exp(1)
void fissionFusion::initialize_vector()
{
    x.resize(static_cast<size_t>(K));
    for (int i = 0; i < K; ++i)
    {
        x[static_cast<size_t>(i)] = exponential_random();
    }
}

// Perform pointwise minimum between two vectors: a = min(a, b)
void fissionFusion::pointwise_min(std::vector<double> &a, const std::vector<double> &b)
{
    for (int i = 0; i < K; ++i)
    {
        a[i] = std::min(a[i], b[i]);
    }
}

// Estimate the group size using the Extrema Propagation formula
// N_hat = (K - 1) / sum(x)
double fissionFusion::estimate_group_size_extrema()
{
    double sum = 0.0f;
    for (double val : x)
    {
        sum += val;
    }
    return (K - 1) / sum;
}

// Receive a message from a neighbor
fissionFusion::ReceiveStatus fissionFusion::process_incoming_vectors()
{
    ReceiveStatus status;

    // 1) 零拷贝“拿走”消息内容：swap 而不是拷贝
    std::vector<double> vec;
    vec.swap(radio_data.data); // radio_data.data 清空，vec 拿到所有数据的所有权

    const size_t stride = static_cast<size_t>(K + 1);
    if (vec.empty() || (vec.size() % stride) != 0)
    {
        return status;
    }

    const int num_neighbors = static_cast<int>(vec.size() / stride);

    // 2) 遍历邻居，就地做逐元素 min，并记录是否有变化（避免 original_x 复制）
    bool any_recv_cur_round = false;
    bool x_changed = false;

    for (int i = 0; i < num_neighbors; ++i)
    {
        const size_t base = static_cast<size_t>(i) * stride;
        const int received_round = static_cast<int>(vec[base]);

        if (received_round > current_round_id)
        {
            if (!status.should_sync_round || received_round > status.sync_to_round)
            {
                status.should_sync_round = true;
                status.sync_to_round = received_round;
            }
            continue;
        }
        if (received_round != current_round_id)
            continue;

        // 逐邻居过滤（整条邻居向量若含非正/非有限值，丢弃）——
        bool neighbor_ok = true;
        for (int k = 0; k < K; ++k)
        {
            const double v = vec[base + 1 + static_cast<size_t>(k)];
            if (!(std::isfinite(v) && v > 0.0))
            {
                neighbor_ok = false;
                break;
            }
        }
        if (!neighbor_ok)
            continue;

        any_recv_cur_round = true;

        // 与 [base+1 .. base+K] 做逐元素 min（无临时 vector，无分配）
        for (int k = 0; k < K; ++k)
        {
            const double v = vec[base + 1 + static_cast<size_t>(k)];
            if (v < x[static_cast<size_t>(k)])
            {
                x[static_cast<size_t>(k)] = v;
                x_changed = true;
            }
        }
    }

    status.received_any_neighbor_in_current_round = any_recv_cur_round;
    status.x_updated = x_changed;
    return status;
}

// Broadcast the local vector x to all neighbors
void fissionFusion::broadcast_vector()
{
    std_msgs::msg::Float64MultiArray msg;
    msg.data.clear();
    msg.data.reserve(static_cast<size_t>(K) + 1);

    msg.data.push_back(static_cast<double>(current_round_id));
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double val = x[i];
        if (val <= 0.0)
        {
            // 若要稳妥，也可以改为继续发送但夹取一个极小正数，避免直接 return
            // 这里保持你的原语义
            std::cerr << "[fissionFusion] WARNING: x[" << i << "]=" << val
                      << " (<=0) — possible data corruption!\n";
            return;
        }
        msg.data.push_back(val);
    }
    radio_actuator_publisher_->publish(msg);
}

// Main function to run one round of extrema propagation
double fissionFusion::extrema_propagation()
{
    const double epsilon = 1e-6;

    if (x.empty())
    {
        initialize_vector();
        propagation_hops = 0;
        N_history.clear();
        has_started_convergence = false;
    }

    broadcast_vector();
    ReceiveStatus status = process_incoming_vectors();

    // std::cout << "[" << this->get_clock()->now().seconds() << "]" << "x updated state is" << status.x_updated << std::endl;

    double N = estimate_group_size_extrema();

    N_history.push_back(N);
    if (N_history.size() > stability_window)
        N_history.pop_front();

    // 初步收敛检测
    if (!has_started_convergence && N_history.size() >= early_converge_window)
    {
        bool early_converged = true;
        for (size_t i = 1; i < early_converge_window; ++i)
        {
            if (std::abs(N_history[i] - N_history[i - 1]) > epsilon)
            {
                early_converged = false;
                break;
            }
        }
        if (early_converged)
        {
            has_started_convergence = true;
        }
    }

    // 接收到更高轮次，且自己已初步收敛，则同步轮次
    if (status.should_sync_round &&
        status.sync_to_round > current_round_id &&
        has_started_convergence)
    {

        current_round_id = status.sync_to_round;

        initialize_vector();
        has_started_convergence = false;
        propagation_hops = 0;
        N_history.clear();

        return N;
    }

    // 更新传播 hop 状态
    if (status.received_any_neighbor_in_current_round)
    {
        if (status.x_updated)
            propagation_hops = 0;
        else
            propagation_hops++; // 向量没有变化的次数
    }

    // 判断稳定性
    bool stable = false;
    if (N_history.size() == stability_window)
    {
        stable = true;
        for (size_t i = 1; i < N_history.size(); ++i)
        {
            if (std::abs(N_history[i] - N_history[i - 1]) > epsilon)
            {
                stable = false;
                break;
            }
        }
    }

    if (stable && propagation_hops >= required_propagation_hops)
    {

        has_started_convergence = false;
        current_round_id++;

        x.clear();
        return N; // 只在真正收敛时返回新估计
    }

    return -1; //- 1; // 否则返回无效估计，继续等待下一步推进
}
