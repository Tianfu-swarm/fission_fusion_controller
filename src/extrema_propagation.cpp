#include "system_init.h"

double fissionFusion::exponential_random()
{
    // 1) 从 current_namespace 提取 self_id（如 /bot8）
    static const std::regex kBotIdRe(R"((?:^|/)bot(\d+)(?:/|$))");
    int self_id = -1;
    {
        std::smatch m;
        if (std::regex_search(current_namespace, m, kBotIdRe))
        {
            try
            {
                self_id = std::stoi(m[1].str());
            }
            catch (...)
            {
                self_id = -1;
            }
        }
    }

    // 2) 为“当前对象 this”取得/创建专属 RNG（只播种一次）
    using clock = std::chrono::high_resolution_clock;
    static std::mutex map_mtx;
    static std::unordered_map<const fissionFusion *, std::mt19937> gens;

    std::mt19937 *gen_ptr = nullptr;
    {
        std::lock_guard<std::mutex> lk(map_mtx);
        auto it = gens.find(this);
        if (it == gens.end())
        {
            const uint64_t rt_ns = static_cast<uint64_t>(clock::now().time_since_epoch().count());
            const uint64_t pid = static_cast<uint64_t>(::getpid());
            const uint64_t ns_h = static_cast<uint64_t>(std::hash<std::string>{}(current_namespace));
            const uint64_t id64 = (self_id >= 0) ? static_cast<uint64_t>(self_id) : 0ULL;

            std::seed_seq seed{
                static_cast<uint32_t>(rt_ns),
                static_cast<uint32_t>(rt_ns >> 32),
                static_cast<uint32_t>(id64),
                static_cast<uint32_t>(pid),
                static_cast<uint32_t>(ns_h),
                static_cast<uint32_t>(ns_h >> 32)};
            it = gens.emplace(this, std::mt19937(seed)).first; // ← 首次调用时播种
        }
        gen_ptr = &it->second;
    }

    // 3) 取样（指数分布 λ=1）
    static thread_local std::exponential_distribution<double> dist(1.0);
    return dist(*gen_ptr);
}

// Initialize the local random vector x with K values drawn from Exp(1)
void fissionFusion::initialize_vector()
{
    x.clear();
    for (int i = 0; i < K; ++i)
    {
        x.push_back(exponential_random());
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

    std::vector vec = radio_data.data;
    radio_data.data.clear();
    if (vec.empty() || vec.size() % (K + 1) != 0)
    {
        return status;
    }

    int num_neighbors = vec.size() / (K + 1);
    std::vector<double> original_x = x;

    for (int i = 0; i < num_neighbors; ++i)
    {
        int base = i * (K + 1);
        int received_round = static_cast<int>(vec[base]);

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

        // 读取邻居的向量 [base + 1, base + 1 + K)
        std::vector<double> x_neighbor(vec.begin() + base + 1, vec.begin() + base + 1 + K);
        if (x_neighbor.size() != K)
            continue;

        status.received_any_neighbor_in_current_round = true;

        pointwise_min(x, x_neighbor);
    }

    // 判断 x 是否发生变化
    const double epsilon = 1e-8;
    for (int i = 0; i < K; ++i)
    {
        if (std::abs(x[i] - original_x[i]) > epsilon)
        {
            status.x_updated = true;
            // std::cout << "get different vector from neighbor" << std::endl;
            break;
        }
    }

    return status;
}

// Broadcast the local vector x to all neighbors
void fissionFusion::broadcast_vector()
{
    std_msgs::msg::Float64MultiArray broadcast_vector;
    broadcast_vector.data.clear();

    broadcast_vector.data.push_back(static_cast<double>(current_round_id));

    for (size_t i = 0; i < x.size(); ++i)
    {
        double val = x[i];

        if (val <= 0.0)
        {
            std::cerr << "[fissionFusion] WARNING: Received x[" << i << "] = "
                      << val
                      << " (<= 0) — possible data corruption!" << std::endl;
            return;
        }

        broadcast_vector.data.push_back(val);
    }

    radio_actuator_publisher_->publish(broadcast_vector);
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
