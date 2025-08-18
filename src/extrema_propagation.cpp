#include "system_init.h"

double fissionFusion::exponential_random()
{
    // 使用 thread_local 保证线程安全（C++11）
    static thread_local std::random_device rd;                     // 随机种子
    static thread_local std::mt19937 gen(rd());                    // Mersenne Twister 引擎
    static thread_local std::exponential_distribution<> dist(1.0); // λ=1 的指数分布

    return dist(gen); // 直接生成指数分布随机数
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

    // std::cout << "[" << this->get_clock()->now().seconds() << "]" << " Send vector: [";
    // for (size_t i = 0; i < broadcast_vector.data.size(); ++i)
    // {
    //     std::cout << broadcast_vector.data[i];
    //     if (i != broadcast_vector.data.size() - 1)
    //         std::cout << ", ";
    // }
    // std::cout << "]" << std::endl;
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

        // std::cout << "[" << this->get_clock()->now().seconds() << "]" << " Init a new vector, round_id=" << current_round_id << ", vector: [";
        // for (size_t i = 0; i < x.size(); ++i)
        // {
        //     std::cout << x[i];
        //     if (i != x.size() - 1)
        //         std::cout << ", ";
        // }
        // std::cout << "]" << std::endl;
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

        x.clear();
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
