#!/bin/bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)

ALPHAS=(1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0)
BETAS=(0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)
REPEATS=10

for alpha in "${ALPHAS[@]}"; do
    for beta in "${BETAS[@]}"; do
        for run in $(seq 1 $REPEATS); do
            echo "========================================="
            echo "Running alpha=$alpha beta=$beta run=$run"
            echo "========================================="

            # 创建结果目录
            results_dir="$WS_DIR/src/fission_fusion_controller/data/alpha${alpha}_beta${beta}/${run}"
            mkdir -p "$results_dir"

            # 调用实验脚本，传入参数
            bash "$SCRIPT_DIR/../launch/convergence.sh" \
                "$alpha" \
                "$beta" \
                "$results_dir"

            echo "Finished alpha=$alpha beta=$beta run=$run"
            sleep 3  # 等待进程完全退出
        done
    done
done

echo "All experiments completed"