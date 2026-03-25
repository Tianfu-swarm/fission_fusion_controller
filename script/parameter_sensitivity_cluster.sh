#!/bin/bash
#$ -N parameter_sensitivity
#$ -wd /home/scc/tianfu.zhang/fission_fusion_controller/script
#$ -o logs/output.$JOB_ID.$TASK_ID.log
#$ -e logs/error.$JOB_ID.$TASK_ID.log
#$ -t 1-10
#$ -pe smp 20
#$ -l h_vmem=4G
#$ -l h_rt=40:00:00

SCRIPT_DIR=/home/scc/tianfu.zhang/fission_fusion_controller/script
WS=/home/scc/tianfu.zhang/ros2_ws

ALPHAS=(1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0)
BETAS=(0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0)
REPEATS=10

# 每个任务负责 50 个实验 (1000/20=50)
TASK_SIZE=100
START=$(( (SGE_TASK_ID - 1) * TASK_SIZE ))
END=$(( START + TASK_SIZE - 1 ))

for IDX in $(seq $START $END); do
    alpha_idx=$((IDX / (10 * REPEATS)))
    beta_idx=$(((IDX % (10 * REPEATS)) / REPEATS))
    run=$(((IDX % REPEATS) + 1))

    ALPHA=${ALPHAS[$alpha_idx]}
    BETA=${BETAS[$beta_idx]}

    RESULTS_DIR=$WS/src/fission_fusion_controller/data/alpha${ALPHA}_beta${BETA}/${run}
    mkdir -p $RESULTS_DIR

    echo "Running alpha=$ALPHA beta=$BETA run=$run"
    bash $SCRIPT_DIR/convergence_cluster.sh "$ALPHA" "$BETA" "$RESULTS_DIR"
    echo "Finished alpha=$ALPHA beta=$BETA run=$run"
done
