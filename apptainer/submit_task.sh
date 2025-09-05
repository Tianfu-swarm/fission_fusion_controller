#!/bin/bash
#$ -N fission_batch_job
#$ -cwd
#$ -o "logs/output.$JOB_ID.$TASK_ID.log"
#$ -e "logs/error.$JOB_ID.$TASK_ID.log"
#$ -t 1-10
#$ -tc 10
#$ -j y
#$ -pe smp 45
#$ -l h_vmem=2G
#$ -l h_rt=20:00:00

TASK_ID=${SGE_TASK_ID}

# 子子任务数量设置（每个SGE子任务内部运行多个）
SUBTASKS_PER_TASK=100

echo "Running on node: $(hostname)"
echo "Date: $(date)"

# 打印CPU信息
free -h
lscpu | grep "CPU(s):"

# 定义路径
BASE_DIR=$PWD
LOCAL_RESULT_DIR=/data/scc/$USER/result
FINAL_RESULT_DIR=$BASE_DIR/result
SCRIPT_PATH=$BASE_DIR/convergence_experiments.sh
CONTAINER_NAME=convergence_experiments.sif
SIF_PATH=$BASE_DIR/$CONTAINER_NAME
SHARED_LOCAL_SIF=/data/scc/$USER/$CONTAINER_NAME
LOCAL_SIF=/tmp/convergence_experiments_task_${TASK_ID}.sif

# 创建目录
mkdir -p $LOCAL_RESULT_DIR
mkdir -p $FINAL_RESULT_DIR
mkdir -p logs

# 拷贝共享镜像（如不存在）
if [ ! -f "$SHARED_LOCAL_SIF" ]; then
    echo "[$(date)] Shared container not found. Copying to compute node..."
    cp $SIF_PATH $SHARED_LOCAL_SIF
fi

# 拷贝为本任务专属容器镜像
cp $SHARED_LOCAL_SIF $LOCAL_SIF

# 循环执行多个子子任务
for ((i=1; i<=SUBTASKS_PER_TASK; i++)); do
  RESULT_ID=${TASK_ID}-${i}
  LOCAL_RESULT_FILE=$LOCAL_RESULT_DIR/task_${RESULT_ID}.csv
  FINAL_RESULT_FILE=$FINAL_RESULT_DIR/task_${RESULT_ID}.csv

  # 跳过已完成任务
  if [ -f "$FINAL_RESULT_FILE" ]; then
    echo "[$(date)] Subtask $RESULT_ID already done. Skipping."
    continue
  fi

  echo "[$(date)] Launching subtask $RESULT_ID → $LOCAL_RESULT_FILE"

  ROS_DOMAIN_ID=$((100 + TASK_ID % 100))

  apptainer exec \
    --containall \
    --cleanenv \
    --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --bind $SCRIPT_PATH:/script.sh \
    --bind $LOCAL_RESULT_DIR:/results \
    $LOCAL_SIF \
    bash /script.sh /results/task_${RESULT_ID}.csv &

  CONTAINER_PID=$!
  sleep 10
  wait $CONTAINER_PID

  echo "[$(date)] Subtask $RESULT_ID completed."

  if [ -f "$LOCAL_RESULT_FILE" ]; then
    rsync -av "$LOCAL_RESULT_FILE" "$FINAL_RESULT_FILE"
    rm -f "$LOCAL_RESULT_FILE"
  else
    echo "[$(date)] WARNING: No result file found for Subtask $RESULT_ID"
  fi

  echo "[$(date)] Waiting 5s before next subtask..."
  sleep 5

done

# 清理容器副本
rm -f $LOCAL_SIF

echo "[$(date)] Cleanup done for Task $TASK_ID."
