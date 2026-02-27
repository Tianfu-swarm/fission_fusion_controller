#!/bin/bash

while true; do
    echo "[`date`] Starting con.sh ..."

    # 启动 con.sh（后台运行）
    bash convergence.sh &
    PID=$!

    echo "[`date`] con.sh PID = $PID"

    # 等 120 秒
    sleep 150


    echo "[`date`] con.sh stopped. Restarting after 1 sec..."
    sleep 5
done
