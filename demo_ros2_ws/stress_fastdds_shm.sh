#重复次数（次数越多，/dev/shm 残留越多）
ITERATIONS=30
#清理历史残留
echo "[*]清理历史残留..."
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

for i in $(seq 1 $ITERATIONS); do
    echo "===== 循环 $i / $ITERATIONS ====="
    #启动 subscriber
    ros2 run fastrtps_big_message_test big_subscriber > sub_${i}.log 2>&1 &
    SUB_WRAPPER_PID=$!
    sleep 1
    SUB_REAL_PID=$(ps -o pid,comm --ppid $SUB_WRAPPER_PID | grep big_subscriber | awk '{print $1}')
    #启动 publisher
    ros2 run fastrtps_big_message_test big_publisher > pub_${i}.log 2>&1 &
    PUB_WRAPPER_PID=$!
    sleep 1
    PUB_REAL_PID=$(ps -o pid,comm --ppid $PUB_WRAPPER_PID | grep big_publisher | awk '{print $1}')

    echo "[*]subscriber PID=$SUB_REAL_PID, publisher PID=$PUB_REAL_PID"
    #等几秒钟保证至少开始通信
    sleep 3
    #强制 kill -9
    kill -9 $SUB_REAL_PID $PUB_REAL_PID 2>/dev/null
    sleep 1
    #检查/dev/shm文件数量
    FILE_COUNT=$(ls /dev/shm | grep -c fastrtps)
    echo "[*] /dev/shm 现有 $FILE_COUNT 个 fastrtps 文件"
    #如果达到一定数量，提示可能通信失败
    if [ $FILE_COUNT -gt 50 ]; then
        echo "已经堆积超过 50 个 SHM 文件，可能即将触发通信失败"
    fi
done

echo "[*]脚本执行完毕，请检查/dev/shm/是否堆满"

