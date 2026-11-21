#!/usr/bin/env bash
# killall.sh —— 全量清理所有 ROS 相关进程与 FastDDS 共享内存残留（自测试前后调用）
LC_ALL=C
pkill -f 'ros2 launch' 2>/dev/null
pkill -f 'ros2 run' 2>/dev/null
# 按安装路径杀所有 ROS 节点（排除自身整条祖先链，防调用者命令行含路径被误杀）
EXCL="$$"
_p=$PPID
while [ "${_p:-0}" -gt 1 ]; do
  EXCL="$EXCL ${_p}"
  _p=$(ps -o ppid= -p "${_p}" 2>/dev/null | tr -d ' ')
done
for p in $(pgrep -f 'ros2_ws/install' 2>/dev/null); do
  case " $EXCL " in *" $p "*) continue ;; esac
  kill -9 "$p" 2>/dev/null
done
for p in $(pgrep -f '/opt/ros/lyrical/lib' 2>/dev/null); do
  case " $EXCL " in *" $p "*) continue ;; esac
  kill -9 "$p" 2>/dev/null
done
pkill -f 'python3 - ' 2>/dev/null
pkill -9 -f 'python3 - 0' 2>/dev/null
pkill -9 -f 'python3 -$' 2>/dev/null
pkill -f cmd_injector 2>/dev/null
pkill -f vibration_probe 2>/dev/null
pkill -f servo_camera 2>/dev/null
pkill -f wheel_cam 2>/dev/null
pkill -f swf01 2>/dev/null
pkill -f estop_check 2>/dev/null
pkill -f degrade_check 2>/dev/null
pkill -f jitter_check 2>/dev/null
pkill -f e2e_latency 2>/dev/null
pkill -f contract_check 2>/dev/null
pkill -f sil_orchestrator 2>/dev/null
pkill -f chain_sampler 2>/dev/null
pkill -f tf_poll 2>/dev/null
pkill -f image_saver 2>/dev/null
pkill -f tf2_echo 2>/dev/null
pkill -f map_saver 2>/dev/null
# 清掉其他残留测试脚本（排除自身 PID，避免自杀）
for p in $(pgrep -f '/tmp/dsh_' 2>/dev/null); do
  if [ "$p" != "$$" ] && [ "$p" != "$PPID" ]; then kill -9 "$p" 2>/dev/null; fi
done
sleep 3
# 第二波：-9 强制清理仍存活的节点（nohup 孤儿等，同样排除自身祖先链）
pkill -9 -f 'ros2 launch' 2>/dev/null
pkill -9 -f 'ros2 run' 2>/dev/null
for p in $(pgrep -f 'ros2_ws/install' 2>/dev/null); do
  case " $EXCL " in *" $p "*) continue ;; esac
  kill -9 "$p" 2>/dev/null
done
for p in $(pgrep -f '/opt/ros/lyrical/lib' 2>/dev/null); do
  case " $EXCL " in *" $p "*) continue ;; esac
  kill -9 "$p" 2>/dev/null
done
for p in $(pgrep -f '/tmp/dsh_' 2>/dev/null); do
  case " $EXCL " in *" $p "*) continue ;; esac
  kill -9 "$p" 2>/dev/null
done
sleep 2
# 清理 FastDDS 共享内存段与锁（残留会导致 RTPS_TRANSPORT_SHM port lock 失败）
rm -f /dev/shm/fastdds_* /dev/shm/sem.fastdds_* /dev/shm/*.fastdds* 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1
sleep 4
echo "剩余相关进程："
pgrep -af 'ros2|raspbot|rplidar|nav2|twist_mux|ekf|safety|base_node|imu_node|slam|amcl|costmap|injector|probe|check' \
  | grep -vE 'ros2-daemon|grep' | sed 's/^/  /' || true
echo "  （空=干净）"
# 安全态：电机停、蜂鸣关、超声关
for i in 0 1 2 3; do python3 -c "import smbus; smbus.SMBus(1).write_i2c_block_data(0x2B,0x01,[$i,0,0])" 2>/dev/null; done
python3 -c "import smbus; smbus.SMBus(1).write_i2c_block_data(0x2B,0x06,[0]); smbus.SMBus(1).write_i2c_block_data(0x2B,0x07,[0])" 2>/dev/null
echo "安全态已置：电机 0、蜂鸣器关、超声关"
