#!/usr/bin/env bash

# 当前环境信息打印脚本，方便调试和问题反馈时收集信息
# 使用方法
# chmod +x ros2_env_report.sh
# 关键：用你“实际跑车/跑节点”的同一个终端环境
#source /opt/ros/humble/setup.bash
#source ~/your_ws/install/setup.bash   # 有的话

#./ros2_env_report.sh
# 或指定输出文件名
#./ros2_env_report.sh ./maintain/my_report.txt


set -euo pipefail

OUT="${1:-ros2_env_report_$(date +%Y%m%d_%H%M%S).txt}"

# 统一写入文件 + 终端
exec > >(tee "$OUT") 2>&1

echo "========== ROS2 ENV REPORT =========="
echo "Time        : $(date -Is)"
echo "User        : $(whoami)"
echo "Host        : $(hostname)"
echo "PWD         : $(pwd)"
echo

echo "========== OS / Kernel =========="
[ -f /etc/os-release ] && cat /etc/os-release || true
echo
uname -a || true
echo

echo "========== Hardware =========="
command -v lscpu >/dev/null && lscpu | sed -n '1,25p' || true
echo
command -v free >/dev/null && free -h || true
echo
command -v df >/dev/null && df -h -x tmpfs -x devtmpfs || true
echo

echo "========== Toolchain =========="
command -v gcc   >/dev/null && gcc --version   | head -n 1 || true
command -v g++   >/dev/null && g++ --version   | head -n 1 || true
command -v cmake >/dev/null && cmake --version | head -n 1 || true
command -v python3 >/dev/null && python3 -V || true
command -v pip3 >/dev/null && pip3 -V || true
command -v colcon >/dev/null && colcon --version || true
command -v rosdep >/dev/null && rosdep --version || true
echo

echo "========== ROS2 Basic =========="
echo "which ros2   : $(command -v ros2 || echo 'NOT FOUND')"
if command -v ros2 >/dev/null; then
  ros2 --version || true
fi
echo

echo "========== ROS2 Environment Variables =========="
# 这些变量最关键：决定你到底在用哪个 distro、哪个 overlay、哪个 DDS
env | egrep '^(ROS|RMW|AMENT|COLCON|CMAKE_PREFIX_PATH|LD_LIBRARY_PATH|FASTRTPS|CYCLONEDDS|RCUTILS)=' | sort || true
echo

echo "========== AMENT / Overlay Paths =========="
echo "AMENT_PREFIX_PATH:"
echo "${AMENT_PREFIX_PATH-}" | tr ':' '\n' | sed '/^$/d' || true
echo
echo "CMAKE_PREFIX_PATH:"
echo "${CMAKE_PREFIX_PATH-}" | tr ':' '\n' | sed '/^$/d' || true
echo

echo "========== ROS2 Doctor ==========" #ip信息谨慎输出
# if command -v ros2 >/dev/null; then
#   ros2 doctor --report || true
# fi
# echo
if command -v ros2 >/dev/null; then
  if command -v timeout >/dev/null; then
    timeout 1s ros2 doctor --report || echo "[WARN] ros2 doctor --report timeout/failed (exit=$?)" #超时
  else
    ros2 doctor --report || echo "[WARN] ros2 doctor --report failed (exit=$?)"
  fi
fi
echo

echo "========== Installed ROS packages (dpkg) =========="
# 统计当前 ROS_DISTRO 的安装包，方便对齐版本
if command -v dpkg >/dev/null; then
  DISTRO="${ROS_DISTRO-}"
  if [ -n "$DISTRO" ]; then
    echo "ROS_DISTRO = $DISTRO"
    dpkg -l | awk -v d="ros-"$DISTRO '
      $1=="ii" && index($2,d)==1 {print $2" "$3}
    ' | wc -l | awk '{print "Total ros-"ENVIRON["ROS_DISTRO"]" packages: "$1}'
    # echo "--- top 80 ---"
    echo 
    dpkg -l | awk -v d="ros-"$DISTRO '
      $1=="ii" && index($2,d)==1 {print $2" "$3}' 
    #   | head -n 80  #限制输出长度
  else
    echo "ROS_DISTRO not set; run this after sourcing your ROS2 setup."
  fi
fi
echo

# echo "========== Network (optional, might include IP) =========="
# command -v ip >/dev/null && ip -br a || true
# echo

# echo "========== Workspace Git Info (optional) =========="
# 如果你在工作空间里（有 src 目录），就粗略抓一下 git 状态（每个仓库）
# if [ -d "src" ]; then
#   echo "Detected a workspace (src/ exists). Scanning git repos under src/ ..."
#   find src -maxdepth 3 -type d -name ".git" 2>/dev/null | while read -r g; do
#     repo_dir="$(dirname "$g")"
#     echo "--- repo: $repo_dir ---"
#     (cd "$repo_dir" && git rev-parse --short HEAD && git status -sb && echo) || true
#   done
# else
#   echo "No src/ found in current directory. (Skip workspace git scan)"
# fi

echo
echo "========== END =========="
echo "Report saved to: $OUT"
