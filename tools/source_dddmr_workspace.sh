#!/usr/bin/env bash
# Source the ROS workspace and the locally installed DDDMR dependency.
source /opt/ros/humble/setup.bash
# 必须 source：实测不 source 它时，本机对机器人上 /lio/*、/livox/* 等话题
# 【一个都发现不了】—— SCAN 会一直打印 "no odom."。source 之后立刻正常。
# 少了这一行会表现为"话题名配对了但就是收不到"，很难查。
source "$HOME/unitree_ros2/install/setup.bash"
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"
export CMAKE_PREFIX_PATH="/home/hjj/.local${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="/home/hjj/.local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

# --- DDS 传输 ---------------------------------------------------------------
# 机器人侧 CycloneDDS 绑 eth10 (192.168.123.18/24)，本机必须绑【同一网段】的
# 网卡，DDS 组播才通。注意这条线 (192.168.123.0/24) 和 SSH/日常上网走的
# 192.168.20.0/24 不是同一条，别混 —— 绑错了能 ping 通但发现不了节点。
#
# 网卡名不要写死：USB 网卡重新插拔后枚举顺序会变，实测出现过 eth0 -> eth2，
# 而 CYCLONEDDS_URI 还指 eth0 时表现为"话题名都配对了，就是一个也收不到"，
# 且没有任何报错。所以这里按【网段】自动挑网卡。
DDDM_NET_PREFIX="${DDDM_NET_PREFIX:-192.168.123.}"   # 机器狗所在网段
DDDM_IFACE="$(ip -4 -o addr show 2>/dev/null \
              | awk -v p="$DDDM_NET_PREFIX" '$4 ~ ("^" p) {print $2; exit}')"
if [ -z "$DDDM_IFACE" ]; then
  echo "[source_dddmr_workspace] 警告: 没有网卡在 ${DDDM_NET_PREFIX}0/24 网段上，" >&2
  echo "  机器狗连上了吗? 回退用 eth0。可用: $(ip -4 -o addr show | awk '{print $2\"=\"$4}' | tr '\n' ' ')" >&2
  DDDM_IFACE="eth0"
fi

# 显式覆盖而不是 ${VAR:-default}：shell 里若残留
# CYCLONEDDS_URI=file://$HOME/.cyclonedds/cyclonedds.xml（绑 lo 且
# AllowMulticast=false），节点会被完全隔离，同样不报错。
if [ -n "${CYCLONEDDS_URI:-}" ] && [[ "${CYCLONEDDS_URI}" != *"${DDDM_IFACE}"* ]]; then
  echo "[source_dddmr_workspace] 覆盖残留的 CYCLONEDDS_URI (改用 ${DDDM_IFACE})" >&2
fi
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${DDDM_IFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain>"
echo "[source_dddmr_workspace] DDS 绑定网卡: ${DDDM_IFACE} (网段 ${DDDM_NET_PREFIX}0/24)" >&2
