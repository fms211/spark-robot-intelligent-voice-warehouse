#!/usr/bin/env bash
# Run in the operator's terminal. Never leave a repeating start publisher.
set -euo pipefail
printf '%s\n' '确认相机绿色框已对准吸盘上的标定贴片。'
read -r -p '确认准备就绪后按 Enter；取消请按 Ctrl+C：' _
for node in /swiftpro_write_node /cali_pos_node /cali_cam_cv3_node; do
  if ! timeout 3 rosnode ping -c 1 "$node" >/dev/null 2>&1; then
    printf '标定节点不可用：%s；没有发送开始信号。\n' "$node" >&2
    exit 1
  fi
done
rostopic pub -1 /start_topic std_msgs/String "data: 'start'"
