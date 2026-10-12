#!/usr/bin/env bash
# 自包含验证：从 git 提交克隆到干净目录，在【不 source 任何外部工作区】的前提下
# 构建并检查产物。用于证明仓库自洽，而不是只看 grep。
set -u
CLONE=/tmp/iso_clone
rm -rf "$CLONE"
git clone -q ~/projects/semnav "$CLONE"

cd "$CLONE"
export SEMNAV_WS="$CLONE"

echo "=== 1) 克隆体内容 ==="
echo "  文件数: $(git ls-tree -r HEAD --name-only | wc -l)"
echo "  包    : $(ls src/ | tr '\n' ' ')"

echo
echo "=== 2) 环境入口（不经管道，避免子 shell 丢环境）==="
# 注意 1：必须 source 到当前 shell。写成 `source ... | tail` 会因子 shell
#         导致环境丢失，随后 colcon 找不到 ament_cmake。
# 注意 2：ROS 的 setup.bash 会引用未绑定的 AMENT_TRACE_SETUP_FILES，
#         在 set -u 下直接报错，所以 source 前后要临时关掉 -u。
set +u
source src/semnav_bringup/scripts/setup_env.sh
set -u

echo
echo "=== 3) 从零构建 ==="
colcon build --symlink-install 2>&1 | tail -6

# ⚠️ 必须重新 source 一次 install/setup.bash：上面第 2 步 source 时 install/ 还不存在
#    （那是克隆来的干净目录），此时工作区 overlay 尚未进入 AMENT_PREFIX_PATH，
#    下面的 ros2 pkg prefix 会全部找不到包 —— 那是检查顺序错，不是仓库有问题。
set +u
source install/setup.bash
set -u

echo
echo "=== 4) 三个包可见性 ==="
for p in semnav_bringup semnav_models semnav_robot; do
  if ros2 pkg prefix "$p" >/dev/null 2>&1; then
    echo "  ✓ $p -> $(ros2 pkg prefix "$p")"
  else
    echo "  ✗ $p 找不到"
  fi
done

echo
echo "=== 5) xacro 能否独立展开 ==="
python3 - "$CLONE" <<'PY'
import sys, re, xacro
from ament_index_python.packages import get_package_share_path
p = get_package_share_path("semnav_robot") / "urdf" / "robot.urdf.xacro"
d = xacro.process_file(str(p), mappings={
    "odom_source": "ground_truth", "enable_lidar": "true",
    "enable_ranges": "true", "enable_tof": "true",
    "enable_cameras": "false", "enable_imu": "true"}).toxml()
name = re.search(r'<robot name="([^"]+)"', d).group(1)
print(f"  ✓ 展开 {len(d)} 字节，机器人名 {name}，{len(re.findall(r'<sensor ', d))} 个传感器")
PY

echo
echo "=== 6) 构建期产物 ==="
W="$CLONE/install/semnav_bringup/share/semnav_bringup"
echo "  world system 数: $(grep -cE 'gz-sim-[a-z-]+-system' "$W/worlds/apartment.world" 2>/dev/null || echo '?')"
echo "  语义真值       : $(ls "$W/config/" 2>/dev/null | grep -c truth) 个"
echo "  家具模型       : $(ls "$CLONE"/install/semnav_models/share/semnav_models/models/ 2>/dev/null | wc -l) 个"
echo "  安装的脚本     : $(ls "$CLONE"/install/semnav_bringup/lib/semnav_bringup/ 2>/dev/null | wc -l) 个"

echo
echo "=== 7) 全仓库 oomwoo 扫描（应仅出处说明）==="
grep -rn "oomwoo" --include="*.py" --include="*.sh" --include="*.xml" \
     --include="*.yaml" --include="*.xacro" . 2>/dev/null \
  | grep -v "^\./\.git/" | sed 's/^/  /'

echo
rm -rf "$CLONE"
echo "已清理克隆体"
