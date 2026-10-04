set -e
cd "$(dirname "$0")"

if [ ! -d unitree_actuator_sdk ]; then
    echo "== 克隆宇树官方电机SDK =="
    git clone https://github.com/unitreerobotics/unitree_actuator_sdk.git
fi

echo "== 编译 =="
mkdir -p build
cd build
cmake .. && make

echo
echo "== 编译完成, 可执行文件: build/step1_spin  build/slow_angle =="
echo "运行(需 sudo 访问串口):"
echo "  步骤1:  sudo ./step1_spin --mode vel            # 官方SDK让电机转起来"
echo "  步骤2:  sudo ./slow_angle                       # 插值回零 + 键盘输入角度"
echo "  步骤3:  sudo ./slow_angle --zero-shift 30       # 零点正向偏移30°"
echo "  步骤4:  处理组  sudo ./slow_angle --zero-shift 30   (默认上电自检, 无跳变)"
echo "          对照组  sudo ./slow_angle --no-rezero --offset <步骤3打印的offset_total°>  (复现跳变)"
