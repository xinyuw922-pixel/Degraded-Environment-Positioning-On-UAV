# MID-360 基线复现与 Jetson 测试

本仓库在 Intel NUC 上跑通了八种方法的完整离线轨迹。本指南从自己的 ROS1 bag 开始，复用代码、运行参数和统计口径。Jetson ARM64 的编译、速度和延迟需在目标机器实测。先选择一条路线跑通，再扩展到其他方法。

| 方法 | 回放入口与选择 | 已验证范围 |
|---|---|---|
| FAST-LIO2、Point-LIO、DLIO、Super-LIO | replay_pointlio.sh，分别选 fastlio、pointlio、dlio、superlio | ROS1 全包，配置在 tools/config/mid360 |
| RKO-LIO、KISS-ICP | replay_python_lio.sh，选 rko、kiss | Python 离线全包，KISS 仅用点云 |
| Traj-LO、GLIM | replay_native_lio.sh，选 trajlo、glim | C++ 全包，Traj-LO 仅用点云，GLIM 使用 CPU 在线里程计 |
| CLID-SLAM | replay_clid.sh | CPU 30 帧冒烟已通过，整包无已验证结果 |
| FAST-LIVO2 | 待接入 | 当前采集缺同步图像、CameraInfo 与相机外参 |

## 1. 准备输入和平台记录

以下命令在仓库根目录、Bash 中运行。将 BAG 改为实际绝对路径，每次运行选择新的输出目录。记录 Jetson 型号、JetPack/L4T、CPU 架构、内存、电源模式、系统/Python/ROS 版本以及 bag 的 SHA256。ROS 路线需要 Noetic、catkin、Livox driver2 的 ROS1 消息；只有 ROS2 的系统需另备可用的 Noetic 环境。Python 路线直接读取 ROS1 bag。

~~~bash
export REPO="$PWD"
export BAG=/absolute/path/to/sensors.bag
export OUT="$HOME/positioning_data/analyses/jetson-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"
uname -m
cat /etc/nv_tegra_release
sha256sum "$BAG"
~~~

输入为 /livox/lidar（livox_ros_driver2/CustomMsg，保留逐点时间）与 /livox/imu（sensor_msgs/Imu）。本适配对应 MID-360 的加速度单位 g 与项目使用的雷达/IMU 外参；更换设备或驱动输出单位时应核对适配。回放只需 bag；复现停靠统计还需从 NUC 另行复制完整 run 目录，保留事件、manifest 与记录的 FAST-LIO2 位姿。

## 2. 优先跑 RKO-LIO 与 KISS-ICP

NUC 使用 CPython 3.11。另建 ARM64 环境，在目标机安装下列实测版本；若 pip 需要从源码构建，请准备 C++ 工具链并保存构建日志。Python wheel 和 third_party 中的开发分支快照版本分别登记。

~~~bash
python3.11 -m venv "$HOME/venvs/lio-modern"
export PYTHON_BIN="$HOME/venvs/lio-modern/bin/python"
"$PYTHON_BIN" -m pip install 'numpy==2.4.6' 'rosbags==0.11.9' 'pyquaternion==0.9.9' 'rko-lio==0.4.0' 'kiss-icp==1.3.0'
bash scripts/replay_python_lio.sh --algorithm rko --bag "$BAG" --output "$OUT/rko-smoke" --duration 10 --threads 4 --voxel-size 0.25
bash scripts/replay_python_lio.sh --algorithm rko --bag "$BAG" --output "$OUT/rko-025" --threads 4 --voxel-size 0.25
bash scripts/replay_python_lio.sh --algorithm kiss --bag "$BAG" --output "$OUT/kiss" --threads 4
unset PYTHON_BIN
~~~

RKO 省略 --voxel-size 时采用发行包默认值（本次版本为 1 m）。分别保存两档结果，以相同输入和线程数比较。

## 3. ROS1 四种基线

每种算法在独立 catkin 工作区原生编译。先按对应上游 README 安装系统依赖，编译 Livox ROS1 驱动，然后设置 LIVOX_SETUP。以下以 Point-LIO 为例。其它算法替换源码、补丁和 algo，参数文件名与 algo 一致。FAST-LIO2 和 Point-LIO 使用的专用工作区在回放时会重定向源码 Log 目录。

~~~bash
export LIVOX_SETUP="$HOME/livox_ws/devel/setup.bash"
source /opt/ros/noetic/setup.bash
source "$LIVOX_SETUP" --extend
algo=pointlio
ws="$HOME/ws-$algo"
mkdir -p "$ws/src"
cp -a "$REPO/third_party/point_lio" "$ws/src/point_lio"
patch -d "$ws/src/point_lio" -p1 < "$REPO/third_party/patches/point_lio_driver2.patch"
(cd "$ws" && catkin_make -j2)
source "$ws/devel/setup.bash" --extend
bash scripts/replay_pointlio.sh --algorithm "$algo" --workspace "$ws" --parameters "$REPO/tools/config/mid360/$algo.yaml" --bag "$BAG" --output "$OUT/$algo" --rate 1
~~~

- FAST-LIO2：设 algo=fastlio。在专用工作区的 src/FAST_LIO 克隆 https://github.com/hku-mars/FAST_LIO ，checkout 7cc4175de6f8ba2edf34bab02a42195b141027e9，执行 git submodule update --init --recursive，在该目录应用 fastlio_driver2.patch。
- DLIO：设 algo=dlio，复制 third_party/dlio 至工作区 src/dlio，在副本根应用 dlio_driver2_dependency.patch。配置已选 Livox 输入及 IMU 位姿原点。
- Super-LIO：设 algo=superlio，复制 third_party/super_lio 至工作区 src/super_lio，在副本根应用 super_lio_message_dependency.patch。保留其 src 下的各个 catkin 包。

## 4. 原生 C++ 两种基线

依赖提交与补丁文件见 [来源清单](third_party/sources.yaml)。Traj-LO 安装 Eigen3、TBB、yaml-cpp 及源码自带 rosbag reader 所需依赖，准备 Sophus 和 robin-map 头文件。GLIM 在独立前缀编译 fmt、spdlog、GTSAM、gtsam_points、GLIM；在后两者的构建副本分别应用 gtsam_points_boost_optional.patch、glim_gtsam42_navstate.patch。将安装前缀加入 CMAKE_PREFIX_PATH 和运行时库搜索路径，加载 Noetic 和 Livox 工作区。

~~~bash
cmake -S tools/trajlo_build -B "$OUT/build-trajlo" -DCMAKE_BUILD_TYPE=Release -DSOPHUS_INCLUDE_DIR=/path/to/Sophus -DROBIN_INCLUDE_DIR=/path/to/robin-map/include
cmake --build "$OUT/build-trajlo" -j2
bash scripts/replay_native_lio.sh --algorithm trajlo --binary "$OUT/build-trajlo/trajlo_headless" --upstream third_party/traj_lo --bag "$BAG" --output "$OUT/trajlo"

cmake -S tools/glim_build -B "$OUT/build-glim" -DCMAKE_BUILD_TYPE=Release
cmake --build "$OUT/build-glim" -j2
bash scripts/replay_native_lio.sh --algorithm glim --binary "$OUT/build-glim/glim_headless" --upstream third_party/glim --bag "$BAG" --output "$OUT/glim"
~~~

## 5. CLID-SLAM 短段验证

按 third_party/clid_slam 中的依赖说明准备算法解释器。当前适配固定 CPU，Jetson CUDA 加速仍待适配。导出器使用上面的现代 Python 环境及 plyfile；外层回放器需要 NumPy、PyYAML。

~~~bash
"$HOME/venvs/lio-modern/bin/python" -m pip install 'plyfile==1.1.5'
"$HOME/venvs/lio-modern/bin/python" -m tools.export_clid_dataset --bag "$BAG" --output "$OUT/clid-data" --frames 30
bash scripts/replay_clid.sh --python /path/to/clid-env/bin/python --upstream third_party/clid_slam --dataset "$OUT/clid-data" --output "$OUT/clid-smoke" --frames 30
~~~

## 6. 对齐与核验

- 核查每次运行的 result.json、输入消息计数、轨迹覆盖、有限位姿、时间戳和日志。短段通过后再跑全量。DLIO 原始发布会有重复时间戳，比较器保留原始文件并在派生分析中去重。
- 已有含停靠窗口及轨迹的同包冷启动参考目录时，从仓库根执行下例。参考目录含 metrics.json 和 trajectory.npz。当前评估对应 4×3 两轮且第一轮第三排按反序假设配对；更换路线需先修改配对规则。末 5 秒两轮误差衡量重复性。
- 在 Jetson 比较速度时固定输入、线程、电源模式与后台负载，同时记录温度和降频。整包时间除以点云帧数给出平均处理成本。传感器采样到 PX4 接收的延迟须另行记录时间戳并对齐时钟。

~~~bash
python3 -m tools.compare_lio --reference /path/to/reference_fastlio_cold --replay "$OUT/rko-025" --candidate-label RKO-LIO
~~~
