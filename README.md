# Degraded Environment Positioning on UAV

用于研究无人机室内定位，并逐步扩展到金属环形管道等退化环境。

现有硬件：Odin1、Livox MID-360、RealSense D435i；目标飞控平台为PX4。
从同一份 MID-360 录制回放八种已完成的基线，或在 Jetson 上开始迁移测试，请参阅 [算法复现与 Jetson 测试指南](REPRODUCTION.md)。当前验证平台为 x86 NUC。

仓库保存实现代码及必要的配置、依赖说明。代理规范、内部实验方案、记录模板、原始传感器数据和运行产物仅在本地保存，由 `.gitignore` 排除。

代码按以下三层组织：

```text
scripts/             # 顶层启动入口，只保存 .sh
  run.sh             # 选择解释器、设置模块路径、转发参数
tools/               # 算法、设备适配、CLI 和实验工作流
  __init__.py
  __main__.py         # Python 工具入口；当前仅提供帮助
utils/               # 底层独立公共模块
  __init__.py         # 不集中导入其他 utils 模块
third_party/         # 第三方代码和仓库，独立于自研三层结构
  README.md          # 引入与修改规范
  sources.yaml       # 来源、版本、许可证及本地补丁登记
```

调用方向为 `scripts → tools → utils`。tools可以按职责组合其他tools模块，但不能形成循环；utils不调用tools或scripts，utils模块之间也不相互导入或调用。utils允许使用标准库和已声明的第三方依赖。

算法、流程编排、参数解析和ROS节点属于tools；时间/坐标数值处理、数据格式处理等公共基础函数在有实际需求时放入独立utils模块。scripts不包含算法或Python代码。

第三方算法、驱动或库统一放入 `third_party/`，由tools中的适配模块接入，不让第三方代码反向依赖本项目。记录来源URL、固定commit、许可证、引入方式和补丁；只有实际引入时才增加依赖。引入的算法快照及固定版本见来源清单，构建与运行使用独立副本，适配补丁另行登记。utils可使用已声明的基础第三方库，但不直接编排third_party中的算法或驱动入口。

从任意工作目录查看入口帮助：

```bash
bash "/home/nuc1/Degraded Environment Positioning/scripts/run.sh" --help
```

可通过 `PYTHON_BIN` 指定Python解释器。当前入口不会启动设备、运行定位或连接飞控。

FAST-LIO2 可视化订阅 `/fastlio/cloud_registered` 和 `/fastlio/Odometry`，显示当前点云、有限累积预览及轨迹。它不负责启动驱动或定位算法，预览不等于完整保存地图。

```bash
bash scripts/view_fastlio.sh --port 8080 --status-path /tmp/fastlio-viewer-status.json
```

依赖 ROS Noetic 的 `rospy`、`sensor_msgs`、`nav_msgs`，以及 NumPy 和 viser（本机验证版本为 NumPy 1.24.4、viser 0.2.23）。启动脚本默认使用现有 `/home/nuc1/odin1_ws/.venv-viser/bin/python`；其他机器通过 `VISER_PYTHON` 指定已安装依赖并能导入 ROS 模块的解释器。服务绑定 `127.0.0.1`，通过 VS Code 转发所选端口后访问。

记录 MID-360 与 FAST-LIO2（先单独启动驱动和定位）：

4列×3行、连续两轮的采集可直接使用向导：

```bash
bash scripts/record_fastlio.sh --guided --operator xinyuw922
```

终端先要求确认初始化稳定，然后逐行提示P01～P12，每轮顺序相同。每点放稳后按一次回车，自动等待稳定5秒、倒计时记录10秒，再提示下一点；两轮24次全部完成后自动关闭bag并校验。移动和等待期间仍连续录原始数据。输入q并回车或Ctrl+C会中止并保存，当前未完成停靠会标为incomplete。计时不等于自动检测静止，须保持设备不动。

可用 `--grid 4 3 --rounds 2 --settle-seconds 5 --hold-seconds 10` 调整计划，记录时间至少10秒；`--reference-id`/`--reference-file`仍可登记独立测量。每次停靠保存round_index和visit_index，分析器兼容旧CSV；点号不会自动变成参考坐标。默认向导用途为tuning，不自动宣称标定或验收完成。

手动模式仍保留：

```bash
bash scripts/record_fastlio.sh --test-id T02 --purpose tuning --operator xinyuw922 \
  --reference-id RECT_60x40cm --note "60 cm x 40 cm ground rectangle; fixed height/orientation"
```

终端中输入 `init` 标记初始化完成；`start A` / `end` 标记一次停靠窗口，其他参考点同理；`mark 说明` 保存备注；`stop` 正常结束。Ctrl+C保留数据并标为中止。每点建议停10秒，事件以主机时间记录，同时保留最近位姿消息时间；二者不默认属于同一时钟域。没有键盘终端时须指定 `--duration 秒数`。

默认保存 `/livox/lidar`、`/livox/imu` 和 `/fastlio/Odometry`；可用时还记录 `/fastlio/tf`、`/tf_static` 与 `/rosout_agg`。必需话题不存在或类型不匹配时拒绝开始；只录原始数据可显式使用 `--raw-only`。可通过 `--reference-file` 复制独立实测参考点CSV；程序不会生成参考坐标、对齐或精度结论。

每次创建唯一目录，默认位于 `/home/nuc1/positioning_data/runs/`，包含原始bag、带header/接收时间的原始位姿CSV、停靠事件、参数快照、记录器源码及版本、逐话题消息数、SHA256及manifest。轨迹CSV保留原始坐标系，尚未转换到rig_base。应另行登记设备/安装/外参/算法版本及参考测量不确定度；此记录器不替代完整实验登记。结束检查不等于精度通过，不会启动或停止雷达/定位服务。

录制完成后检查输入校验值，并提取每次停靠末5秒的均值、波动及相邻停靠距离：

```bash
bash scripts/review_recording.sh /path/to/completed/run
bash scripts/view_recording.sh /path/to/generated/analysis --port 8080
```

离线viser展示保存的轨迹、停靠散点和可拖动的位姿回放；不需要启动ROS master。分析阶段读取ROS bag，需要现有ROS环境。没有独立参考和固定对齐时不计算绝对ATE；停靠波动不等于定位精度。默认记录中没有配准点云，离线界面不会将原始雷达点云冒充已配准地图。

可选地面对齐诊断：`bash scripts/try_floor_alignment.sh /path/to/analysis`。工具用第一次停靠前的5帧近静止原始扫描估计地面候选平面，生成新analysis；不使用停靠点拟合，不修改原始轨迹或实时定位。用 `view_recording.sh` 打开新目录，可切换原始/对齐坐标并显示初始地面样本。这不是外部真值标定，不保证减少高度差，也不会将Z强制归零。

Point-LIO离线复现入口：`bash scripts/replay_pointlio.sh --help`。需要先将`third_party/point_lio`复制到独立catkin工作区，在副本应用`third_party/patches/point_lio_driver2.patch`后构建。入口要求指定workspace、参数YAML、输入bag和全新output目录；使用独立ROS master，只回放原始雷达/IMU。原始源码与BSD许可保留，输出及ROOT_DIR日志写到实验目录。

`tools.compare_lio`使用原FAST-LIO2 bag位姿的时间对应建立相同停靠窗口，再比较Point-LIO结果。针对当前两轮网格沿用第一轮第三排反序候选对应；不自动推广到其他路线，也不将两轮重复性当绝对精度。可视化仅做首个重叠位姿的固定对齐，不做全轨迹最佳拟合。

DLIO支持通过`replay_pointlio.sh --algorithm dlio`复用离线回放器（名称保留兼容），构建`third_party/dlio`副本前应用其依赖补丁。参数根为dlio，默认比较采用IMU原点作base。`tools.compare_lio --candidate-label DLIO`保留原始定时位姿，分析去重、排除启动5秒并在参考时间采样。仅关闭waitUntilMove发布门控用于监控；不调优配准参数。DLIO文档媒体保留本地，Git仅收录代码和许可。

新增离线入口：`replay_pointlio.sh --algorithm superlio`、`replay_python_lio.sh --algorithm rko|kiss`、`replay_native_lio.sh --algorithm trajlo|glim`、`replay_clid.sh`。各入口的必要参数见 `--help`；均要求全新输出目录，保留独立配置、日志和原始轨迹。Python 方法使用独立环境，可用 `PYTHON_BIN` 指定；CLID 的算法解释器另由 `--python` 指定。

`tools/trajlo_build` 和 `tools/glim_build` 提供无界面 CMake 入口，复用第三方估计器；Traj-LO 需另行提供 Sophus/robin-map 头文件路径，GLIM 需本地 GTSAM、gtsam_points、fmt、spdlog 及 ROS/Livox 依赖。GLIM 当前入口比较 CPU 里程计，不启用全局回环。CLID 先用 `tools.export_clid_dataset` 导出 PLY/IMU；其运行器将补丁应用到输出目录内的源码副本，第三方原始源码保持原样。

`python -m tools.baseline_report --comparison-root /path/to/comparisons --output /path/to/new-report` 汇总已完成方法，缺少全量对比的方法显示 pending；输出可用 `view_recording.sh` 展示。所有比较用同一冷启动参考的末5秒时刻采样，统一 IMU 原点，仅作初始位姿对齐。实验结果和依赖环境不进入仓库。官方 Python wheel 版本与导入的 master 源码提交分别记录，不混称为同一构建。

RKO-LIO 可在 `replay_python_lio.sh --algorithm rko` 后添加 `--voxel-size 0.25`，覆盖上游地图体素参数；不指定时保留上游默认值。实际参数写入每次运行的 `result.json`，修改体素的实验应另存输出目录。
