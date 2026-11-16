# Degraded Environment Positioning

主仓库：[Degraded-Environment-Positioning-On-UAV](https://github.com/xinyuw922-pixel/Degraded-Environment-Positioning-On-UAV)。
本地工作目录：`/home/nuc1/Degraded Environment Positioning`。
仓库版本化实验设计、代码、配置和空白记录模板；原始传感器数据、地图及运行产物在外部数据根独立归档。

阶段一是在约 20 平方米普通房间中建立可复现的无人机定位实验流程；通过地面数据和台架验证后，再推进 PX4 飞行，最后推广到金属环形管道。

**当前状态：设计完成，尚未在本项目下采集或完成任何正式实验。**
现有 Odin1 数据读取与 viser 演示属于历史工程检查，不是本协议的验收结果。

## 阅读入口

1. [代理规范](agents.md)：后续编码与实验记录要求。
2. [实验与系统设计](docs/experiment_plan.md)：阶段、对照方案、试验矩阵、门槛与任务清单。
3. [采集与评测规范](docs/recording_protocol.md)：命名、数据契约、指标和归档规则。
4. [设备与环境清单](docs/inventory.md)：本机核查结果及未确认事项。
5. [设计决定](docs/decisions.md)、[工作日志](docs/work_log.md)。

## 每次实验的记录入口

- 批次计划：复制 `templates/batch_plan.yaml`，在采集前列出全部预定试验和重复次数。
- 元数据：每个 run 复制 `templates/run_manifest.yaml`，原始记录保存在 run 目录。
- 操作报告：复制 `templates/run_record.md`，同步填写实际命令、事件和结论。
- 批次登记：`templates/run_registry.csv`，失败和中止试验也要登记。
- 人工参考：`templates/reference_points.csv`、`templates/stop_observations.csv`。
- 事件及离线处理：`templates/events.csv`、`templates/analysis_manifest.yaml`；原始run与每次评测独立编号。

推荐外部数据根：`/home/nuc1/positioning_data`。本次只设计目录结构，未创建采集数据或启动设备。

## 第一轮怎么做

先测房间与参考停靠点、固定传感器支架，完成时间戳和带宽检查；随后优先进行 Odin1 原生里程计与 MID-360＋FAST-LIO2 地面基线，单独检查 D435i 图像。FAST-LIVO2 为待实现候选，不把它列成已可运行功能。

先做每条件 3 次筛查，冻结设置后做独立的每条件 5 次验收。建图、调参、正式测试数据分开。没有连续独立真值时，报告停靠点精度与重复性，不报告全程 ATE/RPE。

阶段一目标包括：固定参考停靠点三维 RMSE ≤0.05 m、P95 ≤0.10 m；这些是设计目标，未达成。静止抖动和接入延迟目标见实验设计，不能替代位置准确度。
