# Degraded Environment Positioning on UAV

用于研究无人机室内定位，并逐步扩展到金属环形管道等退化环境。

现有硬件：Odin1、Livox MID-360、RealSense D435i；目标飞控平台为PX4。
当前已建立三层入口骨架，尚无定位算法、采集工具或飞控桥接的实现。

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

第三方算法、驱动或库统一放入 `third_party/`，由tools中的适配模块接入，不让第三方代码反向依赖本项目。记录来源URL、固定commit、许可证、引入方式和补丁；只有实际引入时才增加依赖。目前该目录没有下载任何第三方仓库。utils可使用已声明的基础第三方库，但不直接编排third_party中的算法或驱动入口。

从任意工作目录查看入口帮助：

```bash
bash "/home/nuc1/Degraded Environment Positioning/scripts/run.sh" --help
```

可通过 `PYTHON_BIN` 指定Python解释器。当前入口不会启动设备、运行定位或连接飞控。
