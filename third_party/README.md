# Third-party dependencies

本目录存放实际使用的第三方算法、驱动和库。当前没有引入任何上游代码，`sources.yaml`中的依赖清单为空。

每个上游项目使用独立子目录。由`tools/`实现适配与工作流；自研调用关系仍为`scripts → tools → utils`。第三方代码不反向依赖本项目。基础第三方库可通过已声明的标准安装接口供utils使用，算法/驱动/进程编排留在tools。

引入时在`sources.yaml`的`dependencies`下增加一项：

```yaml
- name: <dependency_name>
  path: third_party/<dependency_name>
  upstream_url: <repository_url>
  upstream_ref: <branch_or_tag_for_reference>
  commit: <full_commit_hash>
  integration: <vendored_or_submodule>
  license: <verified_license>
  license_files: []
  purpose: <used_by_which_tool>
  build_instructions: <reproducible_build_steps_or_file>
  patches: []
```

- 固定实际commit，分支/标签用于辅助说明。更新版本同时更新记录。
- 选择vendoring时明确复制的源码范围；不把上游`.git`、数据集、模型权重、虚拟环境或编译产物混入源码。
- 选择submodule时使用规范Git子模块流程，提交`.gitmodules`及固定gitlink，说明克隆时的初始化方法。
- 保留LICENSE、NOTICE和版权信息。没有明确许可证的代码不能宣称可自由分发。
- 优先在tools适配上游。必须改上游时记录补丁路径/hash、修改原因、基线commit及验证结果；submodule的本地脏改动不能代替可复现补丁。
- 构建目录、ROS工作区输出及运行数据不进入主仓库。检查主仓库忽略规则是否意外排除上游必需源码或资源，并对具体依赖明确处理。
- 第三方项目内部文件布局、脚本和模块依赖保留其原有规则；自研`utils`的独立模块约束不用于改造上游。
