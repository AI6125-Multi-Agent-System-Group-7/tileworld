# Tileworld

**项目简介**
本项目是 AI6125 多智能体系统课程的组队作业，基于 MASON（Java 的多智能体建模与仿真工具包）实现 Tileworld 测试平台，并在该平台上设计和比较不同的智能体策略。Tileworld 是一个动态且不可预测的网格世界，环境高度参数化，常用于评估智能体架构与元推理策略在不同环境条件下的表现。环境包含代理、可搬运的 tile、不可移动的障碍物和可被填补的 hole；张杰教授还增加了一个加油站，每个Agent必须在油量耗尽前抵达油站进行补充。当代理把 tile 移到 hole 上并填补时获得奖励。参考资料见课程提供的 PDF。\

> [现有Agent介绍这边请~](AgentIntro.md)

**参考资料 (来自 NTU Learn)**
- `group-project.pdf`（课程项目说明）
- `Introducing the Tileworld_ Experimentally evaluating agent architectures.pdf`
- `A history of the Tileworld agent testbed.pdf`

**环境需求**
- JDK 1.8（本项目按 Java SE 8-u211 编译与运行）
- MASON 14（默认 `MASON_14.jar`放在项目根目录，可在.classpath自行调整）
- Java3D 1.5.1（仅在需要 3D 类或 IDE 报 Java3D 缺失时）
- 你的趁手的IDE（IntelliJ, Eclipse, VSCode...）
- 一间空调房，良好的网络，可用的充电口，与可口的小零食

**配置方法（VSCode）**
1. 安装 Java 扩展：`Java` from `Oracle`。
2. 指定 JDK 1.8 与依赖库（路径按自己的机器调整）。示例：
```json
{
  "java.jdt.ls.java.home": "C:\\Program Files (x86)\\Java\\jdk1.8.0_211",
  "java.configuration.runtimes": [
    {
      "name": "JavaSE-1.8",
      "path": "C:\\Program Files (x86)\\Java\\jdk1.8.0_211",
      "default": true
    }
  ],
  "java.project.referencedLibraries": [
    "MASON_14.jar",
    "C:\\Program Files (x86)\\Java\\Java3D\\1.5.1\\lib\\ext\\j3dcore.jar",
    "C:\\Program Files (x86)\\Java\\Java3D\\1.5.1\\lib\\ext\\j3dutils.jar",
    "C:\\Program Files (x86)\\Java\\Java3D\\1.5.1\\lib\\ext\\vecmath.jar"
  ]
}
```
3. VSCode 中执行：`Java: Clean Java Language Server Workspace`，再 `Developer: Reload Window`。

**运行方式**
- GUI 版本（推荐）：运行 `tileworld.TWGUI` 的 `main`。
- 无界面版本：运行 `tileworld.TileworldMain` 的 `main`。

**常见问题**
- 若运行时提示 `Java3D` 缺失，通常是运行 JDK 版本不对（请用 JDK 1.8）或未添加 Java3D 依赖。
