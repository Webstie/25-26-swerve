# 25-26 Swerve (FRC)

这是本赛季（2026）基于 swerve 底盘的 FRC 机器人工程，使用 WPILib Gradle 项目结构与多种第三方库（详见 vendordeps）。

## 快速概览
- **语言与构建**: Java
- **主要目录**: `src/main/java/frc/robot` 包含 `Constants.java`、`Robot.java`、`RobotContainer.java`、子系统与命令实现。
- **路径与自动化**: `src/main/deploy/pathplanner` 包含 PathPlanner 的路径与设置（`navgrid.json`, `settings.json`, `autos/`, `paths/`）。
- **在线/本地工具**: PhotonVision、Phoenix Tuner、PathPlanner 等用于视觉与电机调优。

## 运行与调试
- 代码入口: `frc.robot.Main` / `frc.robot.Robot`。
- 机器人容器: `frc.robot.RobotContainer` 包含按键绑定、命令调度与子系统实例。
- 生成的调参常量: `frc.robot.generated.TunerConstants`（由调试工具 / 生成脚本填充），请勿手动修改生成文件。

## PathPlanner 与轨迹
- 路径与自动：`src/main/deploy/pathplanner/paths` 和 `autos`。
- 打包到部署目录后，机器人可通过 PathPlanner 路径进行自动模式。

## 依赖项 & 第三方库
- 本仓库包含 `vendordeps/`，列出了使用的库版本（例如 `Phoenix5`/`Phoenix6`、`REVLib`、`photonlib`、`PathplannerLib` 等）。

## 开发建议
- 在更改电机/控制参数前，先在本地或仿真环境验证逻辑。
- 对于驱动与控制算法的调整，优先使用 SysID / 自动化工具获取物理量，再更新 `TunerConstants`。

## 常用文件参考
- `src/main/java/frc/robot/Constants.java` — 全局常量。
- `src/main/java/frc/robot/RobotContainer.java` — 按键与命令绑定。
- `src/main/deploy/pathplanner` — PathPlanner 配置与路径。

## 调试与校准（保留原始调试备忘录）

以下内容为原项目调试备忘录，保持不变以便查阅与延续记录：

````markdown
# 调试备忘录

仅记录官方引导不清楚以及复杂之处，其他与官方引导同步即可

## PhotonVision安装使用

### 见 https://docs.photonvision.org/en/latest/docs/quick-start/index.html

当系统刚刚烧录还没有进行初始化时，必须先连接至路由器/交换机，再访问photonvision.local:5800完成初始化，将ip配置由DHCP改为带team number的固定地址（如10.95.97.11，多个设备时改变最后一位数字）

当系统已经配置完毕静态地址后，单纯的摄像头连接测试和标定可以直接通过电脑与香橙派直连进行访问，需要将自己的以太网适配器网络配置为"10.95.97"开头的任意一个不冲突ip地址，子网掩码255.255.255.0，其他留空即可，然后直接访问香橙派ip+5800端口即可访问面板

标定板在PhotonClient中选择默认设置下载到本地，AprilTag在https://www.firstinspires.org/resources/library/frc/playing-field中的April Tags (User Guide & Images)处下载，打印时需要选择实际尺寸、100%缩放，保证准确性

## 使用SysID自动校正电机PID参数（适用于底盘电机）

### 见 https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html

修改
```
/* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = xxx;
```
这里的赋值，决定选择的sysId分析模式，（steering舵轮，rotation旋转，translation直行）

选择模式后，根据RobotContainer中绑定的按键操作触发四次测试（正/反转+动态速度/固定速度）
```
// Run SysId routines when holding back/start and X/Y.
// Note that each routine should be run exactly once in a single log.
Driver.back().and(Driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
Driver.back().and(Driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
Driver.start().and(Driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
Driver.start().and(Driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
```
触发后在PhoenixTuner/Datalog Tool中下载当次测试的log，然后将得到的log其中的hoot格式文件转换为WPILog格式才能使用SysId成功打开并分析，将得到的数值填入TunerConstants中即可

## 使用Phoenix Tuner手动校正电机PID参数（适用于发射摩擦轮等自动校正不准确的电机）

### 见 https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html
先PhoenixTuner设置一个速度值比如20来观察输出
* 逐渐增加Ks直到电机快要转动
* 逐渐增加Kv直到输出速度与速度设定点匹配
* 逐渐增加Kp，直到输出开始围绕设定点振荡
* 在此之后微调Kv等将震荡范围控制在设定点上下即可，一般不需要调其他K值