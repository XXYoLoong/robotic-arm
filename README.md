# robotic-arm 机械臂控制工具

Language / 语言： [English](#english) | [中文](#中文)

---

## English

Six-axis robotic arm controller for the provided serial command set.

### Quick start

The robot must be powered on, switched to host/PC mode, and connected via the
FT232RL adapter before running the tool.

1. Install dependencies:

   ```bash
   pip install -r requirements.txt
   ```

2. Run the controller, pointing at the exposed COM/tty port (115200 8N1 is
   enforced automatically):

   ```bash
   python -m robotic_arm.cli /dev/ttyUSB0 --interval 0.5
   ```

### What the tool does

- Opens the serial port using 115200 8-N-1
- Performs the recommended T01 → T02 → T03 health/identity queries
- Issues **G29** so the robot moves to the startup position before accepting
  other commands
- Starts a background monitor that repeatedly reads **T06** so you can see live
  coordinates without knowing the initial pose
- Waits up to 10 seconds for `G29` to acknowledge so a slow startup move does
  not trigger a false "No response" error

Use `--command "G30"` (or any other instruction such as `M02 F150`) to send a
one-off command right after initialization. Press **Ctrl+C** to stop; if a
communication interruption is detected, reset the controller hardware and rerun
the tool.

### 3D visualization and interactive control

Launch the Tkinter visualizer to view the arm pose in 3D, set a target
coordinate, and drive the robot directly from the GUI:

```bash
python -m robotic_arm.visualizer COM15 --interval 0.5
```

- The left panel lets you enter X/Y/Z and orientation (A/B/C) targets and issue
  `G00` (rapid), `G01` (linear), `G30` (camera pose), or manual `T06` refreshes
  for troubleshooting.
- The right panel shows the current pose (blue) and the target you entered
  (orange). Coordinates auto-refresh using `T06` at the selected interval so you
  can see how the arm moves toward new commands.

### Windows example (COM15)

If the FT232RL shows up as `COM15`, this command performs the recommended
startup sequence and opens an interactive prompt for further control:

```powershell
python -m robotic_arm.cli COM15 --verbose --interactive
```

The prompt accepts any command from the reference table (for example `G30`,
`G04 T1.0`, `G05 X0 Y0 Z0 A0 B0 C0`) and also provides shortcuts—type `help` to
see options like `home` (G29), `origin` (G28), `coords` (T06), and `camera`
(G30).

---

## 中文

针对提供的串口指令集的六轴机械臂控制工具。

### 快速开始

在运行工具前，先给机械臂上电，切换到上位机/PC 模式，并通过 FT232RL
适配器连接好串口。

1. 安装依赖：

   ```bash
   pip install -r requirements.txt
   ```

2. 运行控制器，指向当前暴露的 COM/tty 端口（115200 8N1 会自动设置）：

   ```bash
   python -m robotic_arm.cli /dev/ttyUSB0 --interval 0.5
   ```

### 工具会做什么

- 以 115200 8-N-1 打开串口
- 按推荐顺序执行 T01 → T02 → T03 健康与身份查询
- 下发 **G29**，在接受其他指令前让机械臂回到开机位置
- 启动后台监听，循环读取 **T06**，无需知道初始姿态即可看到实时坐标
- 针对 `G29` 等启动动作，最多等待 10 秒回应，避免因动作耗时导致的“无响应”报错

使用 `--command "G30"`（或其他如 `M02 F150` 的指令）可在初始化后立即发送
一次性命令。按 **Ctrl+C** 退出；若通信中断，请重置控制器硬件并重新运行。

### 三维可视化与交互控制

启动 Tkinter 可视化界面，以 3D 查看机械臂姿态、设置目标坐标，并直接从
GUI 驱动机器人：

```bash
python -m robotic_arm.visualizer COM15 --interval 0.5
```

- 左侧面板可输入 X/Y/Z 与姿态（A/B/C）目标，并发送 `G00`（快速）、`G01`
  （直线）、`G30`（相机位）、或手动刷新 `T06` 进行排查。
- 右侧面板显示当前姿态（蓝色）与输入的目标（橙色）。坐标会按设定的
  间隔自动通过 `T06` 刷新，便于观察机械臂向目标运动的过程。

### Windows 示例（COM15）

当 FT232RL 显示为 `COM15` 时，可以用下列命令完成推荐的启动流程，并打开
交互式提示符继续控制：

```powershell
python -m robotic_arm.cli COM15 --verbose --interactive
```

提示符支持参考表中的任意指令（如 `G30`、`G04 T1.0`、`G05 X0 Y0 Z0 A0 B0 C0`），
也内置了快捷指令——输入 `help` 查看选项，例如 `home`（G29）、`origin`
（G28）、`coords`（T06）和 `camera`（G30）。
