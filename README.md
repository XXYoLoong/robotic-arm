# robotic-arm

Six-axis robotic arm controller for the provided serial command set.

## Quick start

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

Use `--command "G30"` (or any other instruction such as `M02 F150`) to send a
one-off command right after initialization. Press **Ctrl+C** to stop; if a
communication interruption is detected, reset the controller hardware and rerun
the tool.

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
