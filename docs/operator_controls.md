# 🔧 Operator Controls — LER-2026

**Controller:** Xbox Controller (Port 1)

| Input | Action |
|---|---|
| **Y Button** *(hold)* | Shoot — dynamic hub target (robot-front suppliers) |
| **X Button** *(hold)* | Shoot — feed position |
| **Right Trigger** *(hold)* | Shoot — fixed hub coordinates |
| **Left Trigger** *(press)* | Intake (speed proportional to trigger) |
| **A Button** *(hold)* | Intake retract (motor) |
| **Left Bumper** *(press)* | Retract intake (solenoid only) |
| **B Button** *(press)* | Kill shooter (set mode to DEAD) |

---

## 🤖 Autonomous Modes *(Shuffleboard selector)*

| Option | Description |
|---|---|
| `none` | No auto |
| `left` | Shoot + intake sequence, left side |
| `right` | Shoot + intake sequence, right side |
| `human player` | Human player shoot sequence |

> Set **"Set auto initial delay"** (seconds) in Shuffleboard to delay the start.
