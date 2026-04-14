# OpenLoong Dynamics Control - Agent Guide

## Project Overview

OpenLoong Dynamics Control is a humanoid robot motion control framework based on MPC (Model Predictive Control) and WBC (Whole Body Control), operated by Humanoid Robot (Shanghai) Co., Ltd, Shanghai Humanoid Robot Manufacturing Innovation Center, and the OpenAtom Foundation.

This repository provides a control framework deployable on the MuJoCo simulation platform, based on the "Qinglong" (Azure Dragon) robot model, offering motion examples including walking, jumping, and blind stepping over obstacles.

### Core Features

- **Easy Deployment**: Includes main dependencies, no need for numerous third-party library installations
- **Extensible**: Hierarchical modular design with clear logical and functional boundaries
- **Easy to Understand**: Simple code structure with "read-compute-write" logic

## Technology Stack

### Programming Language & Standard
- **Language**: C++17
- **Build System**: CMake (minimum version 3.10)

### Third-Party Dependencies (Included in Repository)

| Library | Purpose | Path |
|---------|---------|------|
| MuJoCo | Physics simulation engine | `third_party/mujoco/` |
| Pinocchio | Dynamics computation | `third_party/pinocchio/` |
| Eigen3 | Matrix operations | `third_party/eigen3/` |
| qpOASES | Quadratic programming solver | `third_party/qpOASES/` |
| GLFW | Graphics UI library | `third_party/glfw/` |
| jsoncpp | JSON parsing | `third_party/jsoncpp/` |
| quill | Logging utility | `third_party/quill/` |
| urdfdom | URDF parsing | `third_party/urdfdom/` |
| boost | Base library | `third_party/boost/` |

### Supported Platforms
- **OS**: Ubuntu 22.04.4 LTS (recommended)
- **Compiler**: g++ 11.4.0
- **Architectures**: x64 (lin_x64) and ARM64 (lin_arm64)

## Project Structure

```
openloong-dyn-control/
├── CMakeLists.txt          # Main build configuration
├── README.md               # Project documentation (Chinese)
├── Tutorial.md             # Model replacement tutorial
├── model_migration_memory.md  # Model migration memory
├── algorithm/              # Control algorithm modules
│   ├── mpc.h/.cpp          # MPC controller
│   ├── wbc_priority.h/.cpp # WBC priority controller
│   ├── wbc_priority_v4.h/.cpp    # speedbot_v4 WBC
│   ├── wbc_priority_v4_leg.h/.cpp # speedbot_v4_leg WBC
│   ├── pino_kin_dyn.h/.cpp       # Pinocchio kinematics/dynamics
│   ├── pino_kin_dyn_v4.h/.cpp    # speedbot_v4 version
│   ├── pino_kin_dyn_v4_leg.h/.cpp # speedbot_v4_leg version
│   ├── gait_scheduler.h/.cpp     # Gait scheduler
│   ├── foot_placement.h/.cpp     # Foot placement planner
│   ├── joystick_interpreter.h/.cpp # Joystick command interpreter
│   ├── StateEst.h/.cpp           # State estimation
│   ├── Eul_W_filter.h/.cpp       # Euler angle filter
│   └── priority_tasks.h/.cpp     # Task priority definitions
├── common/                 # Common modules
│   ├── data_bus.h          # Data bus (core data structure)
│   ├── data_logger.h/.cpp  # Data logger
│   ├── PVT_ctrl.h/.cpp     # PVT joint controller
│   ├── PVT_ctrl_v4.h/.cpp  # speedbot_v4 version
│   ├── PVT_ctrl_v4_leg.h/.cpp # speedbot_v4_leg version
│   └── joint_ctrl_config*.json   # Joint control parameters
├── math/                   # Math utilities
│   ├── useful_math.h/.cpp  # Common math functions
│   ├── bezier_1D.h/.cpp    # Bezier curves
│   ├── LPF_fst.h/.cpp      # Low-pass filter
│   └── ramp_trajectory.h/.cpp # Ramp trajectory
├── sim_interface/          # Simulation interface
│   ├── MJ_interface.h/.cpp       # MuJoCo interface (AzureLoong)
│   ├── MJ_interface_v4.h/.cpp    # MuJoCo interface (speedbot_v4)
│   ├── MJ_interface_v4_leg.h/.cpp # MuJoCo interface (speedbot_v4_leg)
│   └── GLFW_callbacks.h/.cpp     # GLFW callbacks
├── demo/                   # Executable examples
│   ├── walk_wbc_joystick.cpp      # WBC walking + keyboard control
│   ├── walk_mpc_wbc_joystick.cpp  # MPC+WBC walking + keyboard
│   ├── walk_mpc_wbc_v4.cpp        # speedbot_v4 version
│   └── walk_mpc_wbc_leg.cpp       # speedbot_v4_leg version
├── models/                 # Robot models
│   ├── AzureLoong.urdf     # Qinglong robot URDF
│   ├── AzureLoong.xml      # Qinglong MuJoCo model
│   ├── scene_v4.xml        # speedbot_v4 scene
│   ├── scene_v4_leg.xml    # speedbot_v4_leg scene
│   ├── speedbot_v4/        # speedbot_v4 model directory
│   └── speed160/           # speed160 model directory
├── third_party/            # Third-party dependencies (included)
├── record/                 # Simulation data output directory
└── build/                  # Build output directory
```

## Build and Run

### Environment Setup

```bash
# Update and install system dependencies
sudo apt-get update
sudo apt install git cmake gcc-11 g++-11
sudo apt install libglu1-mesa-dev freeglut3-dev
```

### Build

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

### Run Examples

```bash
# Run in build directory
./walk_wbc_joystick           # Basic WBC walking
./walk_mpc_wbc_joystick       # MPC+WBC walking
./walk_mpc_wbc_v4             # speedbot_v4 MPC+WBC walking
./walk_mpc_wbc_leg            # speedbot_v4_leg MPC+WBC walking
```

### Keyboard Controls (Joystick Mode)

| Key | Function |
|-----|----------|
| Space | Start/Stop walking |
| W | Forward (at current set speed) |
| S | Backward (at current set speed) |
| A | Turn left |
| D | Turn right |
| Q | Decrease target speed |
| E | Increase target speed |
| J | Emergency stop and stand |
| H | Reset heading reference |

## Code Organization and Architecture

### Data Flow Architecture

The project uses a **DataBus pattern** for inter-module communication:

```
MJ_Interface -> DataBus -> Algorithm Modules -> DataBus -> PVT_Ctr -> MJ_Interface
                |
           DataLogger (recording)
```

### Core Classes

#### 1. DataBus (common/data_bus.h)
Central data structure containing:
- Sensor feedback (position, velocity, torque, IMU, etc.)
- Control commands (desired position, velocity, torque)
- Robot state (q, dq, kinematics/dynamics quantities)
- MPC/WBC inputs and outputs

Key methods:
- `updateQ()`: Update generalized coordinates q and dq from sensor values

#### 2. MJ_Interface (sim_interface/MJ_interface*.h)
MuJoCo simulation interface responsible for:
- Reading sensor data from MuJoCo (position, velocity, IMU, forces)
- Writing motor torque commands to MuJoCo
- Maintaining joint name to ID mapping

#### 3. Pin_KinDyn (algorithm/pino_kin_dyn*.h)
Pinocchio-based kinematics and dynamics solver:
- Compute Jacobian J and its derivative dJ
- Compute dynamics quantities (M, C, G, Ag, etc.)
- Leg/arm inverse kinematics

#### 4. MPC (algorithm/mpc.h)
Model Predictive Controller:
- Based on single rigid body model
- Uses qpOASES for QP solving
- Prediction horizon N=10, control horizon ch=3

#### 5. WBC_priority (algorithm/wbc_priority*.h)
Whole Body Controller with priority:
- Task-priority based QP formulation
- Separates standing and walking task sets
- Computes joint accelerations and contact forces

#### 6. GaitScheduler (algorithm/gait_scheduler.h)
Gait state machine:
- Manages swing/stance leg states
- Handles phase variable phi
- Touchdown detection based on contact force

#### 7. FootPlacement (algorithm/foot_placement.h)
Swing foot trajectory planner:
- Computes desired swing foot position
- Trajectory generation using Bezier curves

#### 8. PVT_Ctr (common/PVT_ctrl*.h)
Joint-level Position-Velocity-Torque controller:
- PD control with feedforward torque
- Configurable per joint via JSON
- Output filtering and limiting

### Naming Conventions

| Prefix/Suffix | Meaning |
|---------------|---------|
| *_L, _W* | Local (body) frame, World frame |
| *fe_* | Foot end |
| *_L, _l, _R, _r* | Left, Right |
| *swing, sw* | Swing leg |
| *stance, st* | Stance leg |
| *eul, rpy* | Euler angles (roll, pitch, yaw) |
| *omega* | Angular velocity |
| *pos* | Position |
| *vel* | Linear velocity |
| *tor, tau* | Torque |
| *base* | BaseLink |
| *_des* | Desired value |
| *_cur* | Current actual value |
| *_rot* | Rotation matrix |

## Multi-Robot Support

The project supports multiple robot models through parallel implementations:

| Robot | DoF | Interface | WBC | PVT | Demo |
|-------|-----|-----------|-----|-----|------|
| AzureLoong | 31 (14 arm + 2 head + 3 waist + 12 leg) | MJ_interface | wbc_priority | PVT_ctrl | walk_*_joystick |
| speedbot_v4 | 23 (10 arm + 1 waist + 12 leg) | MJ_interface_v4 | wbc_priority_v4 | PVT_ctrl_v4 | walk_mpc_wbc_v4 |
| speedbot_v4_leg | 12 (leg only) | MJ_interface_v4_leg | wbc_priority_v4_leg | PVT_ctrl_v4_leg | walk_mpc_wbc_leg |

### Adding a New Robot

When adding support for a new robot model:

1. **Create parallel files** (do not modify existing AzureLoong files):
   - `sim_interface/MJ_interface_new.*`
   - `algorithm/pino_kin_dyn_new.*`
   - `algorithm/wbc_priority_new.*`
   - `common/PVT_ctrl_new.*`
   - `common/joint_ctrl_config_new.json`

2. **Add model files** to `models/new_robot/`:
   - URDF file for Pinocchio
   - XML file for MuJoCo
   - Mesh files

3. **Add demo** to `demo/walk_mpc_wbc_new.cpp`

4. **Update CMakeLists.txt** to add new executable targets

See `model_migration_memory.md` for detailed migration guidelines.

## Key Control Parameters

### MPC Weights

```cpp
// MPC.h
void set_weight(double u_weight, Eigen::MatrixXd L_diag, Eigen::MatrixXd K_diag);
// u_weight: System input minimum weight
// L_diag: State error weights [eul, pos, omega, vel]
// K_diag: Input weights [fl, tl, fr, tr]
```

### WBC Task Priority

```cpp
// WBC_QP.cpp
std::vector<std::string> taskOrder;
taskOrder.emplace_back("RedundantJoints");
taskOrder.emplace_back("static_Contact");
taskOrder.emplace_back("Roll_Pitch_Yaw_Pz");
taskOrder.emplace_back("PxPy");
taskOrder.emplace_back("SwingLeg");
taskOrder.emplace_back("HandTrack");
```

### Gait Parameters

```cpp
// GaitScheduler.h
double tSwing;        // Swing time per step
double FzThrehold;    // Touchdown force threshold

// FootPlacement.h
double stepHeight;    // Step height
double kp_vx/vy/wz;   // Foot placement adjustment gains
```

### Joint PD Gains

Configured in `common/joint_ctrl_config*.json`:
```json
{
  "joint_name": {
    "kp": 2000.0,
    "kd": 200.0,
    "maxPos": 0.44,
    "minPos": -0.17,
    "maxSpeed": 13.92,
    "maxTorque": 267.0,
    "PVT_LPF_Fc": 20
  }
}
```

## Development Guidelines

### Code Style

- Use meaningful variable names following the prefix/suffix conventions
- Add comments in Chinese for consistency with existing code
- Follow the "read-compute-write" pattern for algorithm modules

### Module Implementation Pattern

Each control module should follow this pattern:

```cpp
class ModuleName {
public:
    void dataBusRead(const DataBus& robotState);  // Read inputs
    void step();                                   // Compute
    void dataBusWrite(DataBus& robotState);        // Write outputs
};
```

### Data Logger Usage

```cpp
DataLogger logger("../record/datalog.log");
logger.addIterm("variable_name", dimension);
logger.finishItermAdding();

// In loop:
logger.startNewLine();
logger.recItermData("variable_name", value);
logger.finishLine();
```

## Testing

### Manual Testing

1. **Standing Test**: Verify robot can stand stably for 20s
2. **Walking Test**: Test forward/backward/turning walking
3. **Disturbance Test**: Apply external forces in simulation

### Regression Testing

When modifying code:
1. Ensure AzureLoong demos still work
2. Test all robot variants
3. Verify data logging works correctly

## Documentation References

- `README.md`: Project overview and basic usage
- `Tutorial.md`: Detailed model replacement tutorial
- `model_migration_memory.md`: Model migration execution memory
- API documentation: https://www.openloong.org.cn/pages/api/html/index.html
- Wiki: https://www.openloong.org.cn/pages/wiki/html/index.html