# SCARA 3DOF Control System

> **Python-based control system for SCARA robot with Inverse Kinematics, Modbus TCP communication, and interactive GUI**

---

## 🔬 Kinematics Deep Dive

### 1. Denavit-Hartenberg (DH) Convention

โปรเจกต์นี้ใช้ **Modified DH (MDH) Convention** ซึ่งแตกต่างจาก Classic DH เล็กน้อย:

**Modified DH vs Classic DH:**
- MDH: Transformations อยู่ที่ frame ปลาย (end) ของ link
- Classic DH: Transformations อยู่ที่ frame ต้น (start) ของ link

**DH Parameter Table สำหรับ SCARA:**

| Joint i | α<sub>i-1</sub> | a<sub>i-1</sub> | d<sub>i</sub> | θ<sub>i</sub> | Type |
|---------|-----------------|-----------------|---------------|---------------|------|
| 1 | 0° | 0 | L1 (130mm) | θ₁ | R |
| 2 | 0° | L2 (115mm) | L4 (14mm) | θ₂ | R |
| 3 | 0° | L3 (130mm) | d₃ | 0° | P |

โดย: R = Revolute, P = Prismatic

**Transformation Matrix แต่ละ Joint:**

```python
# Joint 1 (Revolute)
A1 = [
    [cos(θ₁), -sin(θ₁), 0, 0      ],
    [sin(θ₁),  cos(θ₁), 0, 0      ],
    [0,        0,       1, L1     ],
    [0,        0,       0, 1      ]
]

# Joint 2 (Revolute)  
A2 = [
    [cos(θ₂), -sin(θ₂), 0, L2·cos(θ₂)],
    [sin(θ₂),  cos(θ₂), 0, L2·sin(θ₂)],
    [0,        0,       1, L4         ],
    [0,        0,       0, 1          ]
]

# Joint 3 (Prismatic)
A3 = [
    [1, 0, 0, L3],
    [0, 1, 0, 0 ],
    [0, 0, 1, d₃],
    [0, 0, 0, 1 ]
]
```

### 2. Forward Kinematics (FK) - Complete Derivation

**สมการ FK แบบเต็ม:**

```
T₀³ = A₁ · A₂ · A₃
```

**ขั้นตอนการคำนวณ:**

```python
# Step 1: คำนวณ A1
T01 = SE3(0, 0, L1) * SE3.Rz(θ₁)

# Step 2: คำนวณ A2  
T12 = SE3(L2, 0, L4) * SE3.Rz(θ₂)

# Step 3: คำนวณ A3
T23 = SE3(L3, 0, d₃)

# Step 4: รวมกัน
T03 = T01 * T12 * T23
```

**ผลลัพธ์ในรูป Cartesian Coordinates:**

```python
# X position
x = L2·cos(θ₁ + θ₂) + L3·cos(θ₁)

# Y position  
y = L2·sin(θ₁ + θ₂) + L3·sin(θ₁)

# Z position
z = L1 + L4 + d₃
```

**Implementation ใน Python:**

```python
def forward_kinematics(q1, q2, z):
    """
    Calculate end-effector position from joint angles
    
    Args:
        q1: Joint 1 angle (radians)
        q2: Joint 2 angle (radians)  
        z: Prismatic joint position (meters)
    
    Returns:
        (x, y, z): End-effector position in Cartesian space
    """
    x = L2 * np.cos(q1 + q2) + L3 * np.cos(q1)
    y = L2 * np.sin(q1 + q2) + L3 * np.sin(q1)
    z_pos = L1 + L4 + z
    
    return x, y, z_pos

# ใช้ Robotics Toolbox
T = robot.fkine([q1, q2, z])
position = T.t  # [x, y, z]
rotation = T.R  # 3x3 rotation matrix
```

### 3. Inverse Kinematics (IK) - Analytical vs Numerical

#### 3.1 Analytical Solution (Geometric Approach)

สำหรับ SCARA 2R, สามารถแก้ IK แบบ analytical ได้:

**Given:** Target position (x<sub>t</sub>, y<sub>t</sub>)

**Find:** Joint angles (θ₁, θ₂)

**Step 1: คำนวณ θ₂** (Elbow angle)

```python
# Distance from base to target
r = sqrt(x_t² + y_t²)

# Cosine law
cos_θ₂ = (r² - L2² - L3²) / (2·L2·L3)

# Two solutions (elbow up/down)
θ₂ = ±acos(cos_θ₂)
```

**Step 2: คำนวณ θ₁** (Shoulder angle)

```python
# Intermediate angle
φ = atan2(y_t, x_t)
β = atan2(L2·sin(θ₂), L3 + L2·cos(θ₂))

# Shoulder angle
θ₁ = φ - β
```

**ตัวอย่างโค้ด:**

```python
def analytical_ik_2r(x, y):
    """
    Analytical IK for 2R planar robot
    """
    r = np.sqrt(x**2 + y**2)
    
    # Check reachability
    if r > (L2 + L3) or r < abs(L2 - L3):
        return None
    
    # Solve for θ₂
    cos_θ₂ = (r**2 - L2**2 - L3**2) / (2*L2*L3)
    θ₂ = np.arccos(np.clip(cos_θ₂, -1, 1))  # Elbow down
    
    # Solve for θ₁
    φ = np.arctan2(y, x)
    β = np.arctan2(L2*np.sin(θ₂), L3 + L2*np.cos(θ₂))
    θ₁ = φ - β
    
    return θ₁, θ₂
```

#### 3.2 Numerical Solution (Levenberg-Marquardt)

โปรเจกต์ใช้วิธี **Numerical IK** เพราะ:
- รองรับ prismatic joint (Z-axis)
- Handle constraints และ joint limits
- ยืดหยุ่นกว่าเมื่อมีการเปลี่ยนแปลง robot structure

**Levenberg-Marquardt Algorithm:**

```python
def levenberg_marquardt_ik(T_desired, q0, max_iter=50):
    """
    Numerical IK using Levenberg-Marquardt optimization
    
    Args:
        T_desired: Target transformation (SE3)
        q0: Initial joint guess
        max_iter: Maximum iterations
    
    Returns:
        q: Joint angles solution
    """
    q = q0.copy()
    λ = 0.01  # Damping factor
    
    for i in range(max_iter):
        # Forward kinematics
        T_current = robot.fkine(q)
        
        # Error vector (position only)
        e = T_desired.t - T_current.t  # 3x1 error
        
        # Check convergence
        if np.linalg.norm(e) < 1e-4:
            return q, True
        
        # Jacobian matrix
        J = robot.jacob0(q)[:3, :]  # Only position rows (3x3)
        
        # Levenberg-Marquardt update
        Δq = np.linalg.solve(J.T @ J + λ*np.eye(3), J.T @ e)
        
        # Update joints
        q = q + Δq
        
        # Clamp to joint limits
        q = np.clip(q, q_min, q_max)
        
        # Adjust damping
        if np.linalg.norm(e) < 1e-3:
            λ *= 0.9  # Decrease damping (more Newton-like)
        else:
            λ *= 1.1  # Increase damping (more gradient descent-like)
    
    return q, False  # Failed to converge
```

**ทำไมใช้ LM แทน Newton-Raphson?**

| Method | Pros | Cons |
|--------|------|------|
| Newton-Raphson | เร็ว, converge ดี | อาจ diverge ถ้า initial guess ไม่ดี |
| Gradient Descent | มั่นคง, ไม่ค่อย diverge | ช้า |
| **Levenberg-Marquardt** | **สมดุลระหว่างทั้งสอง** | **ซับซ้อนกว่า** |

### 4. Jacobian Matrix Analysis

**Jacobian คืออะไร?**

Jacobian (J) แสดงความสัมพันธ์ระหว่างความเร็วของ joints กับความเร็วของ end-effector:

```
ẋ = J(q) · q̇
```

**สำหรับ SCARA 3DOF:**

```python
J = [
    [∂x/∂θ₁,  ∂x/∂θ₂,  ∂x/∂z],
    [∂y/∂θ₁,  ∂y/∂θ₂,  ∂y/∂z],
    [∂z/∂θ₁,  ∂z/∂θ₂,  ∂z/∂z]
]
```

**การคำนวณ Jacobian:**

```python
# Method 1: Analytical
def analytical_jacobian(q1, q2):
    J = np.array([
        [-L2*sin(q1+q2) - L3*sin(q1), -L2*sin(q1+q2), 0],
        [ L2*cos(q1+q2) + L3*cos(q1),  L2*cos(q1+q2), 0],
        [0,                            0,             1]
    ])
    return J

# Method 2: Using Robotics Toolbox
J = robot.jacob0(q)  # Jacobian in base frame
```

**Geometric Jacobian vs Analytical Jacobian:**

```python
# Geometric Jacobian (6x3): [linear velocity; angular velocity]
J_geom = robot.jacob0(q)

# Analytical Jacobian (3x3): position only
J_anal = J_geom[:3, :]  # เอาเฉพาะแถวที่ 1-3
```

### 5. Singularity Analysis

**Singularity คืออะไร?**

Singularity เกิดเมื่อ robot สูญเสีย degree of freedom ในบางทิศทาง (det(J) = 0)

**ตรวจสอบ Singularity:**

```python
def check_singularity(q, threshold=0.01):
    """
    Check if robot is near singularity
    
    Returns:
        True if singular, False otherwise
    """
    J = robot.jacob0(q)[:3, :3]
    det_J = np.linalg.det(J)
    
    return abs(det_J) < threshold
```

**Singularities ของ SCARA:**

1. **Elbow Singularity:**
   ```python
   # เกิดเมื่อแขนยืดตรงหรือพับเต็มที่
   θ₂ = 0° or ±180°
   ```

2. **Boundary Singularity:**
   ```python
   # เกิดที่ขอบ workspace
   r = L2 + L3  (outer limit)
   r = |L2 - L3| (inner limit)
   ```

**Manipulability Measure:**

```python
def manipulability(q):
    """
    Yoshikawa's manipulability index
    Higher value = better manipulability
    """
    J = robot.jacob0(q)[:3, :3]
    return np.sqrt(np.linalg.det(J @ J.T))

# Example
μ = manipulability([0, 0, 0])  # Best at center
```

### 6. Workspace Analysis

**Reachable Workspace:**

```python
def plot_workspace():
    """
    Generate and plot SCARA workspace
    """
    angles = np.linspace(-np.pi, np.pi, 360)
    
    # Outer boundary (fully extended)
    r_outer = L2 + L3
    x_outer = r_outer * np.cos(angles)
    y_outer = r_outer * np.sin(angles)
    
    # Inner boundary (fully retracted)
    r_inner = abs(L2 - L3)
    x_inner = r_inner * np.cos(angles)
    y_inner = r_inner * np.sin(angles)
    
    plt.plot(x_outer, y_outer, 'b-', label='Outer limit')
    plt.plot(x_inner, y_inner, 'r-', label='Inner limit')
    plt.fill_between(x_outer, y_outer, alpha=0.2)
```

**Workspace Volume:**

```python
# Planar workspace area
A_workspace = π * (r_outer² - r_inner²)
A_workspace = π * ((L2+L3)² - |L2-L3|²)

# Total volume (including Z-axis)
V_workspace = A_workspace × |Z_range|
V_workspace = π * ((L2+L3)² - |L2-L3|²) × 80mm
```

### 7. Trajectory Generation

**Point-to-Point Motion:**

```python
def generate_trajectory(start, end, duration, dt=0.01):
    """
    Generate smooth trajectory using cubic polynomial
    
    s(t) = a₀ + a₁t + a₂t² + a₃t³
    
    Boundary conditions:
    s(0) = 0,    ṡ(0) = 0
    s(T) = 1,    ṡ(T) = 0
    """
    steps = int(duration / dt)
    t = np.linspace(0, duration, steps)
    
    # Cubic polynomial coefficients
    a0, a1 = 0, 0
    a2 = 3 / duration**2
    a3 = -2 / duration**3
    
    # Trajectory parameter s(t)
    s = a0 + a1*t + a2*t**2 + a3*t**3
    
    # Interpolate position
    trajectory = []
    for si in s:
        pos = start + si * (end - start)
        trajectory.append(pos)
    
    return trajectory
```

**Velocity Profile (Trapezoidal):**

```python
def trapezoidal_profile(distance, v_max, a_max):
    """
    Generate trapezoidal velocity profile
    
    Phases:
    1. Acceleration: 0 → v_max
    2. Constant velocity: v_max
    3. Deceleration: v_max → 0
    """
    # Time to accelerate/decelerate
    t_accel = v_max / a_max
    
    # Distance during accel/decel
    d_accel = 0.5 * a_max * t_accel**2
    
    # Check if we can reach v_max
    if 2*d_accel > distance:
        # Triangular profile
        v_peak = np.sqrt(a_max * distance)
        t_accel = v_peak / a_max
        t_total = 2 * t_accel
    else:
        # Trapezoidal profile  
        d_const = distance - 2*d_accel
        t_const = d_const / v_max
        t_total = 2*t_accel + t_const
    
    return t_total
```

### 8. Error Analysis

**Position Error:**

```python
def position_error(q_desired, q_actual):
    """
    Calculate end-effector position error
    """
    T_des = robot.fkine(q_desired)
    T_act = robot.fkine(q_actual)
    
    error = np.linalg.norm(T_des.t - T_act.t)
    return error  # mm
```

**Joint Space Error:**

```python
def joint_error(q_desired, q_actual):
    """
    Calculate joint space error
    """
    error = np.degrees(q_desired - q_actual)
    return error  # degrees
```

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [System Architecture](#-system-architecture)
- [Robot Specifications](#-robot-specifications)
- [Installation](#-installation)
- [Usage](#-usage)
- [Communication Protocol](#-communication-protocol)
- [GUI Controls](#-gui-controls)
- [Technical Details](#-technical-details)
- [Safety Notes](#-safety-notes)
- [Future Improvements](#-future-improvements)

---

## 🎯 Overview

ระบบควบคุมแขนกล SCARA แบบ 3 DOF ประกอบด้วย:
- **2 แกนหมุน (Revolute)**: J1, J2
- **1 แกนเลื่อน (Prismatic)**: Z-axis

โปรแกรมพัฒนาด้วย Python เพื่อควบคุมแขนกลผ่าน PLC Mitsubishi FX5U โดยใช้ Inverse Kinematics สำหรับคำนวณตำแหน่งและมี GUI แบบโต้ตอบได้แบบ real-time

---

## ✨ Features

### Core Functionality
- ✅ **Inverse Kinematics (IK)** - ใช้ Robotics Toolbox (Levenberg-Marquardt algorithm)
- ✅ **PLC Communication** - เชื่อมต่อกับ Mitsubishi FX5U ผ่าน Modbus TCP
- ✅ **Interactive GUI** - ลากเมาส์เพื่อเคลื่อนที่แขนกลแบบ real-time
- ✅ **Command Debouncing** - ป้องกันการส่งคำสั่งซ้ำรัว ๆ (100ms interval)
- ✅ **Auto Reconnection** - เชื่อมต่อ PLC อัตโนมัติเมื่อขาดการเชื่อมต่อ
- ✅ **Simulation Mode** - ทำงานได้โดยไม่ต้องเชื่อมต่อ PLC

### Visualization
- 📊 Workspace visualization (top view)
- 📈 Real-time trajectory trail
- 🎨 Forward kinematics display
- 📉 Z-axis position graph
- 🎯 Target preview during drag operation

---

## 🏗 System Architecture

```
┌─────────────────┐
│   PC (Python)   │
│  ┌───────────┐  │
│  │    GUI    │  │ ← User Interaction
│  │ (Tkinter) │  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │ Robotics  │  │ ← IK Calculation
│  │  Toolbox  │  │
│  └─────┬─────┘  │
│        │        │
│  ┌─────▼─────┐  │
│  │  Modbus   │  │ ← Communication
│  │    TCP    │  │
│  └─────┬─────┘  │
└────────┼────────┘
         │
    ┌────▼────┐
    │   PLC   │
    │  FX5U   │ ← Motor Control
    └────┬────┘
         │
    ┌────▼────┐
    │  Robot  │
    │   Arm   │ ← Physical Movement
    └─────────┘
```

### Components

1. **PC (Python)**
   - GUI management with Tkinter
   - IK calculation using Robotics Toolbox
   - Command queue and debouncing system
   - Trajectory smoothing

2. **PLC (Mitsubishi FX5U)**
   - Receives joint angles via Modbus TCP
   - Controls stepper motors with pulse trains
   - Real-time position feedback

3. **Robot Arm**
   - SCARA configuration (2R + 1P)
   - Workspace displayed in GUI top view

---

## 🤖 Robot Specifications

### Physical Parameters

| Link | Type | Parameter |
|------|------|-----------|
| Joint 1 | Revolute | L1 = 130 mm |
| Joint 2 | Revolute | L2 = 115 mm |
| Joint 3 | Prismatic | L3 = 130 mm, L4 = 14 mm |

### DH Parameters (Modified DH Convention)

```python
robot = rtb.DHRobot([
    rtb.RevoluteMDH(a=0,    alpha=0, d=L1),
    rtb.RevoluteMDH(a=L2,   alpha=0, d=L4),
    rtb.PrismaticMDH(a=L3,  alpha=0, theta=0)
], name="SCARA_3DOF")
```

### Joint Limits

- **J1**: -90° to 90°
- **J2**: -90° to 90°
- **Z**: 0 to -80 mm

---

## 📦 Installation

### Prerequisites

```bash
Python 3.8 or higher
pip (Python package manager)
```

### Install Dependencies

```bash
pip install roboticstoolbox-python
pip install spatialmath-python
pip install pymodbus
pip install matplotlib
```

### Clone Repository

```bash
git clone https://github.com/yourusername/scara-3dof-control.git
cd scara-3dof-control
```

---

## 🚀 Usage

### Quick Start

```bash
python Scara3DOF.py
```

The GUI will open automatically and start searching for PLC connection.

### Configuration

Edit the following parameters in `Scara3DOF.py`:

```python
# PLC Connection
PLC_IP = "192.168.1.100"  # Your PLC IP address
PLC_PORT = 502             # Modbus TCP port

# Robot Parameters
L1 = 0.13  # Link 1 length (meters)
L2 = 0.115 # Link 2 length (meters)
L3 = 0.13  # Link 3 length (meters)
```

---

## 📡 Communication Protocol

### System Integration Overview

ระบบนี้บูรณาการเชื่อมต่อ 3 ส่วนหลัก:

```
PC (Python) ←→ Modbus TCP ←→ PLC FX5U ←→ Stepper Motors
```

### Modbus TCP Protocol

**ทำไมใช้ Modbus TCP?**
- มาตรฐานอุตสาหกรรมสำหรับ PLC communication
- รองรับ TCP/IP ทำให้เชื่อมต่อผ่าน Ethernet ได้
- ไม่ต้องใช้ hardware adapter พิเศษ
- Real-time performance เพียงพอสำหรับ robot control

**Connection Parameters:**
```python
PLC_IP = "192.168.1.100"  # PLC IP address
PLC_PORT = 502             # Standard Modbus TCP port
UNIT_ID = 1                # Modbus slave ID
```

### Register Map Design

โครงสร้างการส่งข้อมูลถูกออกแบบเพื่อส่งค่า 3 joints พร้อมกัน:

| Register | Bit Width | Value Range | Description |
|----------|-----------|-------------|-------------|
| **0** | 16-bit | 0 - 3000 | abs(J1 pulses) |
| **1** | 1-bit | 0 or 1 | J1 direction (1 = CCW) |
| **2** | 16-bit | 0 - 3000 | abs(J2 pulses) |
| **3** | 1-bit | 0 or 1 | J2 direction (1 = CCW) |
| **4** | 16-bit | 0 - 32000 | Z-axis pulses |
| **5** | 1-bit | 0 or 1 | Z direction (1 = down) |

**ตัวอย่าง Modbus Frame:**
```
Write Multiple Registers (Function Code 0x10)
Start Address: 0
Register Count: 6
Data: [1500, 0, 2000, 1, 16000, 0]
```

### Angle to Pulse Conversion

**สูตรการแปลงสำหรับ J1 และ J2:**

```python
# Constants
STEPS_PER_REV = 400      # Stepper motor steps/revolution
GEAR_RATIO = 3.75        # Reducer gear ratio
DEGREES_PER_PULSE = 360 / (STEPS_PER_REV * GEAR_RATIO)

# Conversion function
def angle_to_pulses(angle_rad):
    angle_deg = np.degrees(angle_rad)
    total_pulses = (angle_deg / 360) * STEPS_PER_REV * GEAR_RATIO
    
    pulse_count = int(abs(total_pulses))
    direction_flag = 1 if total_pulses < 0 else 0
    
    return pulse_count, direction_flag
```

**ตัวอย่างการคำนวณ:**
```
J1 = 45° = 0.785 rad
pulses = (45 / 360) × 400 × 3.75 = 187.5 ≈ 188 pulses
direction = 0 (positive)

J2 = -30° = -0.524 rad
pulses = (30 / 360) × 400 × 3.75 = 125 pulses
direction = 1 (negative)
```

### Z-axis Mapping

Z-axis ใช้ prismatic joint โดยแปลงระยะทาง (mm) เป็น pulse:

```python
Z_MIN = -80  # mm (lowest position)
Z_MAX = 0    # mm (highest position)
Z_PULSE_MAX = 32000

def z_to_pulses(z_mm):
    # Linear mapping: 0mm → 0, -80mm → 32000
    normalized = (z_mm - Z_MAX) / (Z_MIN - Z_MAX)
    pulses = int(normalized * Z_PULSE_MAX)
    direction_flag = 1 if z_mm < 0 else 0
    
    return pulses, direction_flag
```

**Resolution:**
```
Z_resolution = 80 mm / 32000 pulses = 0.0025 mm/pulse = 2.5 μm/pulse
```

### Communication Sequence Diagram

```
Python                Queue              Scheduler           PLC
  │                     │                    │                │
  │──Mouse Drag────────>│                    │                │
  │──IK Calculate──────>│                    │                │
  │                     │──New Command──────>│                │
  │                     │                    │                │
  │                     │                    │──Debounce──    │
  │                     │                    │   (100ms)      │
  │                     │                    │                │
  │                     │                    │──Modbus Write─>│
  │                     │                    │                │
  │                     │                    │<──ACK/NACK─────│
  │                     │                    │                │
  │                     │                    │──Read Status──>│
  │                     │                    │<──Position─────│
  │<──────────────Update Display─────────────────────────────│
```

### Python Implementation

**การเขียน Modbus TCP:**
```python
from pymodbus.client import ModbusTcpClient

def send_command_to_plc(j1_rad, j2_rad, z_mm):
    # Convert angles to pulses
    j1_pulse, j1_dir = angle_to_pulses(j1_rad)
    j2_pulse, j2_dir = angle_to_pulses(j2_rad)
    z_pulse, z_dir = z_to_pulses(z_mm)
    
    # Prepare register values
    registers = [
        j1_pulse,  # Register 0
        j1_dir,    # Register 1
        j2_pulse,  # Register 2
        j2_dir,    # Register 3
        z_pulse,   # Register 4
        z_dir      # Register 5
    ]
    
    # Write to PLC
    try:
        client.write_registers(0, registers, unit=1)
        return True
    except Exception as e:
        print(f"Modbus error: {e}")
        return False
```

**การอ่านค่ากลับจาก PLC:**
```python
def read_feedback_from_plc():
    try:
        # Read 6 registers starting from address 100
        result = client.read_holding_registers(100, 6, unit=1)
        
        if not result.isError():
            actual_j1 = pulses_to_angle(result.registers[0], 
                                       result.registers[1])
            actual_j2 = pulses_to_angle(result.registers[2], 
                                       result.registers[3])
            actual_z = pulses_to_z(result.registers[4])
            
            return actual_j1, actual_j2, actual_z
    except:
        return None
```

### Error Handling & Reconnection

```python
class PLCCommunication:
    def __init__(self):
        self.client = None
        self.connected = False
        self.reconnect_delay = 2.0
    
    def connect(self):
        try:
            self.client = ModbusTcpClient(PLC_IP, port=PLC_PORT)
            self.connected = self.client.connect()
            return self.connected
        except:
            self.connected = False
            return False
    
    def auto_reconnect(self):
        while not self.connected:
            print("Attempting to reconnect to PLC...")
            if self.connect():
                print("PLC connected successfully!")
                break
            time.sleep(self.reconnect_delay)
    
    def send_safe(self, registers):
        if not self.connected:
            self.auto_reconnect()
        
        try:
            self.client.write_registers(0, registers, unit=1)
            return True
        except:
            self.connected = False
            return False
```

### Performance Optimization

**Command Debouncing:**
```python
class CommandScheduler:
    def __init__(self, min_interval=0.1):
        self.min_interval = min_interval  # 100ms
        self.last_send_time = 0
        self.pending_command = None
    
    def schedule(self, command):
        self.pending_command = command
    
    def process(self):
        current_time = time.time()
        
        if self.pending_command is not None:
            if (current_time - self.last_send_time) >= self.min_interval:
                send_to_plc(self.pending_command)
                self.last_send_time = current_time
                self.pending_command = None
```

**ทำไมต้องใช้ Debouncing?**
1. ป้องกันการส่งคำสั่งเร็วเกินไปจนระบบไม่ทัน
2. ลด network traffic
3. ให้ PLC มีเวลาประมวลผลและควบคุม motor
4. ป้องกัน buffer overflow

### PLC Side Configuration

**ใน PLC FX5U ต้องตั้งค่า:**

1. **Modbus TCP Server**
   - เปิดใช้งาน Built-in Ethernet port
   - Set IP Address: 192.168.1.100
   - Enable Modbus TCP protocol

2. **Register Mapping**
   ```
   D0  = J1 pulse count
   D1  = J1 direction
   D2  = J2 pulse count  
   D3  = J2 direction
   D4  = Z pulse count
   D5  = Z direction
   ```

3. **Pulse Output Configuration**
   ```
   Y0 = J1 pulse output
   Y1 = J1 direction output
   Y2 = J2 pulse output
   Y3 = J2 direction output
   Y4 = Z pulse output
   Y5 = Z direction output
   ```

### Timing Analysis

**Communication Cycle Time:**
```
┌─────────────────────────────────────────┐
│ Python Loop: ~16ms (60 FPS GUI)         │
├─────────────────────────────────────────┤
│ IK Calculation: ~5-20ms (depends on)    │
│                 (initial guess quality)  │
├─────────────────────────────────────────┤
│ Debounce Wait: 100ms                    │
├─────────────────────────────────────────┤
│ Modbus TCP: ~1-5ms                      │
│   - Write registers: 1-2ms              │
│   - Network latency: 0.1-1ms (LAN)      │
│   - PLC processing: 1-2ms               │
├─────────────────────────────────────────┤
│ Total Cycle: ~100-150ms per command     │
└─────────────────────────────────────────┘
```

### นำความรู้มาประยุกต์ใช้

การออกแบบ communication protocol นี้ใช้หลักการ:

1. **Real-time Systems** - Debouncing และ scheduling
2. **Control Theory** - Feedback loop และ position control
3. **Network Protocols** - Modbus TCP standard
4. **Embedded Systems** - Register mapping และ pulse generation
5. **Software Engineering** - Error handling และ reconnection logic

---

## 🖱 GUI Controls

### Mouse Controls

| Action | Control |
|--------|---------|
| Move XY | Left-click and drag |
| Move Z | Mouse scroll wheel |
| Adjust Z speed | CTRL + Scroll wheel |

### Manual Input Panel

- **X, Y, Z input fields**: Enter target position in millimeters
- **Home button**: Return to home position (0, 0, 0)
- **Set Current button**: Update display to current position
- **Go button**: Move to specified coordinates

### Visual Elements

- **Workspace circles**: Inner and outer reachable workspace
- **Robot arm**: Real-time visualization of joints
- **Target preview**: Shows intended position during drag
- **Trajectory trail**: Historical path of end-effector
- **Z-axis bar**: Vertical position indicator

---

## 🔧 Technical Details

### Kinematics Theory

#### Forward Kinematics (FK)

Forward kinematics คำนวณตำแหน่ง end-effector จากค่ามุมของ joints โดยใช้ Modified DH Convention

**Transformation Matrix:**
```
T = A₁ × A₂ × A₃
```

สำหรับแต่ละ joint:
```
Aᵢ = Rot(z, θᵢ) × Trans(0, 0, dᵢ) × Trans(aᵢ, 0, 0) × Rot(x, αᵢ)
```

**ตัวอย่างการคำนวณ:**
```python
# Joint angles: q1, q2, prismatic z
T = robot.fkine([q1, q2, z])
position = T.t  # [x, y, z] in Cartesian space
```

**สมการ Forward Kinematics สำหรับ SCARA:**
```
x = L2·cos(q1 + q2) + L3·cos(q1)
y = L2·sin(q1 + q2) + L3·sin(q1)
z = L1 + L4 + z_prismatic
```

#### Inverse Kinematics (IK)

Inverse kinematics หามุม joints จากตำแหน่ง end-effector ที่ต้องการ

**วิธีการแก้ปัญหา IK:**

โปรเจกต์นี้ใช้ **Levenberg-Marquardt (LM) Algorithm** ซึ่งเป็น iterative numerical method:

```python
solution = robot.ikine_LM(
    T_desired,              # Target transformation
    q0=[0, 0, 0],          # Initial guess
    mask=[1, 1, 1, 0, 0, 0] # Only position (x,y,z)
)
```

**Levenberg-Marquardt Algorithm:**

1. กำหนด cost function:
   ```
   E(q) = ||f(q) - x_desired||²
   ```
   โดย f(q) คือ forward kinematics

2. Update joints iteratively:
   ```
   Δq = -(J^T·J + λ·I)⁻¹ · J^T · e
   q_new = q_old + Δq
   ```
   - J = Jacobian matrix
   - λ = damping factor
   - e = error vector

3. ทำซ้ำจนกว่า error < threshold หรือครบ max iterations

**Jacobian Matrix:**

Jacobian แสดงความสัมพันธ์ระหว่างความเร็วของ joints กับความเร็วของ end-effector:

```
ẋ = J(q) · q̇
```

สำหรับ SCARA 3DOF:
```
J = [∂x/∂q1  ∂x/∂q2  ∂x/∂z]
    [∂y/∂q1  ∂y/∂q2  ∂y/∂z]
    [∂z/∂q1  ∂z/∂q2  ∂z/∂z]
```

**การคำนวณ Jacobian:**
```python
J = robot.jacob0(q)  # Jacobian in base frame
```

#### Workspace Analysis

**Reachable Workspace:**

สำหรับ SCARA, workspace เป็นแบบ annular (วงแหวน):

```
r_min = |L2 - L3|  # Inner radius
r_max = L2 + L3    # Outer radius
```

**การตรวจสอบ workspace:**
```python
def is_in_workspace(x, y, z):
    r = sqrt(x² + y²)
    return (r_min ≤ r ≤ r_max) and (z_min ≤ z ≤ z_max)
```

**Singularity Analysis:**

SCARA มี singularities เมื่อ:
1. **Elbow singularity**: เมื่อแขนยืดตรงสุด หรือพับเต็มที่ (det(J) → 0)
2. **Boundary singularity**: ที่ขอบ workspace

### Differential Kinematics

**Velocity Kinematics:**

ความสัมพันธ์ระหว่างความเร็วของ joints และ end-effector:

```
v = J(q) · q̇
```

โดย:
- v = [vₓ, vᵧ, vᵧ]ᵀ (Cartesian velocity)
- q̇ = [q̇₁, q̇₂, ż]ᵀ (Joint velocity)

**Inverse Velocity:**

```
q̇ = J⁻¹ · v
```

ใช้สำหรับ trajectory planning และ real-time control

### Trajectory Planning

**Path Interpolation:**

โปรแกรมใช้ linear interpolation สำหรับ smooth motion:

```python
def interpolate_path(start, end, steps):
    path = []
    for i in range(steps):
        alpha = i / (steps - 1)
        point = start + alpha * (end - start)
        path.append(point)
    return path
```

**Velocity Profile:**

สำหรับ motion control ที่นุ่มนวล ควรใช้ trapezoidal หรือ S-curve velocity profile

### Control System Flow

```
┌─────────────────────────────────────────────────────┐
│                  User Input Layer                    │
│  • Mouse Drag (XY plane)                            │
│  • Scroll Wheel (Z-axis)                            │
│  • Manual Input (X, Y, Z coordinates)               │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│              Workspace Validation                    │
│  • Check r_min ≤ sqrt(x² + y²) ≤ r_max              │
│  • Check z_min ≤ z ≤ z_max                          │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│           Inverse Kinematics (IK) Solver             │
│                                                      │
│  1. Initial guess: q₀ = [0, 0, 0]                  │
│  2. Compute FK: T_current = fkine(q)                │
│  3. Error: e = T_desired - T_current                │
│  4. Jacobian: J = jacob0(q)                         │
│  5. Update: Δq = -(J^T·J + λ·I)⁻¹·J^T·e            │
│  6. New joints: q = q + Δq                          │
│  7. Repeat until ||e|| < ε or max iterations        │
│                                                      │
│  Output: Joint angles [q1, q2, z]                   │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│              Joint Limit Clamping                    │
│  • q1: clamp(-90°, q1, 90°)                         │
│  • q2: clamp(-90°, q2, 90°)                         │
│  • z:  clamp(-80mm, z, 0mm)                         │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│          Angle-to-Pulse Conversion                   │
│                                                      │
│  For J1, J2:                                        │
│    pulse = (angle / π) × 400 × 3.75                 │
│    flag = 1 if angle < 0 else 0                     │
│                                                      │
│  For Z-axis:                                        │
│    pulse = map(z, 0→-80mm, 0→32000)                │
│    flag = 1 if moving down                          │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│            Command Debouncing (100ms)                │
│  • Prevent command flooding                         │
│  • Queue management                                 │
│  • Smooth motion control                            │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│            Modbus TCP Communication                  │
│                                                      │
│  Register Map:                                      │
│  [0] = abs(J1_pulse)                                │
│  [1] = J1_direction_flag                            │
│  [2] = abs(J2_pulse)                                │
│  [3] = J2_direction_flag                            │
│  [4] = Z_pulse                                      │
│  [5] = Z_direction_flag                             │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│         PLC (Mitsubishi FX5U)                       │
│  • Pulse train generation                           │
│  • Stepper motor control                            │
│  • Real-time position feedback                      │
└────────────────────┬────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────┐
│              Physical Robot Motion                   │
│  • 3 Stepper motors                                 │
│  • Mechanical SCARA structure                       │
│  • End-effector positioning                         │
└─────────────────────────────────────────────────────┘
```

### Multi-Threading Architecture

#### 🧵 IK Worker Thread
```python
def ik_worker():
    while running:
        target = ik_queue.get()
        solution = robot.ikine_LM(target)
        if solution.success:
            clamp_joints(solution.q)
            command_queue.put(solution.q)
```

#### 🧵 Command Scheduler
```python
def scheduler():
    last_send = 0
    while running:
        if time.now() - last_send >= 0.1:  # 100ms
            if command_queue.not_empty():
                cmd = command_queue.get()
                send_to_plc(cmd)
                last_send = time.now()
```

#### 🧵 PLC Worker
```python
def plc_worker():
    while running:
        if not plc_connected:
            reconnect_plc()
        
        if has_new_command:
            write_modbus_registers(command)
            read_feedback()
```

#### 🧵 GUI Loop
```python
def gui_update():
    # Forward kinematics for display
    T = robot.fkine(current_joints)
    draw_robot_arm(T)
    
    # Smooth interpolation
    interpolate_display()
    
    # Update trajectory trail
    update_trajectory()
```

### Mathematical Implementation

**Matrix Operations:**

```python
import numpy as np
from spatialmath import SE3

# Homogeneous transformation matrix
T = SE3(x, y, z)

# Rotation matrix
R = T.R  # 3x3 rotation matrix

# Translation vector
t = T.t  # [x, y, z]
```

**Joint Angle Calculation:**

```python
def solve_ik(x, y, z):
    # Create target transformation
    T_target = SE3(x, y, z)
    
    # Solve IK
    sol = robot.ikine_LM(
        T_target,
        q0=[0, 0, 0],
        mask=[1, 1, 1, 0, 0, 0]
    )
    
    return sol.q if sol.success else None
```

### Practical Applications in Kinematics Course

นำความรู้จากวิชา Kinematics มาใช้:

1. **DH Parameters** - ออกแบบ robot model
2. **Forward Kinematics** - คำนวณตำแหน่ง end-effector
3. **Inverse Kinematics** - แปลงตำแหน่งเป้าหมายเป็นมุม joints
4. **Jacobian Matrix** - วิเคราะห์ความเร็วและ singularities
5. **Workspace Analysis** - กำหนดขอบเขตการทำงาน
6. **Trajectory Planning** - วางแผนเส้นทางการเคลื่อนที่

---

## ⚠ Safety Notes

1. **Z-axis Direction**: Prismatic joint requires offset calculation `(z - Z_HOME)`
2. **Simulation Mode**: If PLC is disconnected, system enters simulation automatically
3. **Debouncing Critical**: Prevents timing errors in motor control
4. **Joint Limits**: Software limits prevent mechanical damage
5. **Emergency Stop**: Always have physical emergency stop accessible

### Important Warnings

- ⚠️ Always verify workspace limits before operation
- ⚠️ Test in simulation mode before connecting to real hardware
- ⚠️ Monitor PLC connection status indicator
- ⚠️ Keep clear of robot workspace during operation

---
