import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Circle
from pymodbus.client import ModbusTcpClient
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
import math
import threading
import time
import queue
from collections import deque

# ==========================================
# 1. System Configuration & Parameters
# Reference: README Section 2 & 4
# ==========================================

PLC_IP = '192.168.1.250'
PLC_PORT = 502
REFRESH_RATE_MS = 16
COMMAND_DEBOUNCE_MS = 100

# Physical Parameters (Dimensions in meters)
# L1 = Base Height, L2 = Arm 1 Length, L3 = Arm 2 Length, L4 = Z Offset
L1, L2, L3, L4 = 0.13, 0.115, 0.13, 0.014

# Workspace Limits (Calculated from Horizontal Links L2 & L3)
# Reference: README Section 3.5 Workspace Analysis
R_MIN = abs(L2 - L3)  # |115 - 130| = 15mm
R_MAX = L2 + L3       # 115 + 130 = 245mm

Q_LIMITS = {
    'j1': (-np.deg2rad(90), np.deg2rad(90)),
    'j2': (-np.deg2rad(90), np.deg2rad(90)),
    'z':  (-0.08, 0.0)  # -80mm to 0mm
}

Z_HOME = L1 + L4  # Visual height at Z=0
Z_BOTTOM = Z_HOME + Q_LIMITS['z'][0]

# ==========================================
# 2. Robot Model (Modified DH)
# Reference: README Section 3.1 & Appendix A
# ==========================================
robot = rtb.DHRobot([
    # Joint 1: Base Rotation
    rtb.RevoluteMDH(a=0, alpha=0, d=L1, qlim=Q_LIMITS['j1']),
    # Joint 2: Elbow Rotation (Offset by L2=115mm)
    rtb.RevoluteMDH(a=L2, alpha=0, d=L4, qlim=Q_LIMITS['j2']),
    # Joint 3: Prismatic Z (Offset by L3=130mm)
    rtb.PrismaticMDH(a=L3, alpha=0, theta=0, qlim=Q_LIMITS['z'])
], name="SCARA_3DOF")

current_state = {
    'q_actual': np.array([0.0, 0.0, 0.0]),
    'q_target': np.array([0.0, 0.0, 0.0]),
    'q_display': np.array([0.0, 0.0, 0.0]),
    'target_z': Z_HOME,
    'z_speed': 500,
    'plc_connected': False,
    'running': True,
    'last_command_time': 0,
    'pending_command': None,
    'command_lock': threading.Lock()
}

plc_command_queue = queue.Queue(maxsize=1)
ik_queue = queue.Queue(maxsize=1)
ik_result_queue = queue.Queue(maxsize=1)

# ==========================================
# 3. Helper Functions
# Reference: README Section 7.2 & Appendix
# ==========================================

def angle_to_pulses(angle_rad):
    """Reference: README Section 7.2 Step 1"""
    STEPS_PER_REV = 400
    GEAR_RATIO = 3.75
    
    # Convert to degrees for calculation as per README formula
    angle_deg = np.degrees(angle_rad)
    # Formula: (angle_deg / 360) * STEPS * RATIO
    total_pulses = (angle_deg / 360.0) * STEPS_PER_REV * GEAR_RATIO
    
    pulse_count = int(abs(total_pulses))
    # Direction: 0=CW (Positive?), 1=CCW (Negative) - Adjusted based on README logic
    direction_flag = 1 if total_pulses < 0 else 0
    
    return pulse_count, direction_flag

def z_to_pulses(z_mm):
    """Reference: README Section 7.2 Step 2"""
    # z_mm input is expected to be relative (0 to -80)
    Z_MIN = -80
    Z_MAX = 0
    Z_PULSE_MAX = 32000
    
    # Clamp input
    z_mm = max(Z_MIN, min(Z_MAX, z_mm))
    
    # Normalized 0.0 to 1.0 based on displacement
    normalized = abs(z_mm) / abs(Z_MIN)
    pulses = int(normalized * Z_PULSE_MAX)
    
    # Direction 1 if going down (negative)
    direction_flag = 1 if z_mm < 0 else 0
    
    return pulses, direction_flag

def is_in_workspace(x, y, z_rel):
    """Reference: README Section 3.5"""
    r = np.sqrt(x**2 + y**2)
    # Check annular workspace
    if not (R_MIN <= r <= R_MAX):
        return False
    # Check Z limits
    if not (Q_LIMITS['z'][0] <= z_rel <= Q_LIMITS['z'][1]):
        return False
    return True

# ==========================================
# 4. Thread Workers
# ==========================================

def ik_worker():
    """Reference: README Section 7.1 Step 3"""
    while current_state['running']:
        try:
            if not ik_queue.empty():
                target_pos = ik_queue.get()
                x, y, z_abs = target_pos
                z_rel = z_abs - Z_HOME # Convert absolute GUI Z to relative Robot Z
                
                # Pre-check workspace before IK (Optimization)
                if not is_in_workspace(x, y, z_rel):
                    # Skip calculation if out of bounds
                    continue

                T = SE3(x, y, z_rel)
                
                # IK using Levenberg-Marquardt
                sol = robot.ikine_LM(
                    T, 
                    q0=current_state['q_target'], 
                    mask=[1, 1, 1, 0, 0, 0] # x, y, z only
                )
                
                if sol.success:
                    q_new = sol.q.copy()
                    # Apply Joint Limits
                    q_new[0] = np.clip(q_new[0], *Q_LIMITS['j1'])
                    q_new[1] = np.clip(q_new[1], *Q_LIMITS['j2'])
                    q_new[2] = np.clip(z_rel, *Q_LIMITS['z'])
                    
                    try:
                        ik_result_queue.get_nowait()
                    except:
                        pass
                    ik_result_queue.put(q_new)
            
            time.sleep(0.01)
        except Exception as e:
            print(f"IK Error: {e}")
            time.sleep(0.05)

def command_scheduler():
    """Reference: README Section 7.3 Debouncing"""
    while current_state['running']:
        try:
            current_time = time.time()
            with current_state['command_lock']:
                if current_state['pending_command'] is not None:
                    time_since_last = (current_time - current_state['last_command_time']) * 1000
                    
                    if time_since_last >= COMMAND_DEBOUNCE_MS:
                        q_cmd = current_state['pending_command']
                        try:
                            plc_command_queue.get_nowait()
                        except:
                            pass
                        
                        try:
                            plc_command_queue.put_nowait(q_cmd)
                            current_state['last_command_time'] = current_time
                            current_state['pending_command'] = None
                        except:
                            pass
            time.sleep(0.01)
        except Exception as e:
            print(f"Scheduler Error: {e}")
            time.sleep(0.05)

def plc_worker():
    """Reference: README Section 7.2 Communication"""
    client = None
    reconnect_delay = 2.0
    last_sent_q = None
    
    while current_state['running']:
        if client is None or not client.connected:
            try:
                client = ModbusTcpClient(PLC_IP, port=PLC_PORT, timeout=1.0)
                if client.connect():
                    current_state['plc_connected'] = True
                    print(f"✓ Connected to PLC at {PLC_IP}")
                else:
                    current_state['plc_connected'] = False
                    time.sleep(reconnect_delay)
                    continue
            except Exception as e:
                current_state['plc_connected'] = False
                print(f"✗ PLC Connection Error: {e}")
                time.sleep(reconnect_delay)
                continue

        try:
            if not plc_command_queue.empty():
                q_to_send = plc_command_queue.get()
                
                # Check delta to avoid sending duplicate positions
                if last_sent_q is None or np.linalg.norm(q_to_send - last_sent_q) > 0.0005:
                    
                    # Calculate Pulses using README logic
                    val_j1, flag_j1 = angle_to_pulses(q_to_send[0])
                    val_j2, flag_j2 = angle_to_pulses(q_to_send[1])
                    val_z, flag_z   = z_to_pulses(q_to_send[2] * 1000) # Convert m to mm
                    
                    # Register Mapping (README Section 6.2)
                    registers = [
                        val_j1, flag_j1,  # Reg 0-1: J1
                        val_j2, flag_j2,  # Reg 2-3: J2
                        val_z,  flag_z    # Reg 4-5: Z
                    ]

                    client.write_registers(0, registers)
                    last_sent_q = q_to_send.copy()
                    current_state['q_actual'] = q_to_send.copy()
                    print(f"→ Sent: J1={np.rad2deg(q_to_send[0]):.1f} J2={np.rad2deg(q_to_send[1]):.1f} Z={q_to_send[2]*1000:.1f}")
                
            time.sleep(0.02)
        except Exception as e:
            print(f"PLC Send Error: {e}")
            if client:
                try:
                    client.close()
                except:
                    pass
            client = None
            current_state['plc_connected'] = False
            time.sleep(0.1)

# ==========================================
# 5. GUI Class
# ==========================================

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.configure(bg='#f5f5f5')
        
        self.dragging = False
        self.drag_start_time = 0
        self.last_mouse_pos = None
        self.mouse_xy = (0.0, 0.0)
        self.z_control_mode = False
        
        self.setup_ui()
        self.setup_plot()
        
        # Start Threads
        self.ik_thread = threading.Thread(target=ik_worker, daemon=True)
        self.ik_thread.start()
        
        self.scheduler_thread = threading.Thread(target=command_scheduler, daemon=True)
        self.scheduler_thread.start()
        
        self.plc_thread = threading.Thread(target=plc_worker, daemon=True)
        self.plc_thread.start()
        
        self.update_kinematics()
        self.update_display()

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill='both', expand=True, padx=15, pady=15)

        header_container = tk.Frame(self.main_frame, bg='#ffffff', relief='flat', bd=0)
        header_container.pack(fill='x', pady=(0, 15))
        
        header = tk.Frame(header_container, bg='#ffffff')
        header.pack(fill='x', padx=15, pady=12)
        
        tk.Label(header, text="🤖 SCARA Control Center", 
                font=('Segoe UI', 16, 'bold'), bg='#ffffff', 
                fg='#1a1a1a').pack(side='left')
        
        status_frame = tk.Frame(header, bg='#ffffff')
        status_frame.pack(side='right')
        
        self.status_dot = tk.Canvas(status_frame, width=14, height=14, 
                                   bg='#ffffff', highlightthickness=0)
        self.status_dot.pack(side='left', padx=(0, 8))
        self.status_circle = self.status_dot.create_oval(2, 2, 12, 12, 
                                                         fill='#9e9e9e', outline='')
        
        self.lbl_status = tk.Label(status_frame, text="Connecting...", 
                                  fg="#757575", font=("Segoe UI", 10), bg='#ffffff')
        self.lbl_status.pack(side='left')

        content = ttk.Frame(self.main_frame)
        content.pack(fill='both', expand=True)
        
        plot_container = tk.Frame(content, bg='#ffffff', relief='flat')
        plot_container.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        self.plot_frame = ttk.Frame(plot_container)
        self.plot_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        z_container = tk.Frame(content, bg='#ffffff', relief='flat', width=130)
        z_container.pack(side='right', fill='y')
        z_container.pack_propagate(False)
        
        self.z_frame = ttk.Frame(z_container)
        self.z_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        input_container = tk.Frame(content, bg='#ffffff', relief='flat', width=200)
        input_container.pack(side='right', fill='y', padx=(10, 0))
        input_container.pack_propagate(False)
        
        input_header = tk.Frame(input_container, bg='#1976d2', height=40)
        input_header.pack(fill='x')
        input_header.pack_propagate(False)
        
        tk.Label(input_header, text="📝 Manual Input", 
                font=('Segoe UI', 11, 'bold'), bg='#1976d2', 
                fg='white').pack(expand=True)
        
        input_form = tk.Frame(input_container, bg='#ffffff')
        input_form.pack(fill='both', expand=True, padx=12, pady=12)
        
        tk.Label(input_form, text="X (mm):", font=('Segoe UI', 9), 
                bg='#ffffff', fg='#424242').pack(anchor='w', pady=(0, 2))
        self.entry_x = ttk.Entry(input_form, font=('Consolas', 10))
        self.entry_x.pack(fill='x', pady=(0, 10))
        self.entry_x.insert(0, "0.0")
        
        tk.Label(input_form, text="Y (mm):", font=('Segoe UI', 9), 
                bg='#ffffff', fg='#424242').pack(anchor='w', pady=(0, 2))
        self.entry_y = ttk.Entry(input_form, font=('Consolas', 10))
        self.entry_y.pack(fill='x', pady=(0, 10))
        self.entry_y.insert(0, "0.0")
        
        tk.Label(input_form, text="Z (mm):", font=('Segoe UI', 9), 
                bg='#ffffff', fg='#424242').pack(anchor='w', pady=(0, 2))
        self.entry_z = ttk.Entry(input_form, font=('Consolas', 10))
        self.entry_z.pack(fill='x', pady=(0, 10))
        self.entry_z.insert(0, f"{Z_HOME*1000:.1f}")
        
        btn_frame = tk.Frame(input_form, bg='#ffffff')
        btn_frame.pack(fill='x', pady=(10, 0))
        
        self.btn_go = tk.Button(btn_frame, text="▶ GO", font=('Segoe UI', 10, 'bold'),
                               bg='#43a047', fg='white', relief='flat', cursor='hand2',
                               command=self.on_manual_go, height=2)
        self.btn_go.pack(fill='x', pady=(0, 5))
        
        self.btn_home = tk.Button(btn_frame, text="🏠 HOME", font=('Segoe UI', 9),
                                 bg='#ff9800', fg='white', relief='flat', cursor='hand2',
                                 command=self.on_home)
        self.btn_home.pack(fill='x', pady=(0, 5))
        
        self.btn_get = tk.Button(btn_frame, text="📍 Get Current", font=('Segoe UI', 9),
                                bg='#1976d2', fg='white', relief='flat', cursor='hand2',
                                command=self.on_get_current)
        self.btn_get.pack(fill='x')
        
        self.lbl_input_status = tk.Label(input_form, text="", 
                                         font=('Segoe UI', 8), bg='#ffffff', 
                                         fg='#757575', wraplength=180)
        self.lbl_input_status.pack(pady=(10, 0))

        info_container = tk.Frame(self.main_frame, bg='#ffffff', relief='flat')
        info_container.pack(fill='x', pady=(15, 0))
        
        info_inner = tk.Frame(info_container, bg='#ffffff')
        info_inner.pack(fill='x', padx=15, pady=12)
        
        coord_frame = tk.Frame(info_inner, bg='#ffffff')
        coord_frame.pack(side='left', fill='x', expand=True)
        
        self.lbl_coords = tk.Label(coord_frame, text="📍 Ready", 
                                  font=("Consolas", 11), bg="#ffffff", 
                                  fg='#424242', anchor='w')
        self.lbl_coords.pack(fill='x')
        
        self.lbl_endeff = tk.Label(coord_frame, text="🎯 End Effector: ---", 
                                   font=("Consolas", 10), bg="#ffffff", 
                                   fg='#1976d2', anchor='w')
        self.lbl_endeff.pack(fill='x', pady=(2, 0))
        
        self.lbl_mouse = tk.Label(coord_frame, text="🖱️ Mouse: ---", 
                                 font=("Consolas", 9), bg="#ffffff", 
                                 fg='#757575', anchor='w')
        self.lbl_mouse.pack(fill='x', pady=(2, 0))
        
        self.lbl_mode = tk.Label(info_inner, text="🎯 Click & Drag to Move", 
                                font=("Segoe UI", 9), bg="#ffffff", 
                                fg='#757575', anchor='e')
        self.lbl_mode.pack(side='right', padx=(10, 0))
        
        self.lbl_speed = tk.Label(info_inner, text=f"⚡ Z-Speed: {current_state['z_speed']}", 
                                 font=("Segoe UI", 9), bg="#ffffff", 
                                 fg='#1976d2', anchor='e')
        self.lbl_speed.pack(side='right', padx=(10, 0))

    def setup_plot(self):
        self.fig = Figure(figsize=(5.5, 5.5), dpi=100, facecolor='#ffffff')
        self.ax = self.fig.add_subplot(111, facecolor='#fafafa')
        self.ax.set_title("Top View - XY Plane", fontsize=11, pad=15, 
                         fontweight='600', color='#1a1a1a')
        
        # Plot limit includes full reach
        r_view = R_MAX + 0.1
        self.ax.set_xlim(-r_view, r_view)
        self.ax.set_ylim(-r_view, r_view)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle=':', alpha=0.3, color='#bdbdbd', linewidth=0.8)
        self.ax.set_xlabel('X (m)', fontsize=9, color='#616161')
        self.ax.set_ylabel('Y (m)', fontsize=9, color='#616161')
        
        # Draw Workspace (Corrected to L2 & L3 based on README)
        workspace_inner = Circle((0, 0), R_MIN, color='#ffebee', 
                                fill=True, alpha=0.3, zorder=0)
        workspace_outer = Circle((0, 0), R_MAX, color='#e3f2fd', 
                                fill=True, alpha=0.3, zorder=0)
        self.ax.add_patch(workspace_inner)
        self.ax.add_patch(workspace_outer)
        
        # Outer boundary line
        self.ax.add_patch(Circle((0, 0), R_MAX, color='#1976d2', 
                                fill=False, ls='--', lw=2, alpha=0.4))
        
        self.line_arm, = self.ax.plot([], [], 'o-', lw=7, color='#1565c0', 
                                     ms=11, mfc='#ffffff', mew=2.5, 
                                     mec='#1565c0', zorder=5, solid_capstyle='round')
        
        self.pt_cursor, = self.ax.plot([], [], 'o', ms=14, color='#ff6f00', 
                                      alpha=0.9, zorder=10, mew=2.5, 
                                      mfc='#ff6f00', mec='#ffffff')
        
        self.pt_target, = self.ax.plot([], [], 'o', ms=14, color='#4caf50', 
                                      alpha=0.3, zorder=8, mew=2, 
                                      mfc='none', mec='#4caf50', ls='--')
        
        self.ax.plot([0], [0], 'o', ms=16, color='#212121', zorder=5, 
                    mec='#ffffff', mew=2.5)
        
        self.trail_x = deque(maxlen=60)
        self.trail_y = deque(maxlen=60)
        self.trail_line, = self.ax.plot([], [], '-', lw=2, color='#ff6f00', 
                                       alpha=0.25, zorder=1)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill='both', expand=True)

        self.fig_z = Figure(figsize=(1.8, 5.5), dpi=100, facecolor='#ffffff')
        self.ax_z = self.fig_z.add_subplot(111, facecolor='#fafafa')
        self.ax_z.set_title("Z-Axis", fontsize=10, pad=10, 
                           fontweight='600', color='#1a1a1a')
        # Display Z from Bottom (-80) to Top (0, shown as offset height)
        self.ax_z.set_ylim((Z_BOTTOM - 0.01)*1000, (Z_HOME + 0.01)*1000)
        self.ax_z.set_xlim(0, 1)
        self.ax_z.set_xticks([])
        self.ax_z.set_ylabel('Height (mm)', fontsize=9, color='#616161')
        
        self.ax_z.axhline(Z_BOTTOM*1000, color='#d32f2f', lw=2.5, alpha=0.7, ls='--', zorder=1, label='Min')
        self.ax_z.axhline(Z_HOME*1000, color='#43a047', lw=2.5, alpha=0.7, ls='--', zorder=1, label='Max')
        self.ax_z.grid(True, axis='y', linestyle=':', alpha=0.3, linewidth=0.8)
        
        self.bar_z = self.ax_z.bar([0.5], [0], width=0.75, 
                                   color='#43a047', edgecolor='#2e7d32', 
                                   linewidth=2.5, zorder=3)
        
        self.target_z_line = self.ax_z.axhline(0, color='#ff9800', 
                                              lw=2, ls='--', alpha=0.5, zorder=2)
        
        self.canvas_z = FigureCanvasTkAgg(self.fig_z, master=self.z_frame)
        self.canvas_z.draw()
        self.canvas_z.get_tk_widget().pack(fill='both', expand=True)

        self.canvas.mpl_connect('button_press_event', self.on_mouse_down)
        self.canvas.mpl_connect('button_release_event', self.on_mouse_up)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas_z.mpl_connect('scroll_event', self.on_scroll)
        
        self.root.bind('<Control_L>', self.on_ctrl_press)
        self.root.bind('<Control_R>', self.on_ctrl_press)
        self.root.bind('<KeyRelease-Control_L>', self.on_ctrl_release)
        self.root.bind('<KeyRelease-Control_R>', self.on_ctrl_release)

    def on_mouse_down(self, event):
        if event.button == 1 and event.inaxes == self.ax:
            self.dragging = True
            self.drag_start_time = time.time()
            self.last_mouse_pos = (event.xdata, event.ydata)
            self.lbl_mode.config(text="🎯 Dragging...", fg='#ff6f00')

    def on_mouse_up(self, event):
        if self.dragging:
            self.dragging = False
            self.lbl_mode.config(text="🎯 Click & Drag to Move", fg='#757575')
            if self.last_mouse_pos:
                self.request_ik(self.last_mouse_pos[0], self.last_mouse_pos[1])

    def on_mouse_move(self, event):
        if event.inaxes == self.ax:
            if event.xdata is not None and event.ydata is not None:
                self.mouse_xy = (event.xdata, event.ydata)
            
            if self.dragging:
                self.last_mouse_pos = (event.xdata, event.ydata)
                self.request_ik(event.xdata, event.ydata)

    def request_ik(self, x, y):
        if x is None or y is None:
            return
        
        # Use queue for non-blocking IK request
        try:
            ik_queue.get_nowait()
        except:
            pass
        try:
            ik_queue.put_nowait((x, y, current_state['target_z']))
        except:
            pass

    def on_scroll(self, event):
        if event.key == 'control' or self.z_control_mode:
            step = 50 if event.button == 'up' else -50
            new_speed = current_state['z_speed'] + step
            new_speed = max(50, min(2000, new_speed))
            current_state['z_speed'] = new_speed
            self.lbl_speed.config(text=f"⚡ Z-Speed: {new_speed}")
            self.lbl_mode.config(text="⚡ Z-Speed Adjusted", fg='#1976d2')
        else:
            step = 0.002 if event.button == 'up' else -0.002
            new_z = current_state['target_z'] + step
            new_z = max(Z_BOTTOM, min(Z_HOME, new_z))
            current_state['target_z'] = new_z
            
            # For Z movement, we update target immediately if X,Y is valid
            q = current_state['q_target'].copy()
            q[2] = np.clip(new_z - Z_HOME, *Q_LIMITS['z'])
            current_state['q_target'] = q
            
            with current_state['command_lock']:
                current_state['pending_command'] = q
            
            self.lbl_mode.config(text="↕️ Z-Axis Adjusted", fg='#ff9800')
    
    def on_ctrl_press(self, event):
        self.z_control_mode = True
        self.lbl_mode.config(text="⚡ Speed Control Mode (Scroll to adjust)", fg='#1976d2')
    
    def on_ctrl_release(self, event):
        self.z_control_mode = False
        self.lbl_mode.config(text="🎯 Click & Drag to Move", fg='#757575')
    
    def on_manual_go(self):
        try:
            x = float(self.entry_x.get()) / 1000.0
            y = float(self.entry_y.get()) / 1000.0
            z_abs = float(self.entry_z.get()) / 1000.0
            z_rel = z_abs - Z_HOME
            
            # Strict Workspace Check (As per README)
            if not is_in_workspace(x, y, z_rel):
                dist = np.sqrt(x**2 + y**2)
                if z_rel < Q_LIMITS['z'][0] or z_rel > Q_LIMITS['z'][1]:
                    msg = f"❌ Z out of range!\nRange: {Z_BOTTOM*1000:.1f} to {Z_HOME*1000:.1f}"
                else:
                    msg = f"❌ Out of workspace!\nR={dist*1000:.1f}mm\n(Min {R_MIN*1000:.0f}, Max {R_MAX*1000:.0f})"
                
                self.lbl_input_status.config(text=msg, fg='#e53935')
                return
            
            T = SE3(x, y, z_rel)
            sol = robot.ikine_LM(T, q0=current_state['q_target'], mask=[1, 1, 1, 0, 0, 0])
            
            if sol.success:
                q_new = sol.q.copy()
                q_new[0] = np.clip(q_new[0], *Q_LIMITS['j1'])
                q_new[1] = np.clip(q_new[1], *Q_LIMITS['j2'])
                q_new[2] = np.clip(z_rel, *Q_LIMITS['z'])
                
                current_state['q_target'] = q_new
                current_state['target_z'] = z_abs
                
                with current_state['command_lock']:
                    current_state['pending_command'] = q_new
                    current_state['last_command_time'] = 0
                
                self.lbl_input_status.config(
                    text=f"✓ Command sent!\nJ1={np.rad2deg(q_new[0]):.1f}° J2={np.rad2deg(q_new[1]):.1f}°",
                    fg='#43a047'
                )
            else:
                self.lbl_input_status.config(
                    text="❌ IK solution failed!",
                    fg='#e53935'
                )
                
        except ValueError:
            self.lbl_input_status.config(
                text="❌ Invalid input!\nEnter numbers only",
                fg='#e53935'
            )
        except Exception as e:
            self.lbl_input_status.config(
                text=f"❌ Error: {str(e)[:30]}",
                fg='#e53935'
            )
    
    def on_home(self):
        # Home position: L1+L2 (Max reach) in X, 0 in Y, Z_HOME
        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, f"{(L2+L3)*1000:.1f}") # Extended arm
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, "0.0")
        self.entry_z.delete(0, tk.END)
        self.entry_z.insert(0, f"{Z_HOME*1000:.1f}")
        self.on_manual_go()
    
    def on_get_current(self):
        q = current_state['q_display']
        
        # Forward Kinematics for display
        # Note: L2 is Link 1 (Horiz), L3 is Link 2 (Horiz)
        x = (L2 * np.cos(q[0]) + L3 * np.cos(q[0] + q[1])) * 1000
        y = (L2 * np.sin(q[0]) + L3 * np.sin(q[0] + q[1])) * 1000
        z = (Z_HOME + q[2]) * 1000
        
        self.entry_x.delete(0, tk.END)
        self.entry_x.insert(0, f"{x:.1f}")
        self.entry_y.delete(0, tk.END)
        self.entry_y.insert(0, f"{y:.1f}")
        self.entry_z.delete(0, tk.END)
        self.entry_z.insert(0, f"{z:.1f}")
        
        self.lbl_input_status.config(
            text="✓ Current position loaded",
            fg='#1976d2'
        )

    def update_kinematics(self):
        try:
            if not ik_result_queue.empty():
                q_new = ik_result_queue.get()
                current_state['q_target'] = q_new
                
                # Only update command if not dragging (to avoid flood) 
                # OR update if it's the final position
                with current_state['command_lock']:
                    current_state['pending_command'] = q_new
                        
        except Exception as e:
            print(f"Kinematics Error: {e}")
        
        self.root.after(20, self.update_kinematics)

    def update_display(self):
        try:
            alpha = 0.25
            
            if current_state['plc_connected']:
                target_q = current_state['q_actual']
            else:
                target_q = current_state['q_target']
            
            current_state['q_display'] = (1 - alpha) * current_state['q_display'] + \
                                        alpha * target_q
            
            if current_state['plc_connected']:
                self.status_dot.itemconfig(self.status_circle, fill='#43a047')
                self.lbl_status.config(text="Connected", fg="#43a047")
                mode_text = ""
            else:
                self.status_dot.itemconfig(self.status_circle, fill='#ff9800')
                self.lbl_status.config(text="Simulation", fg="#ff9800")
                mode_text = " [SIM]"

            q = current_state['q_display']
            
            # Forward Kinematics Visualization
            # L2 = Arm 1, L3 = Arm 2
            x1 = L2 * np.cos(q[0])
            y1 = L2 * np.sin(q[0])
            x2 = x1 + L3 * np.cos(q[0] + q[1])
            y2 = y1 + L3 * np.sin(q[0] + q[1])
            
            self.line_arm.set_data([0, x1, x2], [0, y1, y2])
            self.pt_cursor.set_data([x2], [y2])
            
            if self.dragging and self.last_mouse_pos:
                self.pt_target.set_data([self.last_mouse_pos[0]], [self.last_mouse_pos[1]])
            else:
                self.pt_target.set_data([], [])
            
            self.trail_x.append(x2)
            self.trail_y.append(y2)
            self.trail_line.set_data(list(self.trail_x), list(self.trail_y))
            
            z_display = (Z_HOME + current_state['q_target'][2]) * 1000
            self.bar_z[0].set_height(z_display)
            self.target_z_line.set_ydata([z_display, z_display])
            
            # Color coding for Z
            if z_display < 20:
                self.bar_z[0].set_color('#e53935') # Low
            elif z_display > (Z_HOME*1000 - 20):
                self.bar_z[0].set_color('#43a047') # High
            else:
                self.bar_z[0].set_color('#fb8c00') # Mid
            
            self.canvas.draw_idle()
            self.canvas_z.draw_idle()

            d1 = np.rad2deg(q[0])
            d2 = np.rad2deg(q[1])
            zm = q[2] * 1000
            
            at_limit = abs(d1) >= 89 or abs(d2) >= 89
            
            if at_limit:
                icon = "⚠️"
                color = '#e53935'
            else:
                icon = "📍"
                color = '#424242'
                
            self.lbl_coords.config(
                text=f"{icon} J1: {d1:6.2f}° │ J2: {d2:6.2f}° │ Z: {zm:6.1f}mm{mode_text}",
                fg=color
            )
            
            mx, my = self.mouse_xy
            # Calculate robot frame Z for checking
            current_z_rel = current_state['target_z'] - Z_HOME
            in_workspace = is_in_workspace(mx, my, current_z_rel)
            
            dist = np.sqrt(mx**2 + my**2)
            
            if in_workspace:
                mouse_color = '#43a047'
                workspace_status = "✓"
            else:
                mouse_color = '#e53935'
                workspace_status = "✗"
            
            self.lbl_mouse.config(
                text=f"🖱️ Mouse: X={mx*1000:6.1f}mm Y={my*1000:6.1f}mm R={dist*1000:6.1f}mm {workspace_status}",
                fg=mouse_color
            )

        except Exception as e:
            print(f"Display Error: {e}")

        self.root.after(REFRESH_RATE_MS, self.update_display)

    def on_close(self):
        print("\n🛑 Shutting down...")
        current_state['running'] = False
        time.sleep(0.2)
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    root.title("SCARA Control System v2.0")
    root.geometry("1150x680")
    root.configure(bg='#f5f5f5')
    root.minsize(1100, 650)
    
    app = RobotGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    
    print("=" * 50)
    print("🤖 SCARA Control System v2.0")
    print("=" * 50)
    print(f"PLC Target: {PLC_IP}:{PLC_PORT}")
    root.mainloop()
