import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
from pymodbus.client import ModbusTcpClient
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
import math

# ==========================================
# 1. การตั้งค่า (Configuration)
# ==========================================
PLC_IP = '192.168.1.250'
PLC_PORT = 502

# Parameters (เมตร)
L1, L2, L3, L4 = 0.13, 0.115, 0.13, 0.014

# Limits
deg90 = np.deg2rad(90)
Q1_MIN, Q1_MAX = -deg90, deg90
Q2_MIN, Q2_MAX = -deg90, deg90
Q3_MIN, Q3_MAX = -0.15, 0.0

Z_HOME = L1 + L4
Z_BOTTOM = Z_HOME + Q3_MIN

# Robot Model (ใช้คำนวณ IK)
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(a=0, alpha=0, d=L1, qlim=[Q1_MIN, Q1_MAX]),
        rtb.RevoluteMDH(a=L2, alpha=0, d=L4, qlim=[Q2_MIN, Q2_MAX]),
        rtb.PrismaticMDH(a=L3, alpha=0, theta=0, qlim=[Q3_MIN, Q3_MAX]) 
    ], 
    name="MyRobot"
)

# Global Vars
client = None
target_pos = {'x': 0.15, 'y': 0.1, 'z': Z_HOME}

# ==========================================
# 2. Connection & Update Loop
# ==========================================
def connect_plc():
    global client
    lbl_conn_status.config(text="Connecting...", fg="orange")
    root.update()
    try:
        client = ModbusTcpClient(PLC_IP, port=PLC_PORT, timeout=0.5)
        if client.connect():
            lbl_conn_status.config(text="✅ PLC Connected", fg="green")
            btn_connect.config(state="disabled")
        else:
            lbl_conn_status.config(text="❌ Connection Failed", fg="red")
    except:
        lbl_conn_status.config(text="❌ Error", fg="red")

def update_visuals():
    """ฟังก์ชันอัปเดตกราฟิกและส่ง PLC (ทำงานตลอดเวลา)"""
    try:
        curr_q = robot.q
        
        # --- 1. คำนวณตำแหน่งข้อต่อเพื่อวาดแขน (2D Kinematics) ---
        # Base (0,0)
        x0, y0 = 0, 0
        
        # Elbow (ข้อศอก) = L1 * cos(q1), L1 * sin(q1)
        x1 = L1 * np.cos(curr_q[0])
        y1 = L1 * np.sin(curr_q[0])
        
        # End Effector (ปลายมือ) = Elbow + L2 * cos(q1+q2)
        # หมายเหตุ: SCARA มุม q2 สัมพันธ์กับ q1
        x2 = x1 + L2 * np.cos(curr_q[0] + curr_q[1])
        y2 = y1 + L2 * np.sin(curr_q[0] + curr_q[1])

        # อัปเดตเส้นแขน (Base -> Elbow -> End)
        arm_line.set_data([x0, x1, x2], [y0, y1, y2])
        
        # อัปเดตจุดแดง (Target/Actual)
        cursor_point.set_data([x2], [y2])

        # --- 2. อัปเดตตัวเลขและสถานะ ---
        deg_j1 = np.rad2deg(curr_q[0])
        deg_j2 = np.rad2deg(curr_q[1])
        pos_z_mm = curr_q[2] * 1000

        status_color = "black"
        status_text = "Status: OK"
        if abs(deg_j1) >= 89.9 or abs(deg_j2) >= 89.9:
            status_color = "red"
            status_text = "⚠️ Limit Hit (Clamped)"

        lbl_info.config(text=f"J1: {deg_j1:.1f}°  J2: {deg_j2:.1f}°  Z: {pos_z_mm:.1f}mm", fg=status_color)
        lbl_status.config(text=status_text, fg=status_color)

        # --- 3. ส่ง Modbus ---
        if client and client.connected:
            pos_data = [
                int((curr_q[0]/math.pi)*200), 
                int((curr_q[1]/math.pi)*200), 
                int(abs(curr_q[2]*1000))
            ]
            try: client.write_registers(0, pos_data)
            except: pass

    except Exception as e:
        print(f"Error: {e}")

    root.after(50, update_visuals)

# ==========================================
# 3. Mouse Logic
# ==========================================
def on_mouse_event(event):
    if event.button == 1 and event.inaxes == ax_xy:
        raw_x, raw_y = event.xdata, event.ydata
        
        # IK Calculation
        T_try = SE3(raw_x, raw_y, target_pos['z'])
        sol = robot.ikine_LM(T_try, q0=robot.q, mask=[1, 1, 1, 0, 0, 0])
        
        if sol.success:
            q_new = sol.q
            # Clamp Limits
            q_new[0] = np.clip(q_new[0], Q1_MIN, Q1_MAX)
            q_new[1] = np.clip(q_new[1], Q2_MIN, Q2_MAX)
            
            # Update Robot
            robot.q = q_new
            
            # Force redraw immediately
            canvas.draw_idle()

def on_scroll(event):
    step = 0.005
    if event.button == 'up': target_pos['z'] += step
    elif event.button == 'down': target_pos['z'] -= step
    
    target_pos['z'] = max(Z_BOTTOM, min(Z_HOME, target_pos['z']))
    
    # Update Robot Z
    q_curr = robot.q
    q_curr[2] = target_pos['z'] - Z_HOME
    q_curr[2] = np.clip(q_curr[2], Q3_MIN, Q3_MAX)
    robot.q = q_curr
    
    # Update Z Bar
    z_bar[0].set_height(target_pos['z'] * 1000)
    canvas_z.draw_idle()

# ==========================================
# 4. GUI Setup
# ==========================================
print("🚀 Initializing 2D Top View Control...")

root = tk.Tk()
root.title("SCARA Control (2D Top View)")
root.geometry("800x600")
root.attributes('-topmost', True)

# --- Header ---
frame_top = tk.Frame(root, pady=10)
frame_top.pack(fill='x', padx=10)
tk.Label(frame_top, text="SCARA 2D Control", font=("Arial", 16, "bold")).pack(side='left')
btn_connect = tk.Button(frame_top, text="Connect PLC", command=connect_plc, bg="#DDDDDD")
btn_connect.pack(side='right')
lbl_conn_status = tk.Label(frame_top, text="Offline", fg="gray")
lbl_conn_status.pack(side='right', padx=10)

# --- Main Visualization Area ---
frame_viz = tk.Frame(root)
frame_viz.pack(expand=True, fill='both', padx=10)

# 1. Left: XY Top View
fig_xy = Figure(figsize=(5, 5), dpi=100)
ax_xy = fig_xy.add_subplot(111)
ax_xy.set_title("Top View (Robot Skeleton)")
limit_r = L1 + L2 + 0.05
ax_xy.set_xlim(-limit_r, limit_r)
ax_xy.set_ylim(-limit_r, limit_r)
ax_xy.set_aspect('equal')
ax_xy.grid(True, linestyle=':', alpha=0.5)

# วาดเขต Limit (สีเขียวอ่อน) เพื่อความสวยงาม
# สร้าง Wedge (รูปพัด) คร่าวๆ เพื่อแสดงพื้นที่คร่าวๆ (Visual only)
boundary_radius = L1 + L2
circle = Circle((0, 0), boundary_radius, color='gray', fill=False, linestyle='--', alpha=0.3)
ax_xy.add_patch(circle)

# *** Robot Skeleton Elements ***
# เส้นแขน (Base -> Elbow -> End)
arm_line, = ax_xy.plot([], [], 'o-', linewidth=4, markersize=8, color='#007acc', label='Arm')
# จุด Base (สี่เหลี่ยมดำ)
ax_xy.plot([0], [0], 'ks', markersize=10, label='Base')
# จุดปลาย (วงกลมแดง)
cursor_point, = ax_xy.plot([], [], 'ro', markeredgecolor='white', markersize=10, label='End Effector')

ax_xy.legend(loc='upper right', fontsize='small')

canvas = FigureCanvasTkAgg(fig_xy, master=frame_viz)
canvas.draw()
canvas.get_tk_widget().pack(side='left', expand=True, fill='both')

# 2. Right: Z Height Bar
fig_z = Figure(figsize=(1.5, 5), dpi=100)
ax_z = fig_z.add_subplot(111)
ax_z.set_title("Z Height\n(Scroll)")
ax_z.set_ylim(-20, 220)
ax_z.set_xlim(0, 1)
ax_z.set_xticks([])
ax_z.grid(True, axis='y')
ax_z.axhline(0, color='black', linewidth=2)
ax_z.text(0.1, 5, "Floor", fontsize=8)

z_bar = ax_z.bar([0.5], [Z_HOME*1000], width=0.5, color='#4CAF50', alpha=0.8)

canvas_z = FigureCanvasTkAgg(fig_z, master=frame_viz)
canvas_z.draw()
canvas_z.get_tk_widget().pack(side='right', fill='y', padx=5)

# Events
canvas.mpl_connect('button_press_event', on_mouse_event)
canvas.mpl_connect('motion_notify_event', on_mouse_event)
canvas.mpl_connect('scroll_event', on_scroll)
canvas_z.mpl_connect('scroll_event', on_scroll)

# Footer
frame_bot = tk.Frame(root, bg="#f0f0f0", bd=2, relief="groove")
frame_bot.pack(fill='x', padx=10, pady=10)
lbl_status = tk.Label(frame_bot, text="Ready", font=("Arial", 12, "bold"), bg="#f0f0f0", fg="green")
lbl_status.pack()
lbl_info = tk.Label(frame_bot, text="Init...", font=("Consolas", 11), bg="#f0f0f0")
lbl_info.pack()

# Start
robot.q = robot.ikine_LM(SE3(0.15, 0.1, Z_HOME)).q
root.after(100, update_visuals)
root.mainloop()
