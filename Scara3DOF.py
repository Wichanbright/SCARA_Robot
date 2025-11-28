import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from pymodbus.client import ModbusTcpClient
import roboticstoolbox as rtb
from spatialmath import SE3
from roboticstoolbox.backends.PyPlot import PyPlot
import numpy as np
import math

# ==========================================
# 1. การตั้งค่า (Configuration)
# ==========================================
PLC_IP = '192.168.1.250'
PLC_PORT = 502

# Parameters (เมตร)
L1, L2, L3, L4 = 0.13, 0.115, 0.13, 0.014

# Limits (Radians)
deg90 = np.deg2rad(90)
Q1_MIN, Q1_MAX = -deg90, deg90      # -90 ถึง 90
Q2_MIN, Q2_MAX = -deg90, deg90      # -90 ถึง 90
Q3_MIN, Q3_MAX = -0.15, 0.0

Z_HOME = L1 + L4
Z_BOTTOM = Z_HOME + Q3_MIN

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
env = None
prev_q = np.array([0.0, 0.0, 0.0])
target_pos = {'x': 0.15, 'y': 0.1, 'z': Z_HOME}

# ==========================================
# 2. Logic: Mouse & Limits
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

def update_robot_loop():
    global prev_q
    try:
        # Step Simulation
        if env: env.step(0.01)

        # ข้อมูลปัจจุบันของหุ่น
        curr_q = robot.q
        deg_j1 = np.rad2deg(curr_q[0])
        deg_j2 = np.rad2deg(curr_q[1])
        pos_z_mm = curr_q[2] * 1000

        # เช็คว่าชนขอบไหม (เพื่อเปลี่ยนสีตัวหนังสือเตือน)
        status_color = "black"
        status_text = "Status: OK"
        if abs(deg_j1) >= 89.9 or abs(deg_j2) >= 89.9:
            status_color = "red"
            status_text = "⚠️ Limit Hit (Clamped)"

        # อัปเดต GUI Label
        lbl_info.config(
            text=f"J1: {deg_j1:.1f}°  J2: {deg_j2:.1f}°  Z: {pos_z_mm:.1f}mm", 
            fg=status_color
        )
        lbl_status.config(text=status_text, fg=status_color)
        
        # ส่ง Modbus ไป PLC
        if client and client.connected:
            pos_data = [
                int((curr_q[0]/math.pi)*800*3.75), 
                int((curr_q[1]/math.pi)*800*3.75), 
                int(abs(curr_q[2]*800))
            ]
            try: client.write_registers(0, pos_data)
            except: pass

    except Exception as e:
        print(e)
        
    root.after(50, update_robot_loop)

def on_mouse_event(event):
    """ฟังก์ชันจัดการเมื่อคลิกหรือลากเมาส์"""
    # ตรวจสอบว่าคลิกซ้าย (button 1) และอยู่ในกราฟ XY
    if event.button == 1 and event.inaxes == ax_xy:
        raw_x, raw_y = event.xdata, event.ydata
        
        # 1. คำนวณ IK หา Joint ที่ควรจะเป็นเพื่อให้ไปถึงจุดเมาส์
        T_try = SE3(raw_x, raw_y, target_pos['z'])
        sol = robot.ikine_LM(T_try, q0=robot.q, mask=[1, 1, 1, 0, 0, 0])
        
        if sol.success:
            q_proposed = sol.q
            
            # 2. *** สำคัญมาก: Clamp Joint Limits ***
            # ถ้าคำนวณแล้วเกิน 90 ให้ตัดเหลือ 90 ทันที
            q_proposed[0] = np.clip(q_proposed[0], Q1_MIN, Q1_MAX)
            q_proposed[1] = np.clip(q_proposed[1], Q2_MIN, Q2_MAX)
            # q3 ไม่ต้อง clip ที่นี่เพราะเราปรับ Z ผ่าน scroll แยกต่างหาก
            
            # 3. อัปเดตหุ่นยนต์ด้วยค่าที่ถูกตัดแล้ว (Safe Pose)
            robot.q = q_proposed
            
            # 4. คำนวณพิกัดจริงที่หุ่นไปหยุด (Forward Kinematics)
            # เพื่อเอามาวาดจุดแดง (cursor) ให้ตรงกับหัวหุ่นจริงๆ
            real_pos = robot.fkine(q_proposed).t
            
            target_pos['x'] = real_pos[0]
            target_pos['y'] = real_pos[1]
            
            # ย้ายจุดแดงไปที่หัวหุ่นจริง
            cursor_point.set_data([real_pos[0]], [real_pos[1]])
            canvas.draw_idle()

def on_scroll(event):
    """หมุนเมาส์เพื่อปรับแกน Z"""
    step = 0.005
    if event.button == 'up': target_pos['z'] += step
    elif event.button == 'down': target_pos['z'] -= step
    
    # ลิมิตแกน Z
    target_pos['z'] = max(Z_BOTTOM, min(Z_HOME, target_pos['z']))
    
    # อัปเดตหุ่นทันที (เฉพาะแกน Z)
    q_curr = robot.q
    q_curr[2] = target_pos['z'] - Z_HOME # แปลงความสูงเป็นระยะยืดหด (offset)
    # Clip Z joint limit
    q_curr[2] = np.clip(q_curr[2], Q3_MIN, Q3_MAX)
    robot.q = q_curr
    
    # อัปเดตกราฟแท่ง Z
    z_bar[0].set_height(target_pos['z'] * 1000)
    canvas_z.draw_idle()

# ==========================================
# 3. GUI Setup
# ==========================================
print("🚀 Initializing Click & Drag Control...")

# Sim Setup
env = PyPlot()
env.launch(limits=[-0.4, 0.4, -0.4, 0.4, -0.1, 0.5])
env.add(robot)
env.ax.view_init(elev=20, azim=45)

# Tkinter Window
root = tk.Tk()
root.title("SCARA Control (Click & Drag)")
root.geometry("750x600")
root.attributes('-topmost', True)

# --- Header ---
frame_top = tk.Frame(root, pady=10)
frame_top.pack(fill='x', padx=10)
tk.Label(frame_top, text="Click & Drag Control (Limit +/-90)", font=("Arial", 14, "bold")).pack(side='left')
btn_connect = tk.Button(frame_top, text="Connect PLC", command=connect_plc)
btn_connect.pack(side='right')
lbl_conn_status = tk.Label(frame_top, text="Offline", fg="gray")
lbl_conn_status.pack(side='right', padx=10)

# --- Visualization ---
frame_viz = tk.Frame(root)
frame_viz.pack(expand=True, fill='both', padx=10)

# 1. XY Plot (พื้นที่ทำงาน)
fig_xy = Figure(figsize=(4, 4), dpi=100)
ax_xy = fig_xy.add_subplot(111)
ax_xy.set_title("Top View: Click & Drag to Move")
limit_r = L1 + L2 + 0.05
ax_xy.set_xlim(-limit_r, limit_r)
ax_xy.set_ylim(-limit_r, limit_r)
ax_xy.set_aspect('equal')
ax_xy.grid(True, linestyle=':', alpha=0.5)

# วาดวงกลมระยะเอื้อมสูงสุด (Reference)
circle = Circle((0, 0), L1+L2, color='gray', fill=False, linestyle='--', alpha=0.3)
ax_xy.add_patch(circle)

# จุด Origin และจุด Cursor
ax_xy.plot([0], [0], 'k+') 
cursor_point, = ax_xy.plot([0.15], [0.1], 'ro', markeredgecolor='white', markersize=8)

canvas = FigureCanvasTkAgg(fig_xy, master=frame_viz)
canvas.draw()
canvas.get_tk_widget().pack(side='left', expand=True, fill='both')

# 2. Z Plot (ความสูง)
fig_z = Figure(figsize=(1.5, 4), dpi=100)
ax_z = fig_z.add_subplot(111)
ax_z.set_title("Height Z\n(Scroll)")
ax_z.set_ylim(-20, 220)
ax_z.set_xlim(0, 1)
ax_z.set_xticks([])
ax_z.grid(True, axis='y')
ax_z.axhline(0, color='black')
z_bar = ax_z.bar([0.5], [Z_HOME*1000], width=0.5, color='green', alpha=0.6)

canvas_z = FigureCanvasTkAgg(fig_z, master=frame_viz)
canvas_z.draw()
canvas_z.get_tk_widget().pack(side='right', fill='y', padx=5)

# --- Bind Mouse Events ---
# เชื่อมเหตุการณ์คลิก (button_press) และลาก (motion_notify) เข้ากับฟังก์ชันเดียวกัน
canvas.mpl_connect('button_press_event', on_mouse_event)
canvas.mpl_connect('motion_notify_event', on_mouse_event)
canvas.mpl_connect('scroll_event', on_scroll)
canvas_z.mpl_connect('scroll_event', on_scroll)

# --- Footer Info ---
frame_info = tk.Frame(root, bg="#f0f0f0", bd=2, relief="groove")
frame_info.pack(fill='x', padx=10, pady=10)

lbl_status = tk.Label(frame_info, text="Status: Ready", font=("Arial", 12, "bold"), bg="#f0f0f0", fg="green")
lbl_status.pack(pady=2)

lbl_info = tk.Label(frame_info, text="Initializing...", font=("Consolas", 12), bg="#f0f0f0")
lbl_info.pack(pady=2)

# Start Loop
# ตั้งท่าเริ่มต้น
robot.q = robot.ikine_LM(SE3(0.15, 0.1, Z_HOME)).q 
root.after(100, update_robot_loop)
root.mainloop()