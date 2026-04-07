import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button, TextBox
import matplotlib.gridspec as gridspec

class Robot3D:
    def __init__(self, L1=3, L2=4, L3=2):
        """
        Robot lengan 3 DOF dengan konfigurasi articulated (RRR)
        L1: panjang link 1 (dari base ke shoulder)
        L2: panjang link 2 (upper arm)
        L3: panjang link 3 (forearm + end-effector)
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        
        # Sudut joint (radian) - nilai default
        self.theta1 = np.radians(30)  # base rotation (yaw)
        self.theta2 = np.radians(45)  # shoulder (pitch)
        self.theta3 = np.radians(30)  # elbow (pitch)
        
        # Posisi end-effector hasil forward kinematic
        self.ee_x, self.ee_y, self.ee_z = self.calculate_end_effector()
    
    # ========== RUMUS FORWARD KINEMATIC ==========
    def dh_transform(self, theta, alpha, a, d):
        """Matriks transformasi Denavit-Hartenberg"""
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,             np.sin(alpha),                np.cos(alpha),                d],
            [0,             0,                            0,                            1]
        ])
    
    def calculate_end_effector(self):
        """Hitung posisi end-effector dari sudut saat ini"""
        # Parameter DH untuk 3 DOF articulated robot
        T01 = self.dh_transform(self.theta1, np.pi/2, 0, self.L1)
        T12 = self.dh_transform(self.theta2, 0, self.L2, 0)
        T23 = self.dh_transform(self.theta3, 0, self.L3, 0)
        
        # Transformasi total
        T03 = T01 @ T12 @ T23
        
        return T03[0, 3], T03[1, 3], T03[2, 3]
    
    def forward_kinematic(self, theta1, theta2, theta3):
        """Forward kinematic: sudut -> posisi"""
        T01 = self.dh_transform(theta1, np.pi/2, 0, self.L1)
        T12 = self.dh_transform(theta2, 0, self.L2, 0)
        T23 = self.dh_transform(theta3, 0, self.L3, 0)
        T03 = T01 @ T12 @ T23
        
        j0 = np.array([0, 0, 0])
        j1 = np.array([0, 0, self.L1])
        j2 = T01[:3, 3]
        end = T03[:3, 3]
        
        return j0, j1, j2, end
    
    # ========== RUMUS INVERSE KINEMATIC ==========
    def inverse_kinematic(self, x, y, z, config='elbow_down'):
        """
        Inverse kinematic: posisi -> sudut
        config: 'elbow_down' atau 'elbow_up'
        """
        # Cek jangkauan
        r_xy = np.sqrt(x**2 + y**2)
        r_total = np.sqrt(x**2 + y**2 + (z - self.L1)**2)
        
        if r_total > self.L2 + self.L3 + 0.01 or r_total < abs(self.L2 - self.L3) - 0.01:
            return None, None, None  # Target tidak terjangkau
        
        # Theta1 (base rotation)
        theta1 = np.arctan2(y, x)
        
        # Untuk theta2 dan theta3, proyeksikan ke bidang 2D (r, z)
        r = np.sqrt(x**2 + y**2)
        z_prime = z - self.L1
        
        # Hukum cosinus untuk theta3
        cos_theta3 = (r**2 + z_prime**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_theta3 = np.clip(cos_theta3, -1, 1)
        
        if config == 'elbow_down':
            theta3 = np.arccos(cos_theta3)
        else:
            theta3 = -np.arccos(cos_theta3)
        
        # Hitung theta2
        alpha = np.arctan2(z_prime, r)
        beta = np.arctan2(self.L3 * np.sin(theta3), self.L2 + self.L3 * np.cos(theta3))
        theta2 = alpha - beta
        
        return theta1, theta2, theta3
    
    def update_from_angles(self, theta1_deg, theta2_deg, theta3_deg):
        """Update robot dari sudut (mode forward)"""
        self.theta1 = np.radians(theta1_deg)
        self.theta2 = np.radians(theta2_deg)
        self.theta3 = np.radians(theta3_deg)
        self.ee_x, self.ee_y, self.ee_z = self.calculate_end_effector()
        return self.ee_x, self.ee_y, self.ee_z
    
    def update_from_position(self, x, y, z, config='elbow_down'):
        """Update robot dari posisi (mode inverse)"""
        theta1, theta2, theta3 = self.inverse_kinematic(x, y, z, config)
        if theta1 is not None:
            self.theta1 = theta1
            self.theta2 = theta2
            self.theta3 = theta3
            self.ee_x, self.ee_y, self.ee_z = self.calculate_end_effector()
            return True, np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)
        return False, 0, 0, 0


# ========== PROGRAM UTAMA GABUNGAN ==========
class RobotSimulator:
    def __init__(self):
        self.robot = Robot3D()
        self.current_mode = 'forward'  # 'forward' atau 'inverse'
        
        # Setup figure dengan 2 subplot
        self.fig = plt.figure(figsize=(14, 10))
        
        # Layout: 3D view di kiri, panel info di kanan
        gs = gridspec.GridSpec(2, 2, width_ratios=[1.5, 1], height_ratios=[3, 1])
        
        # 3D plot untuk visualisasi robot
        self.ax_3d = self.fig.add_subplot(gs[0, 0], projection='3d')
        
        # Panel informasi (kanan atas)
        self.ax_info = self.fig.add_subplot(gs[0, 1])
        self.ax_info.axis('off')
        
        # Panel kontrol (bawah, menggabungkan kedua kolom)
        self.ax_controls = self.fig.add_subplot(gs[1, :])
        self.ax_controls.axis('off')
        
        self.setup_controls()
        self.update_display()
        
        plt.tight_layout()
        plt.show()
    
    def setup_controls(self):
        """Setup semua kontrol dalam satu panel"""
        # Mode selector buttons
        ax_mode_forward = plt.axes([0.1, 0.08, 0.15, 0.05])
        ax_mode_inverse = plt.axes([0.27, 0.08, 0.15, 0.05])
        
        self.btn_forward = Button(ax_mode_forward, 'Mode: FORWARD')
        self.btn_inverse = Button(ax_mode_inverse, 'Mode: INVERSE')
        
        self.btn_forward.on_clicked(self.set_forward_mode)
        self.btn_inverse.on_clicked(self.set_inverse_mode)
        
        # ===== FORWARD MODE CONTROLS (slider untuk sudut) =====
        # Slider theta1
        ax_theta1 = plt.axes([0.55, 0.20, 0.35, 0.03])
        self.slider_theta1 = Slider(ax_theta1, r'$\theta_1$ (deg)', -180, 180, 
                                     valinit=np.degrees(self.robot.theta1))
        
        # Slider theta2
        ax_theta2 = plt.axes([0.55, 0.15, 0.35, 0.03])
        self.slider_theta2 = Slider(ax_theta2, r'$\theta_2$ (deg)', -90, 90, 
                                     valinit=np.degrees(self.robot.theta2))
        
        # Slider theta3
        ax_theta3 = plt.axes([0.55, 0.10, 0.35, 0.03])
        self.slider_theta3 = Slider(ax_theta3, r'$\theta_3$ (deg)', -135, 135, 
                                     valinit=np.degrees(self.robot.theta3))
        
        # ===== INVERSE MODE CONTROLS (textbox untuk koordinat) =====
        # TextBox untuk X, Y, Z
        ax_x = plt.axes([0.55, 0.28, 0.08, 0.04])
        self.text_x = TextBox(ax_x, 'X:', initial=f"{self.robot.ee_x:.2f}")
        
        ax_y = plt.axes([0.65, 0.28, 0.08, 0.04])
        self.text_y = TextBox(ax_y, 'Y:', initial=f"{self.robot.ee_y:.2f}")
        
        ax_z = plt.axes([0.75, 0.28, 0.08, 0.04])
        self.text_z = TextBox(ax_z, 'Z:', initial=f"{self.robot.ee_z:.2f}")
        
        # Button untuk update inverse
        ax_update = plt.axes([0.85, 0.28, 0.08, 0.04])
        self.btn_update = Button(ax_update, 'Update')
        self.btn_update.on_clicked(self.update_inverse)
        
        # Elbow config selector
        ax_elbow = plt.axes([0.55, 0.24, 0.15, 0.04])
        self.btn_elbow = Button(ax_elbow, 'Elbow: DOWN')
        self.elbow_config = 'elbow_down'
        self.btn_elbow.on_clicked(self.toggle_elbow)
        
        # Reset button
        ax_reset = plt.axes([0.55, 0.05, 0.15, 0.04])
        self.btn_reset = Button(ax_reset, 'Reset')
        self.btn_reset.on_clicked(self.reset)
        
        # Hubungkan event
        self.slider_theta1.on_changed(self.update_forward)
        self.slider_theta2.on_changed(self.update_forward)
        self.slider_theta3.on_changed(self.update_forward)
        
        # Sembunyikan kontrol inverse dulu
        self.set_forward_mode(None)
    
    def set_forward_mode(self, event):
        """Aktifkan mode forward kinematic"""
        self.current_mode = 'forward'
        self.btn_forward.color = 'lightgreen'
        self.btn_inverse.color = 'lightgray'
        
        # Tampilkan slider, sembunyikan textbox inverse
        self.slider_theta1.ax.set_visible(True)
        self.slider_theta2.ax.set_visible(True)
        self.slider_theta3.ax.set_visible(True)
        self.text_x.ax.set_visible(False)
        self.text_y.ax.set_visible(False)
        self.text_z.ax.set_visible(False)
        self.btn_update.ax.set_visible(False)
        self.btn_elbow.ax.set_visible(False)
        
        plt.draw()
    
    def set_inverse_mode(self, event):
        """Aktifkan mode inverse kinematic"""
        self.current_mode = 'inverse'
        self.btn_forward.color = 'lightgray'
        self.btn_inverse.color = 'lightgreen'
        
        # Update textbox dengan posisi saat ini
        self.text_x.set_val(f"{self.robot.ee_x:.2f}")
        self.text_y.set_val(f"{self.robot.ee_y:.2f}")
        self.text_z.set_val(f"{self.robot.ee_z:.2f}")
        
        # Sembunyikan slider, tampilkan textbox inverse
        self.slider_theta1.ax.set_visible(False)
        self.slider_theta2.ax.set_visible(False)
        self.slider_theta3.ax.set_visible(False)
        self.text_x.ax.set_visible(True)
        self.text_y.ax.set_visible(True)
        self.text_z.ax.set_visible(True)
        self.btn_update.ax.set_visible(True)
        self.btn_elbow.ax.set_visible(True)
        
        plt.draw()
    
    def update_forward(self, val):
        """Update dari slider forward kinematic"""
        if self.current_mode != 'forward':
            return
        
        t1 = self.slider_theta1.val
        t2 = self.slider_theta2.val
        t3 = self.slider_theta3.val
        
        self.robot.update_from_angles(t1, t2, t3)
        self.update_display()
    
    def update_inverse(self, event):
        """Update dari input inverse kinematic"""
        if self.current_mode != 'inverse':
            return
        
        try:
            x = float(self.text_x.text)
            y = float(self.text_y.text)
            z = float(self.text_z.text)
            
            success, t1, t2, t3 = self.robot.update_from_position(x, y, z, self.elbow_config)
            
            if success:
                # Update slider values (meskipun tidak terlihat)
                self.slider_theta1.set_val(t1)
                self.slider_theta2.set_val(t2)
                self.slider_theta3.set_val(t3)
                self.update_display()
            else:
                self.show_message("Target di luar jangkauan!", "red")
        except ValueError:
            self.show_message("Masukkan angka yang valid!", "red")
    
    def toggle_elbow(self, event):
        """Toggle konfigurasi elbow (up/down)"""
        if self.elbow_config == 'elbow_down':
            self.elbow_config = 'elbow_up'
            self.btn_elbow.label.set_text('Elbow: UP')
        else:
            self.elbow_config = 'elbow_down'
            self.btn_elbow.label.set_text('Elbow: DOWN')
        
        # Update ulang dengan posisi yang sama
        self.update_inverse(None)
    
    def reset(self, event):
        """Reset ke posisi awal"""
        self.robot.update_from_angles(30, 45, 30)
        self.slider_theta1.set_val(30)
        self.slider_theta2.set_val(45)
        self.slider_theta3.set_val(30)
        
        if self.current_mode == 'inverse':
            self.text_x.set_val(f"{self.robot.ee_x:.2f}")
            self.text_y.set_val(f"{self.robot.ee_y:.2f}")
            self.text_z.set_val(f"{self.robot.ee_z:.2f}")
        
        self.update_display()
    
    def show_message(self, msg, color='black'):
        """Tampilkan pesan sementara"""
        self.ax_info.clear()
        self.ax_info.axis('off')
        self.ax_info.text(0.5, 0.5, msg, transform=self.ax_info.transAxes,
                         fontsize=12, color=color, ha='center', va='center')
        self.fig.canvas.draw_idle()
    
    def update_display(self):
        """Update semua tampilan"""
        # Clear 3D plot
        self.ax_3d.clear()
        
        # Gambar robot
        j0, j1, j2, end = self.robot.forward_kinematic(
            self.robot.theta1, self.robot.theta2, self.robot.theta3
        )
        
        # Gambar link
        self.ax_3d.plot3D([j0[0], j1[0]], [j0[1], j1[1]], [j0[2], j1[2]], 
                          color='blue', linewidth=5)
        self.ax_3d.plot3D([j1[0], j2[0]], [j1[1], j2[1]], [j1[2], j2[2]], 
                          color='blue', linewidth=5)
        self.ax_3d.plot3D([j2[0], end[0]], [j2[1], end[1]], [j2[2], end[2]], 
                          color='blue', linewidth=5)
        
        # Gambar joint
        self.ax_3d.scatter(*j0, color='black', s=100)
        self.ax_3d.scatter(*j1, color='black', s=80)
        self.ax_3d.scatter(*j2, color='black', s=80)
        self.ax_3d.scatter(*end, color='red', s=120, marker='o')
        
        # Setting axis
        self.ax_3d.set_xlim([-8, 8])
        self.ax_3d.set_ylim([-8, 8])
        self.ax_3d.set_zlim([0, 10])
        self.ax_3d.set_xlabel('X')
        self.ax_3d.set_ylabel('Y')
        self.ax_3d.set_zlabel('Z')
        self.ax_3d.set_title('Robot Lengan 3 DOF - Visualisasi 3D')
        self.ax_3d.view_init(elev=25, azim=-60)
        self.ax_3d.grid(True, alpha=0.3)
        
        # Update panel informasi
        self.ax_info.clear()
        self.ax_info.axis('off')
        
        # Mode saat ini
        mode_text = "FORWARD KINEMATIC" if self.current_mode == 'forward' else "INVERSE KINEMATIC"
        self.ax_info.text(0.5, 0.95, mode_text, transform=self.ax_info.transAxes,
                         fontsize=14, weight='bold', ha='center',
                         color='darkgreen' if self.current_mode == 'forward' else 'darkblue')
        
        # Rumus yang digunakan
        info_text = f"""
        ========================================
        
        INPUT (Sudut Joint):
        • θ₁ = {np.degrees(self.robot.theta1):.1f}°
        • θ₂ = {np.degrees(self.robot.theta2):.1f}°
        • θ₃ = {np.degrees(self.robot.theta3):.1f}°
        
        ========================================
        
        FORWARD KINEMATIC (FK):
        • x = L₂·cosθ₁·cosθ₂ + L₃·cosθ₁·cos(θ₂+θ₃)
        • y = L₂·sinθ₁·cosθ₂ + L₃·sinθ₁·cos(θ₂+θ₃)
        • z = L₁ + L₂·sinθ₂ + L₃·sin(θ₂+θ₃)
        
        HASIL FK (End-Effector):
        • x = {self.robot.ee_x:.3f}
        • y = {self.robot.ee_y:.3f}
        • z = {self.robot.ee_z:.3f}
        
        ========================================
        
        INVERSE KINEMATIC (IK):
        • θ₁ = atan2(y, x)
        • θ₃ = acos((r²+z'² - L₂² - L₃²)/(2·L₂·L₃))
        • θ₂ = atan2(z', r) - atan2(L₃·sinθ₃, L₂+L₃·cosθ₃)
        
        PARAMETER ROBOT:
        • L₁ = {self.robot.L1} (base height)
        • L₂ = {self.robot.L2} (upper arm)
        • L₃ = {self.robot.L3} (forearm)
        
        ========================================
        """
        
        self.ax_info.text(0.05, 0.95, info_text, transform=self.ax_info.transAxes,
                         fontsize=9, family='monospace', verticalalignment='top',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        self.fig.canvas.draw_idle()


# ========== VERSI SEDERHANA (TANPA VISUALISASI 3D) ==========
def simple_calculator():
    """Kalkulator sederhana untuk perhitungan FK dan IK"""
    robot = Robot3D()
    
    print("=" * 70)
    print("KALKULATOR FORWARD & INVERSE KINEMATIC ROBOT 3 DOF")
    print("=" * 70)
    print(f"Parameter Robot: L1={robot.L1}, L2={robot.L2}, L3={robot.L3}")
    print("=" * 70)
    
    while True:
        print("\nPilih mode:")
        print("1. Forward Kinematic (sudut → posisi)")
        print("2. Inverse Kinematic (posisi → sudut)")
        print("3. Tampilkan rumus")
        print("4. Keluar")
        
        pilih = input("\nMasukkan pilihan (1/2/3/4): ")
        
        if pilih == '1':
            print("\n--- FORWARD KINEMATIC ---")
            t1 = float(input("Masukkan θ₁ (derajat): "))
            t2 = float(input("Masukkan θ₂ (derajat): "))
            t3 = float(input("Masukkan θ₃ (derajat): "))
            
            robot.update_from_angles(t1, t2, t3)
            
            print("\n" + "=" * 50)
            print("RUMUS FORWARD KINEMATIC:")
            print(f"x = L₂·cosθ₁·cosθ₂ + L₃·cosθ₁·cos(θ₂+θ₃)")
            print(f"  = {robot.L2}·cos({t1}°)·cos({t2}°) + {robot.L3}·cos({t1}°)·cos({t2+t3}°)")
            print(f"  = {robot.ee_x:.4f}")
            print(f"y = L₂·sinθ₁·cosθ₂ + L₃·sinθ₁·cos(θ₂+θ₃)")
            print(f"  = {robot.L2}·sin({t1}°)·cos({t2}°) + {robot.L3}·sin({t1}°)·cos({t2+t3}°)")
            print(f"  = {robot.ee_y:.4f}")
            print(f"z = L₁ + L₂·sinθ₂ + L₃·sin(θ₂+θ₃)")
            print(f"  = {robot.L1} + {robot.L2}·sin({t2}°) + {robot.L3}·sin({t2+t3}°)")
            print(f"  = {robot.ee_z:.4f}")
            print("=" * 50)
            print(f"HASIL: Posisi End-Effector = ({robot.ee_x:.4f}, {robot.ee_y:.4f}, {robot.ee_z:.4f})")
            
        elif pilih == '2':
            print("\n--- INVERSE KINEMATIC ---")
            x = float(input("Masukkan target x: "))
            y = float(input("Masukkan target y: "))
            z = float(input("Masukkan target z: "))
            
            print("\nPilih konfigurasi elbow:")
            print("1. Elbow Down (siku ke bawah)")
            print("2. Elbow Up (siku ke atas)")
            cfg = input("Pilihan (1/2): ")
            config = 'elbow_down' if cfg == '1' else 'elbow_up'
            
            success, t1, t2, t3 = robot.update_from_position(x, y, z, config)
            
            if success:
                print("\n" + "=" * 50)
                print("RUMUS INVERSE KINEMATIC:")
                print(f"θ₁ = atan2(y, x) = atan2({y}, {x}) = {t1:.4f}°")
                print(f"θ₃ = acos((r²+z'² - L₂² - L₃²)/(2·L₂·L₃))")
                print(f"   = {t2:.4f}°")
                print(f"θ₂ = atan2(z', r) - atan2(L₃·sinθ₃, L₂+L₃·cosθ₃)")
                print(f"   = {t3:.4f}°")
                print("=" * 50)
                print(f"HASIL: Sudut joint = ({t1:.2f}°, {t2:.2f}°, {t3:.2f}°)")
            else:
                print("\n[ERROR] Target di luar jangkauan robot!")
                
        elif pilih == '3':
            print("\n" + "=" * 70)
            print("RUMUS LENGKAP KINEMATIK ROBOT 3 DOF")
            print("=" * 70)
            print("\nFORWARD KINEMATIC (FK):")
            print("  x = L₂·cosθ₁·cosθ₂ + L₃·cosθ₁·cos(θ₂+θ₃)")
            print("  y = L₂·sinθ₁·cosθ₂ + L₃·sinθ₁·cos(θ₂+θ₃)")
            print("  z = L₁ + L₂·sinθ₂ + L₃·sin(θ₂+θ₃)")
            print("\nINVERSE KINEMATIC (IK):")
            print("  θ₁ = atan2(y, x)")
            print("  θ₃ = acos((x²+y²+(z-L₁)² - L₂² - L₃²) / (2·L₂·L₃))")
            print("  θ₂ = atan2(z-L₁, √(x²+y²)) - atan2(L₃·sinθ₃, L₂+L₃·cosθ₃)")
            print("\nMATRIKS TRANSFORMASI DH:")
            print("  Tᵢ = [cosθᵢ  -sinθᵢ·cosαᵢ   sinθᵢ·sinαᵢ   aᵢ·cosθᵢ]")
            print("       [sinθᵢ   cosθᵢ·cosαᵢ  -cosθᵢ·sinαᵢ   aᵢ·sinθᵢ]")
            print("       [0       sinαᵢ         cosαᵢ         dᵢ      ]")
            print("       [0       0             0             1       ]")
            print("\nPARAMETER DH:")
            print("  Joint | θ     | α     | a     | d")
            print("  ------|-------|-------|-------|------")
            print(f"  1     | θ₁    | 90°   | 0     | L₁={robot.L1}")
            print(f"  2     | θ₂    | 0°    | L₂={robot.L2} | 0")
            print(f"  3     | θ₃    | 0°    | L₃={robot.L3} | 0")
            print("=" * 70)
            
        elif pilih == '4':
            print("Terima kasih!")
            break
        else:
            print("Pilihan tidak valid!")


# ========== MAIN PROGRAM ==========
if __name__ == "__main__":
    print("=" * 60)
    print("PROGRAM GABUNGAN FORWARD & INVERSE KINEMATIC")
    print("ROBOT LENGAN 3 DOF")
    print("=" * 60)
    print("\nPilih versi program:")
    print("1. Versi Interaktif dengan Visualisasi 3D")
    print("2. Versi Kalkulator (tanpa visualisasi, hanya rumus)")
    
    pilihan = input("\nMasukkan pilihan (1/2): ")
    
    if pilihan == '1':
        RobotSimulator()
    elif pilihan == '2':
        simple_calculator()
    else:
        print("Pilihan tidak valid!")