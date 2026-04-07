import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import math

class Robot3DOF:
    def __init__(self, link_lengths):
        """
        Inisialisasi robot 3 DOF planar
        
        Args:
            link_lengths: list atau tuple berisi panjang link [L1, L2, L3]
        """
        self.L1, self.L2, self.L3 = link_lengths
        
    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Menghitung posisi setiap joint dan end-effector
        
        Args:
            theta1, theta2, theta3: sudut joint dalam radian
            
        Returns:
            posisi joint0 (base), joint1, joint2, end-effector
        """
        # Joint 0 (base)
        x0, y0 = 0, 0
        
        # Joint 1
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)
        
        # Joint 2
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)
        
        # End-effector (Joint 3)
        x3 = x2 + self.L3 * np.cos(theta1 + theta2 + theta3)
        y3 = y2 + self.L3 * np.sin(theta1 + theta2 + theta3)
        
        return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
    
    def get_end_effector_position(self, theta1, theta2, theta3):
        """Mendapatkan posisi end-effector saja"""
        positions = self.forward_kinematics(theta1, theta2, theta3)
        return positions[-1]

class RobotVisualizer:
    def __init__(self, robot, initial_angles_deg=[30, 45, 60]):
        """
        Inisialisasi visualizer
        
        Args:
            robot: objek Robot3DOF
            initial_angles_deg: sudut awal dalam derajat [theta1, theta2, theta3]
        """
        self.robot = robot
        self.initial_angles_deg = initial_angles_deg
        
        # Konversi ke radian
        self.theta1 = math.radians(initial_angles_deg[0])
        self.theta2 = math.radians(initial_angles_deg[1])
        self.theta3 = math.radians(initial_angles_deg[2])
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Membuat figure dan slider"""
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(left=0.1, bottom=0.35)
        
        # Set judul dan label
        self.ax.set_title('Simulasi Forward Kinematik Robot 3 DOF Planar', fontsize=14, fontweight='bold')
        self.ax.set_xlabel('X (meter)', fontsize=12)
        self.ax.set_ylabel('Y (meter)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Rentang plot
        total_length = self.robot.L1 + self.robot.L2 + self.robot.L3
        margin = 2
        self.ax.set_xlim(-margin, total_length + margin)
        self.ax.set_ylim(-total_length - margin, total_length + margin)
        
        # Buat slider
        slider_color = 'lightblue'
        
        # Slider untuk theta1
        ax_theta1 = plt.axes([0.1, 0.25, 0.8, 0.03])
        self.slider_theta1 = Slider(ax_theta1, 'θ1 (deg)', -180, 180, 
                                    valinit=self.initial_angles_deg[0], 
                                    valstep=1, color=slider_color)
        
        # Slider untuk theta2
        ax_theta2 = plt.axes([0.1, 0.20, 0.8, 0.03])
        self.slider_theta2 = Slider(ax_theta2, 'θ2 (deg)', -180, 180, 
                                    valinit=self.initial_angles_deg[1], 
                                    valstep=1, color=slider_color)
        
        # Slider untuk theta3
        ax_theta3 = plt.axes([0.1, 0.15, 0.8, 0.03])
        self.slider_theta3 = Slider(ax_theta3, 'θ3 (deg)', -180, 180, 
                                    valinit=self.initial_angles_deg[2], 
                                    valstep=1, color=slider_color)
        
        # Koneksi event handler
        self.slider_theta1.on_changed(self.update)
        self.slider_theta2.on_changed(self.update)
        self.slider_theta3.on_changed(self.update)
        
        # Label informasi (inisialisasi)
        self.info_text = None
        
        # Plot awal
        self.update(None)
        
    def update(self, val):
        """Update plot berdasarkan nilai slider"""
        # Ambil nilai dari slider (dalam derajat)
        theta1_deg = self.slider_theta1.val
        theta2_deg = self.slider_theta2.val
        theta3_deg = self.slider_theta3.val
        
        # Konversi ke radian
        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg)
        theta3_rad = math.radians(theta3_deg)
        
        # Hitung forward kinematics
        positions = self.robot.forward_kinematics(theta1_rad, theta2_rad, theta3_rad)
        
        # Clear axis
        self.ax.clear()
        
        # Set ulang batas sumbu
        total_length = self.robot.L1 + self.robot.L2 + self.robot.L3
        margin = 2
        self.ax.set_xlim(-margin, total_length + margin)
        self.ax.set_ylim(-total_length - margin, total_length + margin)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (meter)', fontsize=12)
        self.ax.set_ylabel('Y (meter)', fontsize=12)
        self.ax.set_title('Simulasi Forward Kinematik Robot 3 DOF Planar', fontsize=14, fontweight='bold')
        
        # Ekstrak koordinat
        x_coords = [p[0] for p in positions]
        y_coords = [p[1] for p in positions]
        
        # Gambar link robot (perbaikan: hapus warna redundant)
        self.ax.plot(x_coords, y_coords, 'b-o', linewidth=3, markersize=8, label='Robot Arm')
        
        # Gambar joint points
        self.ax.plot(x_coords, y_coords, 'ro', markersize=10, zorder=5)
        
        # Label joint
        joint_labels = ['Base', 'Joint 1', 'Joint 2', 'End-Effector']
        for i, (x, y) in enumerate(positions):
            self.ax.annotate(joint_labels[i], (x, y), xytext=(5, 5), 
                           textcoords='offset points', fontsize=9)
        
        # Gambar lingkaran kerja end-effector
        end_effector = positions[-1]
        circle = plt.Circle((end_effector[0], end_effector[1]), 0.1, 
                           color='green', alpha=0.3, label='End-Effector Area')
        self.ax.add_patch(circle)
        
        # Hitung posisi end-effector dalam koordinat kartesian
        x_ee, y_ee = end_effector
        distance = np.sqrt(x_ee**2 + y_ee**2)
        
        # Tampilkan informasi
        info_str = f"""Forward Kinematics Results:
θ1 = {theta1_deg:.1f}° | θ2 = {theta2_deg:.1f}° | θ3 = {theta3_deg:.1f}°
─────────────────────────────────
End-Effector Position:
X = {x_ee:.3f} m
Y = {y_ee:.3f} m
Distance from base = {distance:.3f} m
─────────────────────────────────
Link Lengths: L1={self.robot.L1}, L2={self.robot.L2}, L3={self.robot.L3} m"""
        
        # Update info text (perbaikan: gunakan text, bukan remove)
        if self.info_text is not None:
            self.info_text.remove()
        self.info_text = self.ax.text(0.02, 0.98, info_str, transform=self.ax.transAxes,
                                      verticalalignment='top', fontsize=10,
                                      bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
        
        # Legend
        self.ax.legend(loc='upper right')
        
        # Redraw
        self.fig.canvas.draw_idle()
        
    def show(self):
        """Menampilkan plot"""
        plt.show()

def main():
    """Fungsi utama"""
    print("=" * 60)
    print("SIMULASI FORWARD KINEMATIK ROBOT 3 DOF PLANAR")
    print("=" * 60)
    
    # Parameter robot
    print("\nMasukkan panjang link (dalam meter):")
    L1 = float(input("Panjang Link 1 (L1): ") or 2.0)
    L2 = float(input("Panjang Link 2 (L2): ") or 1.5)
    L3 = float(input("Panjang Link 3 (L3): ") or 1.0)
    
    print("\nMasukkan sudut awal (dalam derajat):")
    theta1_deg = float(input("Sudut joint 1 (θ1): ") or 30)
    theta2_deg = float(input("Sudut joint 2 (θ2): ") or 45)
    theta3_deg = float(input("Sudut joint 3 (θ3): ") or 60)
    
    # Buat robot
    robot = Robot3DOF([L1, L2, L3])
    
    # Hitung posisi awal end-effector
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg)
    theta3_rad = math.radians(theta3_deg)
    pos_ee = robot.get_end_effector_position(theta1_rad, theta2_rad, theta3_rad)
    
    print("\n" + "=" * 60)
    print("HASIL FORWARD KINEMATIK AWAL")
    print("=" * 60)
    print(f"Posisi End-Effector: ({pos_ee[0]:.3f}, {pos_ee[1]:.3f}) meter")
    print(f"Jarak dari base: {np.sqrt(pos_ee[0]**2 + pos_ee[1]**2):.3f} meter")
    
    # Jalankan visualisasi
    print("\nMemulai simulasi interaktif...")
    print("Gunakan slider untuk mengubah sudut joint secara real-time")
    print("Tutup jendela untuk keluar")
    
    visualizer = RobotVisualizer(robot, [theta1_deg, theta2_deg, theta3_deg])
    visualizer.show()

if __name__ == "__main__":
    main()