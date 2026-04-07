import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Circle
import math

class InverseKinematics3DOF:
    def __init__(self, L1, L2, L3):
        """
        Inisialisasi robot 3 DOF dengan metode geometri
        
        Args:
            L1, L2, L3: panjang link dalam meter
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.max_reach = L1 + L2 + L3
        self.min_reach = abs(L1 - L2 - L3)
        
    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Forward kinematics untuk validasi
        """
        theta1_r = math.radians(theta1)
        theta2_r = math.radians(theta2)
        theta3_r = math.radians(theta3)
        
        # Joint positions
        x0, y0 = 0, 0
        x1 = self.L1 * np.cos(theta1_r)
        y1 = self.L1 * np.sin(theta1_r)
        x2 = x1 + self.L2 * np.cos(theta1_r + theta2_r)
        y2 = y1 + self.L2 * np.sin(theta1_r + theta2_r)
        x3 = x2 + self.L3 * np.cos(theta1_r + theta2_r + theta3_r)
        y3 = y2 + self.L3 * np.sin(theta1_r + theta2_r + theta3_r)
        
        return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
    
    def inverse_kinematics_geometric(self, x_target, y_target, orientation=None, elbow_up=True):
        """
        Inverse kinematics dengan metode geometri untuk 3 DOF planar
        
        Args:
            x_target, y_target: posisi target end-effector
            orientation: sudut orientasi end-effector yang diinginkan (derajat)
            elbow_up: True untuk elbow up, False untuk elbow down
        
        Returns:
            (theta1, theta2, theta3) dalam derajat atau None jika tidak reachable
        """
        # Jarak dari base ke target
        D = np.sqrt(x_target**2 + y_target**2)
        
        # Cek apakah target reachable
        if D > self.max_reach + 0.1:
            print(f"⚠️ Target terlalu jauh! Jarak: {D:.2f}m > Max reach: {self.max_reach:.2f}m")
            return None
        if D < self.min_reach - 0.1 and D > 0:
            print(f"⚠️ Target terlalu dekat! Jarak: {D:.2f}m < Min reach: {self.min_reach:.2f}m")
            return None
        
        # Jika orientation tidak ditentukan, hitung berdasarkan posisi
        if orientation is None:
            # Orientasi default: mengarah ke target
            orientation = math.degrees(math.atan2(y_target, x_target))
        
        # Konversi orientation ke radian
        theta3 = math.radians(orientation)
        
        # Posisi wrist (joint 2) = target - L3 * orientation vector
        wrist_x = x_target - self.L3 * np.cos(theta3)
        wrist_y = y_target - self.L3 * np.sin(theta3)
        
        # Jarak dari base ke wrist
        D_wrist = np.sqrt(wrist_x**2 + wrist_y**2)
        
        # Cek apakah wrist reachable oleh L1 dan L2
        if D_wrist > self.L1 + self.L2 + 0.1:
            print(f"⚠️ Wrist position tidak reachable! Jarak: {D_wrist:.2f}m")
            return None
        if D_wrist < abs(self.L1 - self.L2) - 0.1:
            print(f"⚠️ Wrist position terlalu dekat! Jarak: {D_wrist:.2f}m")
            return None
        
        # Hukum cosinus untuk mencari theta2
        cos_theta2 = (self.L1**2 + self.L2**2 - D_wrist**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)  # Numerical stability
        
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        if not elbow_up:
            sin_theta2 = -sin_theta2
        
        theta2_rad = np.arctan2(sin_theta2, cos_theta2)
        theta2 = math.degrees(theta2_rad)
        
        # Hitung theta1 menggunakan geometri
        gamma = np.arctan2(wrist_y, wrist_x)
        
        # Sudut antara L1 dan garis ke wrist
        alpha = np.arctan2(self.L2 * sin_theta2, self.L1 + self.L2 * cos_theta2)
        
        theta1_rad = gamma - alpha
        theta1 = math.degrees(theta1_rad)
        
        return theta1, theta2, orientation
    
    def get_all_solutions(self, x_target, y_target, orientation=None):
        """
        Mendapatkan semua kemungkinan solusi inverse kinematics
        """
        solutions = []
        
        # Elbow up
        sol_up = self.inverse_kinematics_geometric(x_target, y_target, orientation, elbow_up=True)
        if sol_up:
            solutions.append(('Elbow Up', sol_up))
        
        # Elbow down
        sol_down = self.inverse_kinematics_geometric(x_target, y_target, orientation, elbow_up=False)
        if sol_down:
            solutions.append(('Elbow Down', sol_down))
        
        return solutions

class InverseKinematicsVisualizer:
    def __init__(self, robot):
        """
        Inisialisasi visualizer inverse kinematics
        """
        self.robot = robot
        self.target_x = 15.0  # Target yang reachable untuk L1=3, L2=7, L3=9
        self.target_y = 5.0
        self.orientation = None
        self.current_solution = None
        self.elbow_up = True
        
        # Setup plot
        self.setup_plot()
        
    def setup_plot(self):
        """Membuat figure dan kontrol"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 7))
        plt.subplots_adjust(left=0.08, bottom=0.25, right=0.95, top=0.95)
        
        # Setup axis untuk robot visualization
        self.ax1.set_title('Inverse Kinematics - Robot 3 DOF Planar', fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('X (meter)', fontsize=12)
        self.ax1.set_ylabel('Y (meter)', fontsize=12)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Setup axis untuk error plot
        self.ax2.set_title('Error Tracking', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('Iteration', fontsize=12)
        self.ax2.set_ylabel('Position Error (m)', fontsize=12)
        self.ax2.grid(True, alpha=0.3)
        self.error_history = []
        
        # Slider untuk target X
        ax_target_x = plt.axes([0.1, 0.15, 0.35, 0.03])
        max_reach = self.robot.max_reach
        self.slider_target_x = Slider(ax_target_x, 'Target X (m)', -max_reach, max_reach, 
                                      valinit=self.target_x, valstep=0.5)
        
        # Slider untuk target Y
        ax_target_y = plt.axes([0.55, 0.15, 0.35, 0.03])
        self.slider_target_y = Slider(ax_target_y, 'Target Y (m)', -max_reach, max_reach, 
                                      valinit=self.target_y, valstep=0.5)
        
        # Slider untuk orientation
        ax_orientation = plt.axes([0.1, 0.10, 0.35, 0.03])
        self.slider_orientation = Slider(ax_orientation, 'Orientation (deg)', -180, 180, 
                                         valinit=0, valstep=5)
        
        # Button untuk elbow up/down menggunakan button biasa
        ax_elbow_up = plt.axes([0.55, 0.10, 0.15, 0.04])
        self.btn_elbow_up = Button(ax_elbow_up, 'Elbow Up', color='lightgreen', hovercolor='green')
        
        ax_elbow_down = plt.axes([0.72, 0.10, 0.15, 0.04])
        self.btn_elbow_down = Button(ax_elbow_down, 'Elbow Down', color='lightcoral', hovercolor='red')
        
        # Button untuk solve
        ax_solve = plt.axes([0.55, 0.04, 0.15, 0.04])
        self.btn_solve = Button(ax_solve, 'Solve IK', color='lightblue', hovercolor='blue')
        
        # Button untuk reset
        ax_reset = plt.axes([0.72, 0.04, 0.15, 0.04])
        self.btn_reset = Button(ax_reset, 'Reset View', color='lightgray', hovercolor='gray')
        
        # Status text untuk elbow
        self.elbow_status = self.ax1.text(0.02, 0.92, 'Elbow Mode: Up', transform=self.ax1.transAxes,
                                          fontsize=10, fontweight='bold', color='green')
        
        # Koneksi event handlers
        self.slider_target_x.on_changed(self.target_changed)
        self.slider_target_y.on_changed(self.target_changed)
        self.slider_orientation.on_changed(self.orientation_changed)
        self.btn_elbow_up.on_clicked(self.set_elbow_up)
        self.btn_elbow_down.on_clicked(self.set_elbow_down)
        self.btn_solve.on_clicked(self.solve_ik)
        self.btn_reset.on_clicked(self.reset_view)
        
        # Solve initial
        self.solve_ik(None)
        
    def target_changed(self, val):
        """Handler saat target berubah"""
        self.target_x = self.slider_target_x.val
        self.target_y = self.slider_target_y.val
        self.solve_ik(None)
    
    def orientation_changed(self, val):
        """Handler saat orientation berubah"""
        self.orientation = self.slider_orientation.val if self.slider_orientation.val != 0 else None
        self.solve_ik(None)
    
    def set_elbow_up(self, event):
        """Set elbow up mode"""
        self.elbow_up = True
        self.elbow_status.set_text('Elbow Mode: Up')
        self.elbow_status.set_color('green')
        self.solve_ik(None)
    
    def set_elbow_down(self, event):
        """Set elbow down mode"""
        self.elbow_up = False
        self.elbow_status.set_text('Elbow Mode: Down')
        self.elbow_status.set_color('red')
        self.solve_ik(None)
    
    def reset_view(self, event):
        """Reset view ke default"""
        self.ax1.set_xlim(-self.robot.max_reach - 2, self.robot.max_reach + 2)
        self.ax1.set_ylim(-self.robot.max_reach - 2, self.robot.max_reach + 2)
        self.fig.canvas.draw_idle()
    
    def solve_ik(self, event):
        """Solve inverse kinematics untuk target saat ini"""
        # Gunakan orientation jika slider tidak 0
        orientation = self.orientation if self.orientation != 0 else None
        
        # Solve IK
        self.current_solution = self.robot.inverse_kinematics_geometric(
            self.target_x, self.target_y, orientation, elbow_up=self.elbow_up
        )
        
        # Update visualisasi
        self.update_visualization()
    
    def update_visualization(self):
        """Update semua visualisasi"""
        # Clear axes
        self.ax1.clear()
        self.ax2.clear()
        
        # Setup axes
        self.ax1.set_title('Inverse Kinematics - Robot 3 DOF Planar', fontsize=14, fontweight='bold')
        self.ax1.set_xlabel('X (meter)', fontsize=12)
        self.ax1.set_ylabel('Y (meter)', fontsize=12)
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_aspect('equal')
        
        # Set limits
        margin = 2
        self.ax1.set_xlim(-self.robot.max_reach - margin, self.robot.max_reach + margin)
        self.ax1.set_ylim(-self.robot.max_reach - margin, self.robot.max_reach + margin)
        
        # Plot target
        self.ax1.plot(self.target_x, self.target_y, 'g*', markersize=15, label='Target Position', zorder=10)
        
        # Draw workspace (outer circle)
        outer_workspace = Circle((0, 0), self.robot.max_reach, fill=False, color='gray', linestyle='--', alpha=0.5)
        self.ax1.add_patch(outer_workspace)
        
        # Draw inner workspace (minimum reach)
        if self.robot.min_reach > 0:
            inner_workspace = Circle((0, 0), self.robot.min_reach, fill=False, color='red', linestyle='--', alpha=0.3)
            self.ax1.add_patch(inner_workspace)
        
        if self.current_solution:
            theta1, theta2, theta3 = self.current_solution
            
            # Hitung forward kinematics untuk solusi
            positions = self.robot.forward_kinematics(theta1, theta2, theta3)
            
            # Extract coordinates
            x_coords = [p[0] for p in positions]
            y_coords = [p[1] for p in positions]
            
            # Gambar robot
            self.ax1.plot(x_coords, y_coords, 'b-o', linewidth=3, markersize=8, label='Robot Configuration', zorder=5)
            self.ax1.plot(x_coords, y_coords, 'ro', markersize=10, zorder=6)
            
            # Label joint
            joint_labels = ['Base', 'Joint 1', 'Joint 2', 'End-Effector']
            for i, (x, y) in enumerate(positions):
                self.ax1.annotate(joint_labels[i], (x, y), xytext=(5, 5), 
                                textcoords='offset points', fontsize=9)
            
            # Gambar orientation line
            end_effector = positions[-1]
            orient_angle_rad = math.radians(theta3)
            orient_line_x = [end_effector[0], end_effector[0] + 0.8 * np.cos(orient_angle_rad)]
            orient_line_y = [end_effector[1], end_effector[1] + 0.8 * np.sin(orient_angle_rad)]
            self.ax1.plot(orient_line_x, orient_line_y, 'g-', linewidth=2, label='Orientation')
            
            # Hitung error
            error = np.sqrt((end_effector[0] - self.target_x)**2 + (end_effector[1] - self.target_y)**2)
            
            # Update error history
            self.error_history.append(error)
            if len(self.error_history) > 50:
                self.error_history.pop(0)
            
            # Tampilkan informasi solusi
            info_text = f"""INVERSE KINEMATICS SOLUTION
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
θ₁ = {theta1:.2f}°
θ₂ = {theta2:.2f}°
θ₃ = {theta3:.2f}°
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
End-Effector Position:
X = {end_effector[0]:.3f} m
Y = {end_effector[1]:.3f} m
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Target Position:
X = {self.target_x:.3f} m
Y = {self.target_y:.3f} m
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Position Error: {error:.4f} m
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Reachability: {'✓ Reachable' if error < 0.01 else '⚠ Approximated'}"""
            
            self.ax1.text(0.02, 0.98, info_text, transform=self.ax1.transAxes,
                         verticalalignment='top', fontsize=9,
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9))
            
            # Plot error history
            if len(self.error_history) > 1:
                self.ax2.plot(self.error_history, 'b-', linewidth=2)
                self.ax2.set_xlabel('Iteration', fontsize=12)
                self.ax2.set_ylabel('Position Error (m)', fontsize=12)
                self.ax2.set_title('Error Convergence', fontsize=14, fontweight='bold')
                self.ax2.grid(True, alpha=0.3)
                
                # Add horizontal line for tolerance
                self.ax2.axhline(y=0.001, color='r', linestyle='--', label='Tolerance (1mm)')
                self.ax2.legend()
            
        else:
            # Target tidak reachable
            self.ax1.text(0.5, 0.5, '⚠ TARGET NOT REACHABLE!\nAdjust target position',
                         transform=self.ax1.transAxes, ha='center', va='center',
                         fontsize=14, color='red', fontweight='bold',
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
        # Legend
        self.ax1.legend(loc='upper right')
        
        # Redraw elbow status
        if hasattr(self, 'elbow_status'):
            self.elbow_status = self.ax1.text(0.02, 0.92, f'Elbow Mode: {"Up" if self.elbow_up else "Down"}', 
                                             transform=self.ax1.transAxes,
                                             fontsize=10, fontweight='bold', 
                                             color='green' if self.elbow_up else 'red')
        
        # Redraw
        self.fig.canvas.draw_idle()
    
    def show(self):
        """Menampilkan plot"""
        plt.show()

def main():
    """Fungsi utama"""
    print("=" * 70)
    print("SIMULASI INVERSE KINEMATIK ROBOT 3 DOF PLANAR")
    print("METODE GEOMETRI")
    print("=" * 70)
    
    # Parameter robot
    print("\nMasukkan panjang link (dalam meter):")
    L1 = float(input("Panjang Link 1 (L1): ") or 3.0)
    L2 = float(input("Panjang Link 2 (L2): ") or 7.0)
    L3 = float(input("Panjang Link 3 (L3): ") or 9.0)
    
    print("\n" + "=" * 70)
    print("INFORMASI ROBOT")
    print("=" * 70)
    print(f"Panjang link: L1={L1}m, L2={L2}m, L3={L3}m")
    print(f"Maximum reach: {L1+L2+L3:.2f}m")
    print(f"Minimum reach: {abs(L1-L2-L3):.2f}m")
    print(f"Workspace: Antara radius {abs(L1-L2-L3):.2f}m dan {L1+L2+L3:.2f}m")
    
    print("\n" + "=" * 70)
    print("PANDUAN PENGGUNAAN")
    print("=" * 70)
    print("1. Gunakan slider untuk mengatur posisi target (X, Y)")
    print("2. Atur orientasi end-effector (opsional)")
    print("3. Pilih konfigurasi Elbow Up atau Elbow Down")
    print("4. Klik 'Solve IK' untuk menghitung inverse kinematics")
    print("5. Program akan menampilkan solusi joint angles dan posisi end-effector")
    print("\nCatatan:")
    print(f"- Target harus berada di antara radius {abs(L1-L2-L3):.2f}m dan {L1+L2+L3:.2f}m")
    print("- Target ditandai dengan bintang hijau (*)")
    print("- Lingkaran putus-putus menunjukkan batas workspace")
    print("- Garis hijau menunjukkan orientasi end-effector")
    
    input("\nTekan Enter untuk memulai simulasi...")
    
    # Buat robot
    robot = InverseKinematics3DOF(L1, L2, L3)
    
    # Contoh demonstrasi dengan target yang reachable
    print("\n" + "=" * 70)
    print("DEMONSTRASI INVERSE KINEMATIK")
    print("=" * 70)
    
    # Target yang reachable untuk L1=3, L2=7, L3=9 (min=13, max=19)
    test_targets = [
        (15.0, 5.0, "Target 1 - Dalam workspace"),
        (16.0, 0.0, "Target 2 - Sisi kanan"),
        (0.0, 16.0, "Target 3 - Sisi atas"),
        (-14.0, 8.0, "Target 4 - Sisi kiri"),
    ]
    
    print("\nMencoba beberapa target contoh:")
    for x, y, desc in test_targets:
        print(f"\n{desc}: Target ({x}, {y})")
        D = np.sqrt(x**2 + y**2)
        print(f"  Jarak dari base: {D:.2f}m")
        
        if robot.min_reach <= D <= robot.max_reach:
            solutions = robot.get_all_solutions(x, y)
            if solutions:
                for name, sol in solutions:
                    print(f"  {name}: θ1={sol[0]:.2f}°, θ2={sol[1]:.2f}°, θ3={sol[2]:.2f}°")
            else:
                print(f"  ❌ Tidak ada solusi!")
        else:
            print(f"  ❌ Target di luar workspace! (Harus antara {robot.min_reach:.2f}m dan {robot.max_reach:.2f}m)")
    
    # Jalankan visualisasi interaktif
    print("\n" + "=" * 70)
    print("MEMULAI SIMULASI INTERAKTIF")
    print("=" * 70)
    print("Tutup jendela untuk keluar")
    
    visualizer = InverseKinematicsVisualizer(robot)
    visualizer.show()

if __name__ == "__main__":
    main()