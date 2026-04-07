import numpy as np
import math
import matplotlib.pyplot as plt

class InverseKinematics3DOF:
    def __init__(self):
        """
        Inisialisasi robot 3 DOF dengan panjang link default
        """
        # PANJANG LINK DEFAULT (bisa diubah langsung di sini)
        self.L1 = 5.0   # meter
        self.L2 = 4.0   # meter  
        self.L3 = 3.0   # meter
        
        self.max_reach = self.L1 + self.L2 + self.L3
        self.min_reach = abs(self.L1 - self.L2 - self.L3)
    
    def forward_kinematics(self, theta1, theta2, theta3):
        """
        Forward kinematics untuk mendapatkan posisi semua joint
        """
        theta1_r = math.radians(theta1)
        theta2_r = math.radians(theta2)
        theta3_r = math.radians(theta3)
        
        # Posisi Joint 1 (base)
        x1, y1 = 0, 0
        
        # Posisi Joint 2 (elbow)
        x2 = self.L1 * math.cos(theta1_r)
        y2 = self.L1 * math.sin(theta1_r)
        
        # Posisi Joint 3 (wrist)
        x3 = x2 + self.L2 * math.cos(theta1_r + theta2_r)
        y3 = y2 + self.L2 * math.sin(theta1_r + theta2_r)
        
        # Posisi End-Effector
        x_end = x3 + self.L3 * math.cos(theta1_r + theta2_r + theta3_r)
        y_end = y3 + self.L3 * math.sin(theta1_r + theta2_r + theta3_r)
        
        return (x1, y1), (x2, y2), (x3, y3), (x_end, y_end)
    
    def inverse_kinematics_geometric(self, x_target, y_target, orientation=None, elbow_up=True):
        """
        Inverse kinematics dengan metode geometri
        """
        # Jarak dari base ke target
        D = np.sqrt(x_target**2 + y_target**2)
        
        # Cek reachability
        if D > self.max_reach + 0.01:
            return None
        if D < self.min_reach - 0.01 and D > 0:
            return None
        
        # Tentukan orientasi
        if orientation is None:
            orientation = math.degrees(math.atan2(y_target, x_target))
        
        theta3 = math.radians(orientation)
        
        # Posisi wrist
        wrist_x = x_target - self.L3 * math.cos(theta3)
        wrist_y = y_target - self.L3 * math.sin(theta3)
        D_wrist = np.sqrt(wrist_x**2 + wrist_y**2)
        
        # Cek wrist reachability
        if D_wrist > self.L1 + self.L2 + 0.01:
            return None
        if D_wrist < abs(self.L1 - self.L2) - 0.01:
            return None
        
        # Hukum cosinus untuk theta2
        cos_theta2 = (self.L1**2 + self.L2**2 - D_wrist**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)
        
        sin_theta2 = np.sqrt(1 - cos_theta2**2)
        if not elbow_up:
            sin_theta2 = -sin_theta2
        
        theta2_rad = np.arctan2(sin_theta2, cos_theta2)
        theta2 = math.degrees(theta2_rad)
        
        # Hitung theta1
        gamma = np.arctan2(wrist_y, wrist_x)
        alpha = np.arctan2(self.L2 * sin_theta2, self.L1 + self.L2 * cos_theta2)
        theta1_rad = gamma - alpha
        theta1 = math.degrees(theta1_rad)
        
        return theta1, theta2, orientation
    
    def plot_simulation(self, x_target, y_target):
        """
        Menampilkan simulasi inverse kinematics
        """
        # Hitung kedua konfigurasi
        sol_up = self.inverse_kinematics_geometric(x_target, y_target, elbow_up=True)
        sol_down = self.inverse_kinematics_geometric(x_target, y_target, elbow_up=False)
        
        if not sol_up and not sol_down:
            print(f"\n❌ Target ({x_target}, {y_target}) tidak reachable!")
            print(f"   Jarak: {np.sqrt(x_target**2 + y_target**2):.2f} m")
            print(f"   Jangkauan: {self.min_reach:.1f} m - {self.max_reach:.1f} m")
            return
        
        # Buat figure dengan 2 subplot (elbow up dan elbow down)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
        
        # Plot untuk Elbow Up
        if sol_up:
            theta1, theta2, theta3 = sol_up
            joints_up = self.forward_kinematics(theta1, theta2, theta3)
            self._plot_robot(ax1, joints_up, theta1, theta2, theta3, x_target, y_target, "ELBOW UP")
        else:
            ax1.text(0.5, 0.5, 'Tidak ada solusi\nElbow Up', 
                    ha='center', va='center', transform=ax1.transAxes, fontsize=14)
            ax1.set_title("ELBOW UP (Tidak Reachable)")
        
        # Plot untuk Elbow Down
        if sol_down:
            theta1, theta2, theta3 = sol_down
            joints_down = self.forward_kinematics(theta1, theta2, theta3)
            self._plot_robot(ax2, joints_down, theta1, theta2, theta3, x_target, y_target, "ELBOW DOWN")
        else:
            ax2.text(0.5, 0.5, 'Tidak ada solusi\nElbow Down', 
                    ha='center', va='center', transform=ax2.transAxes, fontsize=14)
            ax2.set_title("ELBOW DOWN (Tidak Reachable)")
        
        plt.suptitle(f'SIMULASI INVERSE KINEMATIK ROBOT 3 DOF\nTarget: ({x_target:.2f}, {y_target:.2f}) m', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.show()
        
        # Tampilkan hasil perhitungan
        print("\n" + "="*70)
        print("HASIL PERHITUNGAN INVERSE KINEMATIK")
        print("="*70)
        
        if sol_up:
            theta1, theta2, theta3 = sol_up
            joints = self.forward_kinematics(theta1, theta2, theta3)
            x_end, y_end = joints[3]
            error = np.sqrt((x_end - x_target)**2 + (y_end - y_target)**2)
            
            print("\n🔧 KONFIGURASI 1 (ELBOW UP):")
            print(f"   θ₁ (Base)   = {theta1:.3f}°")
            print(f"   θ₂ (Elbow)  = {theta2:.3f}°")
            print(f"   θ₃ (Wrist)  = {theta3:.3f}°")
            print(f"\n📍 POSISI JOINT:")
            print(f"   Base  : (0.000, 0.000) m")
            print(f"   Elbow : ({joints[1][0]:.3f}, {joints[1][1]:.3f}) m")
            print(f"   Wrist : ({joints[2][0]:.3f}, {joints[2][1]:.3f}) m")
            print(f"   End   : ({joints[3][0]:.3f}, {joints[3][1]:.3f}) m")
            print(f"\n✅ Error posisi: {error:.6f} m")
        
        if sol_down:
            theta1, theta2, theta3 = sol_down
            joints = self.forward_kinematics(theta1, theta2, theta3)
            x_end, y_end = joints[3]
            error = np.sqrt((x_end - x_target)**2 + (y_end - y_target)**2)
            
            print("\n🔧 KONFIGURASI 2 (ELBOW DOWN):")
            print(f"   θ₁ (Base)   = {theta1:.3f}°")
            print(f"   θ₂ (Elbow)  = {theta2:.3f}°")
            print(f"   θ₃ (Wrist)  = {theta3:.3f}°")
            print(f"\n📍 POSISI JOINT:")
            print(f"   Base  : (0.000, 0.000) m")
            print(f"   Elbow : ({joints[1][0]:.3f}, {joints[1][1]:.3f}) m")
            print(f"   Wrist : ({joints[2][0]:.3f}, {joints[2][1]:.3f}) m")
            print(f"   End   : ({joints[3][0]:.3f}, {joints[3][1]:.3f}) m")
            print(f"\n✅ Error posisi: {error:.6f} m")
    
    def _plot_robot(self, ax, joints, theta1, theta2, theta3, x_target, y_target, title):
        """
        Plot robot pada axis tertentu
        """
        (x1, y1), (x2, y2), (x3, y3), (x_end, y_end) = joints
        
        # Gambar link robot
        ax.plot([x1, x2], [y1, y2], 'b-', linewidth=3, label='Link 1 (Base-Elbow)')
        ax.plot([x2, x3], [y2, y3], 'r-', linewidth=3, label='Link 2 (Elbow-Wrist)')
        ax.plot([x3, x_end], [y3, y_end], 'g-', linewidth=3, label='Link 3 (Wrist-End)')
        
        # Gambar joint points
        ax.plot(x1, y1, 'bo', markersize=15, label='Base (Joint 1)', zorder=3)
        ax.plot(x2, y2, 'ro', markersize=12, label='Elbow (Joint 2)', zorder=3)
        ax.plot(x3, y3, 'yo', markersize=12, label='Wrist (Joint 3)', zorder=3)
        ax.plot(x_end, y_end, 'go', markersize=15, label='End-Effector', zorder=3)
        
        # Gambar target
        ax.plot(x_target, y_target, 'm*', markersize=20, label='Target', zorder=4)
        
        # Gambar lingkaran jangkauan
        circle_max = plt.Circle((0, 0), self.max_reach, fill=False, color='gray', 
                                linestyle='--', alpha=0.5, label=f'Max Reach ({self.max_reach:.0f}m)')
        circle_min = plt.Circle((0, 0), self.min_reach, fill=False, color='gray', 
                                linestyle=':', alpha=0.5, label=f'Min Reach ({self.min_reach:.0f}m)')
        ax.add_patch(circle_max)
        ax.add_patch(circle_min)
        
        # Anotasi sudut
        # θ1 di base
        theta1_rad = math.radians(theta1)
        arc_radius = self.L1 / 4
        theta1_angles = np.linspace(0, theta1_rad, 30)
        arc_x = arc_radius * np.cos(theta1_angles)
        arc_y = arc_radius * np.sin(theta1_angles)
        ax.plot(arc_x, arc_y, 'b--', linewidth=1.5, alpha=0.7)
        
        # Teks sudut
        ax.text(x2/2 + 0.3, y2/2 + 0.3, f'θ₁={theta1:.1f}°', 
                fontsize=10, fontweight='bold', 
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.8))
        ax.text((x2+x3)/2 + 0.3, (y2+y3)/2 + 0.3, f'θ₂={theta2:.1f}°',
                fontsize=10, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.8))
        ax.text((x3+x_end)/2 + 0.3, (y3+y_end)/2 + 0.3, f'θ₃={theta3:.1f}°',
                fontsize=10, fontweight='bold',
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.8))
        
        # Pengaturan plot
        ax.grid(True, alpha=0.3, linestyle='--')
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.8)
        ax.axvline(x=0, color='k', linestyle='-', linewidth=0.8)
        ax.set_xlabel('X (meter)', fontsize=11, fontweight='bold')
        ax.set_ylabel('Y (meter)', fontsize=11, fontweight='bold')
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.legend(loc='upper left', bbox_to_anchor=(1, 1), fontsize=9)
        ax.axis('equal')
        
        # Set batas plot
        max_range = max(abs(x_target), abs(y_target), self.max_reach)
        margin = max_range * 0.2 if max_range > 0 else 5
        ax.set_xlim(-margin, self.max_reach + margin)
        ax.set_ylim(-margin, self.max_reach + margin)


def main():
    print("="*70)
    print("     SIMULASI INVERSE KINEMATIK ROBOT 3 DOF PLANAR")
    print("="*70)
    
    # Buat objek robot
    robot = InverseKinematics3DOF()
    
    # Tampilkan informasi robot
    print(f"\n📌 SPESIFIKASI ROBOT:")
    print(f"   L1 (Base-Elbow) = {robot.L1} m")
    print(f"   L2 (Elbow-Wrist) = {robot.L2} m")
    print(f"   L3 (Wrist-End)   = {robot.L3} m")
    print(f"   📐 Jangkauan: {robot.min_reach:.1f} m s/d {robot.max_reach:.1f} m")
    
    while True:
        print("\n" + "="*70)
        print("INPUT KOORDINAT TARGET")
        print("="*70)
        
        try:
            x = float(input("  X target (meter): "))
            y = float(input("  Y target (meter): "))
            
            # Tampilkan simulasi
            robot.plot_simulation(x, y)
            
        except ValueError:
            print("\n❌ Input tidak valid! Masukkan angka yang benar.")
        
        print("\n" + "-"*70)
        lagi = input("Input koordinat lagi? (y/n): ").lower()
        if lagi != 'y':
            break
    
    print("\n" + "="*70)
    print("TERIMA KASIH TELAH MENGGUNAKAN SIMULATOR INI!")
    print("="*70)


if __name__ == "__main__":
    main()