# Forward Kinematik Robot 3 DOF Planar

## Deskripsi Program

Program simulasi forward kinematik untuk robot lengan planar 3 DOF. Menghitung posisi end-effector berdasarkan sudut joint input dan menampilkan visualisasi interaktif.

**Fitur:**
- Input panjang link dan sudut joint
- Perhitungan posisi end-effector otomatis
- Visualisasi real-time dengan slider interaktif
- Tampilan workspace robot

## Rumus Forward Kinematik

### Posisi End-Effector
x = L1·cos(θ1) + L2·cos(θ1+θ2) + L3·cos(θ1+θ2+θ3)
y = L1·sin(θ1) + L2·sin(θ1+θ2) + L3·sin(θ1+θ2+θ3)


### Posisi Setiap Joint
Joint 0 (Base): (0, 0)
Joint 1: (L1·cosθ1, L1·sinθ1)
Joint 2: (x1 + L2·cos(θ1+θ2), y1 + L2·sin(θ1+θ2))
End-Effector: (x2 + L3·cos(θ1+θ2+θ3), y2 + L3·sin(θ1+θ2+θ3))


### Jarak dari Base
Distance = √(x² + y²)


### Parameter Denavit-Hartenberg

| Link | a (m) | α (rad) | d (m) | θ (rad) |
|------|-------|---------|-------|---------|
| 1    | 0     | 0       | 0     | θ1      |
| 2    | L1    | 0       | 0     | θ2      |
| 3    | L2    | 0       | 0     | θ3      |

### Matriks Transformasi Homogen
T = [cosθ -sinθ 0 a]
[sinθ cosθ 0 0]
[0 0 1 d]
[0 0 0 1]


Transformasi total: `T_total = T1 × T2 × T3`

## Metode yang Digunakan

### 1. Metode Geometri Langsung
- Pendekatan trigonometri sederhana
- Perhitungan cepat
- Cocok untuk robot planar

### 2. Metode Denavit-Hartenberg
- Pendekatan matriks transformasi
- Sistematis untuk berbagai konfigurasi robot
- Menghasilkan posisi dan orientasi
Hasil End Of Factor dari Program :

<img width="447" height="356" alt="Screenshot 2026-04-07 164257" src="https://github.com/user-attachments/assets/5edddbac-c05c-446a-a0ee-08eb30213063" />
Tampilan dari Join Robot
