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

Input dan Output yang dihasilkan :


<img width="447" height="356" alt="Screenshot 2026-04-07 164257" src="https://github.com/user-attachments/assets/5edddbac-c05c-446a-a0ee-08eb30213063" />

Tampilan Join Robot:


<img width="932" height="789" alt="Screenshot 2026-04-07 164206" src="https://github.com/user-attachments/assets/511d7801-ed7b-4790-a0cf-860bb840c456" />


# Inverse Kinematik Robot 3 DOF Planar - Metode Geometri

## Deskripsi Program

Program simulasi inverse kinematik untuk robot lengan planar 3 DOF menggunakan metode geometri. Menghitung sudut joint (θ1, θ2, θ3) agar end-effector mencapai posisi target.

**Fitur:**
- Input panjang link dan posisi target
- Dua konfigurasi solusi (Elbow Up/Down)
- Visualisasi interaktif dengan slider
- Tracking error dan workspace

# 🤖 Inverse Kinematics Robot 3 DOF Planar

## 📋 Deskripsi Program

Program ini merupakan implementasi **Inverse Kinematics** untuk robot manipulator dengan **3 Degree of Freedom (DOF)** planar. Program dapat menghitung sudut-sudut joint (θ₁, θ₂, θ₃) yang diperlukan agar end-effector robot mencapai posisi target (x, y) yang diinginkan.

Program menampilkan **dua konfigurasi solusi**:
- **Elbow Up** (siku ke atas)
- **Elbow Down** (siku ke bawah)

Setiap konfigurasi divisualisasikan dalam bentuk grafik 2D yang menunjukkan posisi base, elbow, wrist, dan end-effector.


## 📐 Spesifikasi Robot

| Parameter | Nilai | Keterangan |
|-----------|-------|-------------|
| L₁ | 5.0 m | Panjang link 1 (Base ke Elbow) |
| L₂ | 4.0 m | Panjang link 2 (Elbow ke Wrist) |
| L₃ | 3.0 m | Panjang link 3 (Wrist ke End-Effector) |
| Jangkauan Min | 2.0 m | Jarak minimal yang dapat dijangkau |
| Jangkauan Max | 12.0 m | Jarak maksimal yang dapat dijangkau |

> **Catatan:** Panjang link dapat diubah langsung pada kode di bagian `__init__` method.

## 🧮 Metode dan Rumus yang Digunakan

### 1. Forward Kinematics

Forward kinematics digunakan untuk menghitung posisi end-effector berdasarkan sudut joint yang diketahui.

**Rumus:**
x = L₁·cos(θ₁) + L₂·cos(θ₁+θ₂) + L₃·cos(θ₁+θ₂+θ₃)
y = L₁·sin(θ₁) + L₂·sin(θ₁+θ₂) + L₃·sin(θ₁+θ₂+θ₃)

text

**Posisi Joint:**
- **Joint 1 (Base):** (0, 0)
- **Joint 2 (Elbow):** (L₁·cos θ₁, L₁·sin θ₁)
- **Joint 3 (Wrist):** (x₂ + L₂·cos(θ₁+θ₂), y₂ + L₂·sin(θ₁+θ₂))
- **End-Effector:** (x₃ + L₃·cos(θ₁+θ₂+θ₃), y₃ + L₃·sin(θ₁+θ₂+θ₃))

### 2. Inverse Kinematics (Metode Geometri)

Metode geometri digunakan untuk menghitung sudut joint berdasarkan posisi target (x, y).

#### Langkah 1: Menentukan Orientasi End-Effector (θ₃)
θ₃ = atan2(y_target, x_target)

text

Orientasi default mengarah ke target.

#### Langkah 2: Menghitung Posisi Wrist
wrist_x = x_target - L₃·cos(θ₃)
wrist_y = y_target - L₃·sin(θ₃)
D_wrist = √(wrist_x² + wrist_y²)

text

#### Langkah 3: Menghitung θ₂ (Hukum Cosinus)

Menggunakan hukum cosinus pada segitiga yang dibentuk oleh L₁, L₂, dan D_wrist:
cos θ₂ = (L₁² + L₂² - D_wrist²) / (2·L₁·L₂)
sin θ₂ = ±√(1 - cos²θ₂)
θ₂ = atan2(sin θ₂, cos θ₂)

text

**Tanda ± menentukan konfigurasi:**
- `+` untuk Elbow Up
- `-` untuk Elbow Down

#### Langkah 4: Menghitung θ₁
γ = atan2(wrist_y, wrist_x)
α = atan2(L₂·sin θ₂, L₁ + L₂·cos θ₂)
θ₁ = γ - α

text

### 3. Validasi Reachability

Program memeriksa apakah target berada dalam jangkauan robot:
D = √(x_target² + y_target²)

if D > (L₁ + L₂ + L₃): # Melebihi jangkauan maksimal
Target tidak reachable

if D < |L₁ - L₂ - L₃|: # Kurang dari jangkauan minimal
Target tidak reachable

Input koordinat x dan y :


<img width="523" height="274" alt="Screenshot 2026-04-07 182401" src="https://github.com/user-attachments/assets/5c7503b7-03a8-4702-94fd-8b235d7fe2c2" />


Hasil dari program simulasi inverse kinematik:


<img width="898" height="601" alt="Screenshot 2026-04-07 182418" src="https://github.com/user-attachments/assets/b247c4e8-8543-451f-9b12-6434c050344b" />
