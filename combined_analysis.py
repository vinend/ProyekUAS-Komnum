import numpy as np
import matplotlib.pyplot as plt
import random
import os

# ===================================================================================
# BAGIAN 0: PEMBUATAN DATA
# ===================================================================================
def generate_synthetic_data(num_sets=15, filename="synthetic_data.txt"):
    """Menghasilkan file data sintetis untuk diuji."""
    print(f"Menghasilkan {num_sets} set data ke dalam file '{filename}'...")
    with open(filename, "w") as f:
        for _ in range(num_sets):
            c1 = random.uniform(0.05, 0.5)
            c2 = random.uniform(100.0, 500.0)
            v0 = random.uniform(1.0, 15.0)
            tolerance = 1e-6
            max_iter = 100
            f.write(f"{c1} {c2} {v0} {tolerance} {max_iter}\n")
    print("Pembuatan data selesai.")

# ===================================================================================
# BAGIAN 1: FUNGSI-FUNGSI INTI
# ===================================================================================

def f_drone(v, c1, c2):
    """f(v) = 2*c1*v - 2*c2*v^-3"""
    if v <= 1e-9: return 1e12
    return 2.0 * c1 * v - (2.0 * c2 / (v**3))

def df_drone(v, c1, c2):
    """f'(v) = 2*c1 + 6*c2*v^-4"""
    if v <= 1e-9: return 1e12
    return 2.0 * c1 + (6.0 * c2 / (v**4))

def newton_raphson_solver(c1, c2, v0, tolerance=1e-6, max_iter=100):
    """Solver Newton-Raphson untuk menemukan v_opt."""
    v_current = v0
    for _ in range(max_iter):
        f_val = f_drone(v_current, c1, c2)
        df_val = df_drone(v_current, c1, c2)
        if abs(df_val) < 1e-10: return v_current, False
        v_next = v_current - f_val / df_val
        if v_next <= 0: return v_current, False
        if abs(v_next - v_current) < tolerance:
            return v_next, True
        v_current = v_next
    return v_current, False

def power_consumption(v, c1, c2):
    """Fungsi daya drone P(v) yang telah divektorisasi."""
    v_arr = np.asarray(v)
    return np.where(v_arr > 1e-9, c1 * (v_arr**3) + c2 / v_arr, 0)

def high_accuracy_diff(f, x, h=0.01):
    """Diferensiasi Akurasi Tinggi (Centered, O(h^4))."""
    f_xp1, f_xm1 = f(x + h), f(x - h)
    f_xp2, f_xm2 = f(x + 2*h), f(x - 2*h)
    return (-f_xp2 + 8*f_xp1 - 8*f_xm1 + f_xm2) / (12 * h)

def trapezoidal_solver(f, a, b, n):
    """Aturan Trapesium (Helper untuk Romberg)."""
    h = (b - a) / n
    x = np.linspace(a, b, n + 1)
    y = f(x)
    return h * (0.5*y[0] + 0.5*y[-1] + np.sum(y[1:-1]))

def romberg_solver(f, a, b, max_iter=6):
    """Integrasi Romberg."""
    I = np.zeros((max_iter, max_iter))
    for i in range(max_iter):
        I[i, 0] = trapezoidal_solver(f, a, b, 1 << i)
    for k in range(1, max_iter):
        for j in range(max_iter - k):
            I[j, k] = (4**k * I[j + 1, k - 1] - I[j, k - 1]) / (4**k - 1.0)
    return I[0, max_iter - 1]

# ===================================================================================
# BAGIAN 2: FUNGSI-FUNGSI VISUALISASI
# ===================================================================================

def create_visualizations(results):
    """Membuat dan menyimpan semua plot dari hasil analisis."""
    print(f"\n{'='*70}\nMEMBUAT VISUALISASI HASIL\n{'='*70}\n")
    
    # --- Plot Ringkasan ---
    v_opts_num = [r['v_opt_num'] for r in results if r['converged']]
    v_opts_ana = [r['v_opt_ana'] for r in results if r['converged']]
    energies = [r['energy'] for r in results]
    
    # FIG 1: Validasi Solver Numerik
    plt.figure(figsize=(8, 8))
    plt.scatter(v_opts_ana, v_opts_num, alpha=0.7, edgecolors='k', label='Kasus Uji')
    plt.plot([min(v_opts_ana), max(v_opts_ana)], [min(v_opts_ana), max(v_opts_ana)], 'r--', label='Kesesuaian Sempurna (y=x)')
    plt.title('Validasi Kecepatan Optimal: Numerik vs. Analitik')
    plt.xlabel('v_opt Analitik (m/s)')
    plt.ylabel('v_opt Numerik (m/s)')
    plt.grid(True)
    plt.legend()
    plt.savefig('1_validasi_v_optimal.png')
    print("Plot '1_validasi_v_optimal.png' disimpan.")

    # FIG 2: Distribusi Energi Manuver
    plt.figure(figsize=(10, 6))
    plt.hist(energies, bins=10, edgecolor='k', alpha=0.7)
    plt.title('Distribusi Total Energi Manuver di Berbagai Skenario Drone')
    plt.xlabel('Total Energi (Joule)')
    plt.ylabel('Frekuensi (Jumlah Kasus)')
    plt.grid(axis='y')
    plt.savefig('2_distribusi_energi.png')
    print("Plot '2_distribusi_energi.png' disimpan.")

    # --- Plot Perbandingan Gabungan ---
    T_maneuver = 10.0
    t = np.linspace(0, T_maneuver, 200)
    v_maneuver_awal = 1.0
    cmap = plt.get_cmap('viridis', len(results))

    # FIG 3: Plot Gabungan Profil Kecepatan
    plt.figure(figsize=(12, 7))
    for i, r in enumerate(results):
        v_t = r['velocity_profile']
        plt.plot(t, v_t, color=cmap(i), label=f'Kasus {i+1}')
    plt.title('Perbandingan Profil Kecepatan Manuver untuk Semua Kasus')
    plt.xlabel('Waktu (s)')
    plt.ylabel('Kecepatan (m/s)')
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    plt.tight_layout(rect=[0, 0, 0.85, 1])
    plt.savefig('3_perbandingan_profil_kecepatan.png')
    print("Plot '3_perbandingan_profil_kecepatan.png' disimpan.")

    # FIG 4: Plot Gabungan Profil Daya
    plt.figure(figsize=(12, 7))
    for i, r in enumerate(results):
        p_t = r['power_profile']
        plt.plot(t, p_t, color=cmap(i), label=f'Kasus {i+1}')
    plt.title('Perbandingan Profil Daya Manuver untuk Semua Kasus')
    plt.xlabel('Waktu (s)')
    plt.ylabel('Daya (Watt)')
    plt.grid(True)
    plt.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    plt.tight_layout(rect=[0, 0, 0.85, 1])
    plt.savefig('4_perbandingan_profil_daya.png')
    print("Plot '4_perbandingan_profil_daya.png' disimpan.")
    
    # --- MODIFIKASI: FIG 5: Grid Profil Manuver Individu ---
    num_results = len(results)
    cols = 3
    rows = (num_results + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows), squeeze=False)
    fig.suptitle('Grid Profil Manuver Individu untuk Setiap Kasus Uji', fontsize=16, y=1.02)
    
    flat_axes = axes.flatten()
    for i, r in enumerate(results):
        ax1 = flat_axes[i]
        v_t = r['velocity_profile']
        p_t = r['power_profile']
        
        ax1.plot(t, v_t, 'b-', label='Kecepatan')
        ax1.set_xlabel('Waktu (s)')
        ax1.set_ylabel('Kecepatan (m/s)', color='b')
        ax1.tick_params(axis='y', labelcolor='b')
        ax1.grid(True)
        
        ax2 = ax1.twinx()
        ax2.plot(t, p_t, 'g--', label='Daya')
        ax2.set_ylabel('Daya (Watt)', color='g')
        ax2.tick_params(axis='y', labelcolor='g')
        
        ax1.set_title(f"Kasus {i+1} (v_opt={r['v_opt_num']:.2f} m/s)")
    
    # Sembunyikan sumbu yang tidak terpakai
    for i in range(num_results, len(flat_axes)):
        flat_axes[i].axis('off')
        
    plt.tight_layout(rect=[0, 0, 1, 0.98])
    plt.savefig('5_grid_profil_manuver.png')
    print("Plot '5_grid_profil_manuver.png' disimpan.")
    
    plt.close('all') # Menutup semua figure

# ===================================================================================
# BAGIAN 3: FUNGSI UTAMA (MAIN)
# ===================================================================================
def main():
    generate_synthetic_data()
    results = []
    
    T_maneuver = 10.0
    v_maneuver_awal = 1.0
    t_profile = np.linspace(0, T_maneuver, 200)

    with open("synthetic_data.txt", "r") as f:
        for i, line in enumerate(f):
            parts = [float(p) for p in line.split()]
            c1, c2, v0, tol, max_iter = parts
            
            print(f"\n{'='*70}\nMEMPROSES KASUS UJI {i+1}: c1={c1:.4f}, c2={c2:.2f}, v0={v0:.2f}\n{'='*70}")
            
            # Tahap 1: Pencarian Akar
            v_opt, converged = newton_raphson_solver(c1, c2, v0, tol, int(max_iter))
            v_analytical = (c2 / c1)**0.25
            if not converged: v_opt = v_analytical
            
            # Tahap 2: Analisis Kinerja
            power_func = lambda v: power_consumption(v, c1, c2)
            dp_dv_numerical = high_accuracy_diff(power_func, v_opt)
            
            def velocity_maneuver_func(t):
                sin_term = np.sin((np.pi * t) / (2.0 * T_maneuver))
                return v_maneuver_awal + (v_opt - v_maneuver_awal) * sin_term**2

            def power_maneuver_time_func(t):
                return power_consumption(velocity_maneuver_func(t), c1, c2)
            
            energy_romberg = romberg_solver(power_maneuver_time_func, 0, T_maneuver)
            
            # Hitung dan simpan profil untuk visualisasi nanti
            velocity_profile_data = velocity_maneuver_func(t_profile)
            power_profile_data = power_consumption(velocity_profile_data, c1, c2)

            results.append({
                "c1": c1, "c2": c2, "v0": v0,
                "v_opt_num": v_opt, "v_opt_ana": v_analytical,
                "dp_dv_num": dp_dv_numerical, "energy": energy_romberg,
                "converged": converged, "case_num": i + 1,
                "velocity_profile": velocity_profile_data,
                "power_profile": power_profile_data
            })
            
    create_visualizations(results)

if __name__ == "__main__":
    main()