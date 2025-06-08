// combined_analysis.cpp
//
// DESKRIPSI:
// Program ini menggabungkan dua analisis untuk setiap drone yang didefinisikan dalam 'synthetic_data.txt':
// 1. PENCARIAN AKAR (dari Drone.cpp): Menemukan kecepatan optimal (v_opt) menggunakan metode Newton-Raphson.
// 2. ANALISIS INTEGRASI & DIFERENSIASI (dari modified_case.cpp):
//    - Diferensiasi: Menghitung laju perubahan daya (dP/dv) pada v_opt.
//    - Integrasi: Menghitung total energi yang dibutuhkan untuk manuver percepatan dari 1 m/s ke v_opt.
//
// KOMPILASI:
// g++ combined_analysis.cpp -o combined_analysis -std=c++11
//
// CARA MENJALANKAN:
// 1. Jalankan program Data.cpp (jika belum) untuk membuat 'synthetic_data.txt'.
// 2. Jalankan program ini: ./combined_analysis

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <functional>
#include <iomanip>
#include <fstream>
#include <sstream>

// ===================================================================================
// BAGIAN 1: FUNGSI-FUNGSI INTI (Gabungan dari Drone.cpp dan modified_case.cpp)
// ===================================================================================

// --- Fungsi untuk Pencarian Akar (Menemukan v_opt) ---

// f(v) = 2*c1*v - 2*c2*v^-3
double f_drone(double v, double c1, double c2) {
    if (v <= 1e-9) return 1e12;
    return 2.0 * c1 * v - (2.0 * c2 / (v * v * v));
}

// f'(v) = 2*c1 + 6*c2*v^-4
double df_drone(double v, double c1, double c2) {
    if (v <= 1e-9) return 1e12;
    return 2.0 * c1 + (6.0 * c2 / (v * v * v * v));
}

// Solver Newton-Raphson untuk menemukan v_opt
double newton_raphson_solver(double c1, double c2, double v0, double tolerance, int max_iter, bool& converged) {
    double v_current = v0;
    converged = false;
    for (int iter = 0; iter < max_iter; ++iter) {
        double f_val = f_drone(v_current, c1, c2);
        double df_val = df_drone(v_current, c1, c2);

        if (std::abs(df_val) < 1e-10) return v_current; // Gagal, turunan nol

        double v_next = v_current - f_val / df_val;
        if (v_next <= 0) return v_current; // Gagal, kecepatan non-fisik

        if (std::abs(v_next - v_current) < tolerance) {
            converged = true;
            return v_next;
        }
        v_current = v_next;
    }
    return v_current; // Gagal, iterasi maksimum
}


// --- Fungsi untuk Analisis Integrasi & Diferensiasi ---

// Fungsi daya drone P(v)
double power_consumption(double v, double c1, double c2) {
    if (v <= 1e-9) return 0;
    return c1 * std::pow(v, 3) + c2 / v;
}

// Diferensiasi Akurasi Tinggi (Centered, O(h^4)) dari Bab 23
double high_accuracy_diff(const std::function<double(double)>& f, double x, double h) {
    double f_xp1 = f(x + h), f_xm1 = f(x - h);
    double f_xp2 = f(x + 2*h), f_xm2 = f(x - 2*h);
    return (-f_xp2 + 8*f_xp1 - 8*f_xm1 + f_xm2) / (12 * h);
}

// Aturan Trapesium (Helper untuk Romberg)
double trapezoidal_solver(const std::function<double(double)>& f, double a, double b, int n) {
    double h = (b - a) / n;
    double sum = 0.5 * (f(a) + f(b));
    for (int i = 1; i < n; i++) sum += f(a + i * h);
    return h * sum;
}

// Integrasi Romberg (Bab 22)
double romberg_solver(const std::function<double(double)>& f, double a, double b, int max_iter = 6) {
    std::vector<std::vector<double>> I(max_iter, std::vector<double>(max_iter, 0.0));
    for (int i = 0; i < max_iter; i++) I[i][0] = trapezoidal_solver(f, a, b, 1 << i);
    for (int k = 1; k < max_iter; k++) {
        for (int j = 0; j < max_iter - k; j++) {
            I[j][k] = (std::pow(4, k) * I[j + 1][k - 1] - I[j][k - 1]) / (std::pow(4, k) - 1.0);
        }
    }
    return I[0][max_iter - 1];
}


// ===================================================================================
// BAGIAN 2: FUNGSI UTAMA (MAIN)
// ===================================================================================

int main() {
    std::string input_filename = "synthetic_data.txt";
    std::ifstream infile(input_filename);

    if (!infile.is_open()) {
        std::cerr << "Error: Tidak dapat membuka file '" << input_filename << "'.\n";
        std::cerr << "Pastikan file tersebut ada atau jalankan program Data.cpp terlebih dahulu.\n";
        return 1;
    }

    std::cout << "Membaca data dari '" << input_filename << "' untuk analisis komprehensif...\n";
    
    std::string line;
    int case_num = 0;
    while (std::getline(infile, line)) {
        case_num++;
        std::stringstream ss(line);
        double c1, c2, v0, tol;
        int max_iter;

        if (!(ss >> c1 >> c2 >> v0 >> tol >> max_iter)) {
            std::cerr << "Error parsing baris ke-" << case_num << ". Format tidak valid.\n";
            continue;
        }

        std::cout << "\n=======================================================================\n";
        std::cout << "KASUS UJI " << case_num << ": c1=" << c1 << ", c2=" << c2 << ", v0=" << v0 << "\n";
        std::cout << "=======================================================================\n\n";
        
        // --- TAHAP 1: MENEMUKAN KECEPATAN OPTIMAL (PENCARIAN AKAR) ---
        std::cout << "--- TAHAP 1: Menemukan Kecepatan Optimal (Metode Newton-Raphson) ---\n";
        bool converged = false;
        double v_opt = newton_raphson_solver(c1, c2, v0, tol, max_iter, converged);
        double v_analytical = std::pow(c2 / c1, 0.25);
        
        if (!converged) {
            std::cout << "Status: Newton-Raphson tidak konvergen. Menggunakan solusi analitik sebagai gantinya.\n";
            v_opt = v_analytical;
        }
        
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "Kecepatan Optimal (v_opt) Numerik  : " << v_opt << " m/s\n";
        std::cout << "Kecepatan Optimal (v_opt) Analitik : " << v_analytical << " m/s\n\n";

        // --- TAHAP 2: ANALISIS PADA KECEPATAN OPTIMAL ---

        // A. Analisis Diferensiasi (Bab 23)
        std::cout << "--- TAHAP 2a: Menganalisis Laju Perubahan Daya pada v_opt (Diferensiasi) ---\n";
        auto power_func = [&](double v){ return power_consumption(v, c1, c2); };
        
        double h_diff = 0.01;
        double dp_dv_numerical = high_accuracy_diff(power_func, v_opt, h_diff);
        double dp_dv_analytical = 3.0 * c1 * std::pow(v_opt, 2) - c2 / std::pow(v_opt, 2);

        std::cout << "Laju Perubahan Daya (dP/dv) pada v_opt:\n";
        std::cout << "  - Hasil Numerik (Akurasi Tinggi): " << dp_dv_numerical << " Watt/(m/s)\n";
        std::cout << "  - Hasil Analitik                : " << dp_dv_analytical << " Watt/(m/s)\n\n";

        // B. Analisis Integrasi (Bab 21 & 22)
        std::cout << "--- TAHAP 2b: Menganalisis Energi Manuver ke v_opt (Integrasi) ---\n";
        const double v_maneuver_awal = 1.0;
        const double T_maneuver = 10.0; // Durasi manuver 10 detik

        auto velocity_maneuver = [&](double t) {
            double sin_term = std::sin((M_PI * t) / (2.0 * T_maneuver));
            return v_maneuver_awal + (v_opt - v_maneuver_awal) * sin_term * sin_term;
        };
        auto power_maneuver_time = [&](double t) {
            return power_consumption(velocity_maneuver(t), c1, c2);
        };

        std::cout << "Menghitung total energi untuk manuver dari " << v_maneuver_awal << " m/s ke " << v_opt << " m/s selama " << T_maneuver << " detik.\n";
        
        // Menggunakan Integrasi Romberg (Bab 22) sebagai referensi akurat
        double energy_romberg = romberg_solver(power_maneuver_time, 0, T_maneuver);
        std::cout << "Total Energi yang Dibutuhkan (dihitung dengan Integrasi Romberg):\n";
        std::cout << "  - Energi Total: " << energy_romberg << " Joule\n\n";
    }

    infile.close();
    return 0;
}