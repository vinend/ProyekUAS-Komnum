# Analisis Kinerja Drone dengan Metode Numerik

Repositori ini berisi kode dan hasil proyek analisis kinerja drone menggunakan metode pencarian akar, integrasi, dan diferensiasi numerik. Tujuannya adalah memberikan pemahaman holistik kinerja drone, melampaui optimasi parameter tunggal.

## Abstrak Ringkas

Seiring meningkatnya peran drone, efisiensi operasional menjadi krusial. Studi ini mengintegrasikan tiga metode numerik untuk analisis kinerja drone yang komprehensif:
1.  **Pencarian Akar (Newton-Raphson)**: Menentukan kecepatan optimal ($v_{opt}$) untuk jangkauan maksimal.
2.  **Diferensiasi Numerik**: Mengevaluasi sensitivitas konsumsi daya ($dP/dv$) terhadap perubahan kecepatan.
3.  **Integrasi Numerik (Metode Romberg)**: Menghitung total energi untuk manuver realistis.
Pendekatan ini, diimplementasikan dalam C++ dan divisualisasikan dengan Python, mengubah parameter desain menjadi wawasan kinerja praktis.

## Metodologi Analisis

Analisis dilakukan dalam tiga tahap per set parameter drone:

### 1. Kecepatan Optimal (Pencarian Akar)

Kecepatan optimal ($v_{opt}$) dicari dengan metode Newton-Raphson pada turunan fungsi rasio daya-kecepatan:
$$f(v) = \frac{d(P(v)/v)}{dv} = 2c_{1}v - 2c_{2}v^{-3} = 0$$
Model daya drone: $P(v) = c_{1}v^{3} + c_{2}/v$.

### 2. Sensitivitas Daya (Diferensiasi Numerik)

Sensitivitas daya ($dP/dv$) di sekitar $v_{opt}$ dihitung dengan formula beda hingga terpusat orde tinggi:
$$P'(v) \approx \frac{-P(v+2h) + 8P(v+h) - 8P(v-h) + P(v-2h)}{12h}$$

### 3. Energi Manuver (Integrasi Numerik)

Total energi (E) untuk manuver percepatan dari kecepatan awal ke $v_{opt}$ selama durasi T dihitung dengan Integrasi Romberg:
$$E = \int_{0}^{T} P(v(t)) dt$$

## Struktur Repositori

```
.
├── Proyek_UAS_KomputasiNumerik.pdf  # Laporan lengkap
├── combined_analysis.cpp            # Implementasi C++
├── combined_analysis.py             # Skrip Python (analisis & visualisasi)
├── synthetic_data.txt               # Contoh data input
├── 1_validasi_v_optimal.png         # Hasil: Validasi solver
├── 2_distribusi_energi.png          # Hasil: Distribusi energi
├── 3_perbandingan_profil_kecepatan.png # Hasil: Perbandingan kecepatan
├── 4_perbandingan_profil_daya.png      # Hasil: Perbandingan daya
└── 5_grid_profil_manuver.jpg        # Hasil: Grid profil manuver
```

## Cara Menjalankan

Replikasi hasil analisis dengan skrip Python:

### Prasyarat
  - Python 3
  - Library: `numpy`, `matplotlib`

### Langkah-langkah
1.  **Clone repositori:**
    ```bash
    git clone https://github.com/vinend/ProyekUAS-Komnum.git
    cd ProyekUAS-Komnum
    ```
2.  **Instal library:**
    ```bash
    pip install numpy matplotlib
    ```
3.  **Jalankan skrip Python:**
    ```bash
    python combined_analysis.py
    ```
Skrip akan menghasilkan `synthetic_data.txt`, memproses kasus uji, dan menyimpan visualisasi.

### Opsi Alternatif (C++)
1.  Pastikan `synthetic_data.txt` ada.
2.  Kompilasi C++:
    ```bash
    g++ combined_analysis.cpp -o combined_analysis -std=c++11
    ```
3.  Jalankan:
    ```bash
    ./combined_analysis
    ```
    Program akan mencetak hasil numerik ke konsol.

## Hasil Utama Ringkas

Kerangka kerja ini memvalidasi metode numerik dan memberikan wawasan desain.

**1. Validasi Solver Akurat**
Grafik validasi Newton-Raphson menunjukkan kesesuaian sempurna antara kecepatan optimal numerik dan analitik.

**2. Distribusi Energi Manuver**
Histogram menunjukkan sebaran energi untuk manuver percepatan pada 15 skenario drone, menyoroti rentang kinerja.

**3. Analisis Profil Manuver**
Grid visualisasi menunjukkan hubungan profil kecepatan dan konsumsi daya selama manuver, memperlihatkan dinamika daya.

## Kesimpulan Ringkas

Kerangka analisis numerik terpadu ini memberikan pemahaman kinerja drone yang lebih dalam, mengungkap *trade-off* antara efisiensi jelajah dan agilitas manuver. Hasil kuantitatifnya dapat diintegrasikan ke sistem robotika nyata untuk optimasi misi otonom.

## Author

  - **Andi Muhammad Alvin Farhansyah**
  - **NPM:** 2306161933
  - **Email:** andi.muhammad35@ui.ac.id
  - **GitHub:** [vinend](https://github.com/vinend/ProyekUAS-Komnum)
