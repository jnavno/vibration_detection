""" Description:
-------------
This script processes all `.csv` files in the current folder and generates
a `_plotted.pdf` file for each. Each PDF contains three pages:
1. Combined vibration signal (magnitude of X, Y, Z)
2. FFT spectrum
3. Individual X, Y, Z axis vibration plots

Usage:
-------
1. Copy this script into any folder that contains `.csv` vibration data files.
2. Update the folder_path variable to match the folder you are on.
3. Open a terminal and navigate into that folder.
4. Run the script using:

   python3 batch_plot_vibrations_current_dir.py

Requirements:
--------------
- Python 3
- pandas
- numpy
- matplotlib
- scipy

Each `_plotted.pdf` file will be saved in the same folder as its corresponding `.csv`.

"""
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from matplotlib.backends.backend_pdf import PdfPages

# === CONFIGURATION ===
fs = 333
duration = 12
folder_path = "/home/anda/MEGA/UVERD/hardware/Design Files/_Ecominga_/_TreeGuard_/Docs/vibration_detection_data/no_vibra_5min"


# === PROCESS EACH CSV FILE IN THE FOLDER ===
for filename in os.listdir(folder_path):
    if filename.endswith(".csv"):
        full_path = os.path.join(folder_path, filename)
        print(f"🔍 Processing: {filename}")

        # Load data
        df = pd.read_csv(full_path)
        signal_real = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
        signal_real = (signal_real - np.mean(signal_real)) / np.std(signal_real)
        signal_real = signal_real[:fs * duration]

        # FFT
        N = len(signal_real)
        yf = fft(signal_real)
        xf = fftfreq(N, 1 / fs)
        fft_magnitude = np.abs(yf[:N // 2])

        # Output PDF filename
        base = os.path.splitext(filename)[0]
        pdf_filename = os.path.join(folder_path, f"{base}_plotted.pdf")

        # Generate PDF with all 3 plots
        with PdfPages(pdf_filename) as pdf:
            # Plot 1: Combined signal
            plt.figure(figsize=(12, 4))
            plt.plot(signal_real)
            plt.title(f"Real Vibration Signal — {base}")
            plt.grid(True)
            plt.tight_layout()
            pdf.savefig()
            plt.close()

            # Plot 2: FFT
            plt.figure(figsize=(12, 4))
            plt.plot(xf[:N // 2], fft_magnitude)
            plt.title(f"FFT Spectrum — {base}")
            plt.xlabel("Frequency (Hz)")
            plt.ylabel("Amplitude")
            plt.grid(True)
            plt.tight_layout()
            pdf.savefig()
            plt.close()

            # Plot 3: Axis-specific
            plt.figure(figsize=(12, 4))
            plt.plot(df['accel_x'], label='X')
            plt.plot(df['accel_y'], label='Y')
            plt.plot(df['accel_z'], label='Z')
            plt.legend()
            plt.title(f"Axis-specific Vibration — {base}")
            plt.grid(True)
            plt.tight_layout()
            pdf.savefig()
            plt.close()

        print(f"✅ Saved to: {pdf_filename}")