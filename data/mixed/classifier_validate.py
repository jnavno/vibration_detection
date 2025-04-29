"""
Description:
-------------
This script classifies `.csv` vibration log files in a given folder into one of
three categories: Likely Machete, Likely Chainsaw, or Likely Ambient. It uses
wavelet decomposition and FFT analysis to extract key features from the signal.

Each file's classification result is printed to the terminal.

Usage:
-------
1. Update the 'folder_to_classify' variable to point to the folder with `.csv` files.
2. Open a terminal and navigate into the project directory.
3. Run the script using:

   python3 classifier_validate.py

Requirements:
--------------
- Python 3
- pandas
- numpy
- pywt
- scipy

Notes:
------
- Files are classified based on:
    - High-frequency wavelet energy ratios (D1+D2 vs D3–D5).
    - Dominant frequency from FFT.
    - Peak count from high-frequency detail coefficients.
- This script is intended for validation and tuning of offline classification
  before embedded deployment.

"""

import os
import pandas as pd
import numpy as np
import pywt
from scipy.fft import fft, fftfreq
from pathlib import Path
import random

# === CONFIG ===
fs = 333
duration = 12
num_samples = fs * duration
wavelet = 'db4'
level = 5

# === Set folder with mixed data ===
folder_to_classify = Path("/home/anda/Documents/repositories/iot_projects/vibration_detection/data/mixed")

def extract_features(filepath):
    df = pd.read_csv(filepath)
    signal = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
    signal = (signal - np.mean(signal)) / np.std(signal)
    signal = signal[:num_samples]

    # FFT
    yf = fft(signal)
    xf = fftfreq(len(signal), 1 / fs)
    fft_magnitude = np.abs(yf[:len(signal)//2])
    dominant_freq = xf[np.argmax(fft_magnitude)]

    # Wavelet
    coeffs = pywt.wavedec(signal, wavelet, level=level)
    energies = [np.sum(np.square(c)) for c in coeffs[1:]]
    total_energy = sum(energies)
    d1 = coeffs[1]

    d1_ratio = energies[4] / total_energy
    d2_ratio = energies[3] / total_energy
    d3_d5_ratio = sum(energies[0:3]) / total_energy

    peak_count = np.sum(np.abs(d1) > 0.2 * max(np.abs(d1)))

    return d1_ratio, d2_ratio, d3_d5_ratio, dominant_freq, peak_count

def classify(d1_ratio, d2_ratio, d3_d5_ratio, dominant_freq, peak_count, filename=None):
    total_high_freq_energy = d1_ratio + d2_ratio

    if filename:
        print(f"\n🔍 {filename}:")
        print(f"    - d1+d2 energy: {total_high_freq_energy:.2f}")
        print(f"    - d3–d5 energy: {d3_d5_ratio:.2f}")
        print(f"    - dominant_freq: {dominant_freq:.2f} Hz")
        print(f"    - peak_count: {peak_count}")

    # --- TUNED CONDITIONS ---
    # Machete detection
    if 10 <= peak_count <= 35 and total_high_freq_energy > 0.55 and dominant_freq < 15:
        return "✅ Likely Machete"
    # Chainsaw detection
    elif peak_count >= 30 and total_high_freq_energy > 0.6 and 50 <= dominant_freq <= 130:
        return "🔧 Likely Chainsaw"
    # Ambient detection
    elif peak_count < 10 and d3_d5_ratio > 0.5 and dominant_freq < 10:
        return "🌬️ Likely Ambient"
    # Non-event filter (dominant freq too low for chainsaw)
    elif total_high_freq_energy > 0.6 and dominant_freq < 30:
        return "⚠️ Possibly Non-Event (low freq)"
    else:
        return "⚠️ Unclassified"


# === Run classification ===
all_files = list(folder_to_classify.glob("*.csv"))
random.shuffle(all_files)

print("🔎 Classifying vibration logs:\n")
for filepath in all_files:
    d1, d2, d3_d5, freq, peaks = extract_features(filepath)
    result = classify(d1, d2, d3_d5, freq, peaks, filename=filepath.name)
    print(f"{filepath.name:30} --> {result}")
