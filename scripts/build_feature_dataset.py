"""
Description:
-------------
This script processes labeled vibration `.csv` files from three categories
(machete, chainsaw, and ambient noise) to extract key features. It saves the
extracted features and their labels into a single CSV file for further analysis
and classifier tuning.

Usage:
-------
1. Ensure the folders 'mac_short', 'chain_short', and 'ambient_noise' each contain `.csv` vibration data files.
2. Update the folder paths at the top of the script if needed.
3. Open a terminal and navigate into the project directory.
4. Run the script using:

   python3 extract_features_to_csv.py

Requirements:
--------------
- Python 3
- pandas
- numpy
- pywt
- scipy

Notes:
------
- For each vibration file, the script extracts:
    - d1_ratio (high-frequency energy in D1 band)
    - d2_ratio (high-frequency energy in D2 band)
    - d3_d5_ratio (lower-frequency energy combined from D3–D5)
    - dominant_freq (dominant frequency from FFT)
    - peak_count (number of strong peaks in D1 detail coefficients)
- The extracted features are saved to 'vibration_profile.csv' for easy
  visualization and classifier threshold optimization.

"""

import os
import pandas as pd
import numpy as np
import pywt
from scipy.fft import fft, fftfreq
from pathlib import Path

# === Set paths for each category ===
folder_machete = Path("/home/anda/Documents/repositories/iot_projects/vibration_detection/data/mac_short")
folder_chainsaw = Path("/home/anda/Documents/repositories/iot_projects/vibration_detection/data/chain_short")
folder_ambient = Path("/home/anda/Documents/repositories/iot_projects/vibration_detection/data/ambient_noise")
output_path = folder_machete.parent / "vibration_profile.csv"  # <-- now saving CSV

# === Config ===
fs = 333
duration = 12
num_samples = fs * duration
wavelet = 'db4'
level = 5

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

    return {
        "d1_ratio": d1_ratio,
        "d2_ratio": d2_ratio,
        "d3_d5_ratio": d3_d5_ratio,
        "dominant_freq": dominant_freq,
        "peak_count": np.sum(np.abs(d1) > 0.2 * max(np.abs(d1)))
    }

def process_folder(folder, label):
    features = []
    for file in folder.glob("*.csv"):
        f = extract_features(file)
        f["label"] = label
        features.append(f)
    return features

# === Process each class ===
print("🔍 Extracting features from training sets...")
all_features = []
all_features += process_folder(folder_machete, "machete")
all_features += process_folder(folder_chainsaw, "chainsaw")
all_features += process_folder(folder_ambient, "ambient")

df_features = pd.DataFrame(all_features)
df_features.to_csv(output_path, index=False)  # <-- save as CSV now
print(f"✅ Feature profile saved to {output_path}")
