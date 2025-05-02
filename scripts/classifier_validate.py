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
from scipy.signal import find_peaks
from pathlib import Path
import random

# === CONFIG ===
fs = 333
duration = 12
num_samples = fs * duration
wavelet = 'db4'
level = 5

# === Set folder and output file ===
folder_to_classify = Path("/home/anda/Documents/repositories/iot_projects/vibration_detection/data/mixed")
output_csv = "classified_results.csv"


def fft_band_energy_ratio(signal, fs):
    yf = fft(signal)
    xf = fftfreq(len(signal), 1 / fs)
    mag = np.abs(yf[:len(signal)//2])
    xf = xf[:len(mag)]

    total_energy = np.sum(mag)
    band_0_25 = np.sum(mag[(xf >= 0) & (xf <= 25)])
    band_125_200 = np.sum(mag[(xf >= 125) & (xf <= 200)])

    return band_0_25 / total_energy, band_125_200 / total_energy


def count_short_strikes(signal, fs, max_width=125):
    peaks, _ = find_peaks(signal, height=0.5, width=(1, max_width))
    return len(peaks)


def axis_features(df):
    offset_x_y = np.mean(df['accel_x'] - df['accel_y'])
    offset_y_z = np.mean(df['accel_y'] - df['accel_z'])

    max_amp_x = np.max(np.abs(df['accel_x']))
    max_amp_y = np.max(np.abs(df['accel_y']))
    max_amp_z = np.max(np.abs(df['accel_z']))

    return offset_x_y, offset_y_z, max_amp_x, max_amp_y, max_amp_z


def extract_features(filepath):
    df = pd.read_csv(filepath)
    df = df.iloc[:num_samples]

    signal = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
    signal = (signal - np.mean(signal)) / np.std(signal)

    coeffs = pywt.wavedec(signal, wavelet, level=level)
    energies = [np.sum(np.square(c)) for c in coeffs[1:]]
    total_energy = sum(energies)
    d1_ratio = energies[4] / total_energy
    d2_ratio = energies[3] / total_energy
    d3_d5_ratio = sum(energies[0:3]) / total_energy
    peak_count = np.sum(np.abs(coeffs[1]) > 0.2 * max(np.abs(coeffs[1])))

    band_0_25, band_125_200 = fft_band_energy_ratio(signal, fs)
    strike_count = count_short_strikes(signal, fs)
    offset_x_y, offset_y_z, max_x, max_y, max_z = axis_features(df)

    return {
        'd1_ratio': d1_ratio,
        'd2_ratio': d2_ratio,
        'd3_d5_ratio': d3_d5_ratio,
        'high_freq_energy': d1_ratio + d2_ratio,
        'peak_count': peak_count,
        'fft_0_25_ratio': band_0_25,
        'fft_125_200_ratio': band_125_200,
        'strike_count': strike_count,
        'offset_x_y': offset_x_y,
        'offset_y_z': offset_y_z,
        'max_x': max_x,
        'max_y': max_y,
        'max_z': max_z
    }


def classify(features):
    hf_energy = features['high_freq_energy']
    strike_count = features['strike_count']
    band_0_25 = features['fft_0_25_ratio']
    band_125_200 = features['fft_125_200_ratio']
    max_x, max_y, max_z = features['max_x'], features['max_y'], features['max_z']
    offset_x_y, offset_y_z = features['offset_x_y'], features['offset_y_z']

    # ✅ Machete
    if (
        1 <= strike_count <= 100 and
        hf_energy >= 0.4 and
        band_0_25 > 0.3 and
        band_125_200 < 0.22 and
        offset_x_y >= 16000 and
        abs(offset_y_z) <= 1200 and
        max_y > 15000 and max_z > 15000
    ):
        return "✅ Likely Machete"

    # 🔧 Chainsaw
    elif (
        features['peak_count'] > 30 and
        strike_count > 200 and
        hf_energy > 0.6 and
        band_0_25 < 0.3 and
        band_125_200 > 0.15 and
        (max_y > 3000 or max_z > 3000)
    ):
        return "🔧 Likely Chainsaw"

    # 🌬️ Non-Event (refined)
    elif (
        max_y < 2000 and
        max_z < 2000 and
        offset_x_y > 16000 and
        0.12 < band_125_200 < 0.27 and
        hf_energy > 0.7 and
        strike_count > 600
    ):
        return "🌬️ Likely Non-Event"

    # ⚠️ Vibration Detected (ambiguous)
    elif (
        hf_energy > 0.5 and
        strike_count > 20 and
        (max_y > 1000 or max_z > 1000)
    ):
        return "⚠️ Vibration Detected (ambiguous)"

    # ⚠️ Fully Unclassified
    return "⚠️ Unclassified"


# === MAIN ===
all_files = list(folder_to_classify.glob("*.csv"))
random.shuffle(all_files)

results = []

print("🔎 Classifying vibration logs:\n")
for filepath in all_files:
    features = extract_features(filepath)
    result = classify(features)

    print(f"\n🔍 {filepath.name}:")
    for key, val in features.items():
        print(f"    - {key}: {val:.2f}" if isinstance(val, float) else f"    - {key}: {val}")
    print(f"{filepath.name:30} --> {result}")

    results.append({
        'filename': filepath.name,
        **features,
        'classification': result
    })

# Save to CSV
df_results = pd.DataFrame(results)
df_results.to_csv(output_csv, index=False)
print(f"\n✅ Results saved to {output_csv}")