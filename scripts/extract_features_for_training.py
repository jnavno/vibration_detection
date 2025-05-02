import os
import pandas as pd
import numpy as np
import pywt
from scipy.fft import fft, fftfreq
from scipy.signal import find_peaks
from pathlib import Path

# === TO RUN IT ===
#   Unzip your /mixed.zip to a folder named mixed.
#   Place this script in the same directory.
#   Run from terminal:
#   python3 extract_features_for_training.py

# === CONFIG ===
fs = 333
duration = 12
num_samples = fs * duration
wavelet = 'db4'
level = 5

input_folder = Path("/home/anda/Documents/repositories/iot_projects/vibration_detection/data/mixed")
output_file = "enhanced_classified_results.csv"

def fft_band_energy_ratio(signal, fs):
    yf = fft(signal)
    xf = fftfreq(len(signal), 1 / fs)
    mag = np.abs(yf[:len(signal)//2])
    xf = xf[:len(mag)]
    total_energy = np.sum(mag)
    return (
        np.sum(mag[(xf >= 0) & (xf <= 25)]) / total_energy,
        np.sum(mag[(xf >= 125) & (xf <= 200)]) / total_energy
    )

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

def infer_label_from_filename(filename):
    name = filename.lower()
    if name.startswith("mac_"):
        return "machete"
    elif name.startswith("chain_"):
        return "chainsaw"
    elif name.startswith("non_event_"):
        return "non_event"
    elif name.startswith("accel_"):
        return "chainsaw"  # You may adjust this depending on your intent
    else:
        return "unknown"

def extract_features(filepath):
    try:
        df = pd.read_csv(filepath)
        if not {'accel_x', 'accel_y', 'accel_z'}.issubset(df.columns):
            print(f"⚠️ Skipping {filepath.name}: Missing accel columns.")
            return None
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

        features = {
            'filename': filepath.name,
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
            'max_z': max_z,
            'true_label': infer_label_from_filename(filepath.name)
        }
        return features
    except Exception as e:
        print(f"❌ Error processing {filepath.name}: {e}")
        return None

def main():
    files = list(input_folder.glob("*.csv"))
    print(f"🔍 Found {len(files)} CSV files.")

    results = []
    for f in files:
        features = extract_features(f)
        if features:
            results.append(features)

    if results:
        df = pd.DataFrame(results)
        df.to_csv(output_file, index=False)
        print(f"\n✅ Saved results with true labels to {output_file}")
    else:
        print("\n⚠️ No features extracted.")

if __name__ == "__main__":
    main()
