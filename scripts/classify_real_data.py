
"""
classify_real_data.py

Purpose:
    Validate signal classification (machete, chainsaw, wind, non-event) using real CSV files
    Run FFT + wavelet-based classification on real vibration CSV files.
    Detects likely machete, chainsaw, wind, or ambient noise.

Usage:
  Run inside Jupyter/Colab or standalone with a CSV upload.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pywt
from scipy.signal import find_peaks
from scipy.fft import fft, fftfreq
from google.colab import files
from matplotlib.backends.backend_pdf import PdfPages
import os

# --- Upload CSV ---
print("📁 Upload your real vibration .csv file")
uploaded = files.upload()

# --- Load CSV ---
filename = list(uploaded.keys())[0]
df = pd.read_csv(filename)

# --- Combine 3-axis magnitude and normalize ---
signal_real = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
signal_real = (signal_real - np.mean(signal_real)) / np.std(signal_real)

# --- Auto-detect duration from sample count ---
# --- Auto-detect duration from sample count ---
fs = 333  # sampling frequency

# Use raw axis data for sample count
total_samples = len(df['accel_x'])
duration = total_samples / fs

print(f"📏 Detected {total_samples} samples → Duration = {duration:.2f} seconds")

# Now compute magnitude and normalize after confirming length
signal_real = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
signal_real = (signal_real - np.mean(signal_real)) / np.std(signal_real)

# --- FFT ---
N = len(signal_real)
yf = fft(signal_real)
xf = fftfreq(N, 1 / fs)
fft_magnitude = np.abs(yf[:N // 2])
dominant_freq = xf[:N // 2][np.argmax(fft_magnitude)]

# --- Wavelet ---
wavelet = 'db4'
level = 5
coeffs = pywt.wavedec(signal_real, wavelet, level=level)
energies = [np.sum(np.square(c)) for c in coeffs[1:]]
total_energy = sum(energies)

# Band energy ratios
d1_ratio = energies[4] / total_energy
d2_ratio = energies[3] / total_energy
d3_d5_ratio = sum(energies[0:3]) / total_energy

# --- Peak Detection in D1 ---
d1 = coeffs[1]
peak_height_threshold = 0.2 * max(np.abs(d1))
peaks, _ = find_peaks(np.abs(d1), height=peak_height_threshold, distance=20)
peaks_per_second = len(peaks) / duration

# --- Classification ---
is_high_freq = (d1_ratio + d2_ratio) > 0.75
is_low_freq = d3_d5_ratio > 0.5
has_few_peaks = len(peaks) <= 10
has_dense_peaks = peaks_per_second > 1
is_very_low_freq = dominant_freq < 10

if is_very_low_freq and is_low_freq:
    detection = "🌬️ Likely Wind (low overall frequency)"
elif has_few_peaks and is_high_freq and not is_very_low_freq:
    detection = "✅ Likely Machete (few sharp peaks, high D1+D2 energy)"
elif not has_few_peaks and has_dense_peaks and is_high_freq:
    detection = "🔧 Likely Chainsaw (dense high-freq buzz)"
elif is_low_freq and not is_high_freq:
    detection = "🟡 Possibly Ambient / Non-Event"
else:
    detection = "⚠️ Unclassified or unclear pattern"

# --- Report ---
print("\n🌲 VIBRATION DETECTION REPORT")
print("-----------------------------")
print(f"File: {filename}")
print(f"Sample Rate: {fs} Hz")
print(f"Duration: {duration} seconds")
print(f"FFT Dominant Frequency: {dominant_freq:.2f} Hz")
print()
print(f"D1+D2 energy ratio: {d1_ratio + d2_ratio:.2f}")
print(f"D3–D5 energy ratio: {d3_d5_ratio:.2f}")
print(f"Peak count: {len(peaks)}, PPS: {peaks_per_second:.2f}")
print()
print(f"🔍 Detection Result: {detection}")

# --- Plots + Save to PDF ---
basename = filename.split('/')[-1]
pdf_filename = f"{basename}_plotted.pdf"
time_axis = np.linspace(0, duration, total_samples, endpoint=False)

with PdfPages(pdf_filename) as pdf:
    # Signal plot
    plt.figure(figsize=(12, 4))
    plt.plot(time_axis, signal_real)
    plt.title(f"Real Vibration Signal — {basename}")
    plt.xlabel("Time (s)")
    plt.grid(True)
    plt.tight_layout()
    pdf.savefig()
    plt.show()
    plt.close()

    # FFT plot
    plt.figure(figsize=(12, 4))
    plt.plot(xf[:N // 2], fft_magnitude)
    plt.title(f"FFT Spectrum — {basename}")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.grid(True)
    plt.tight_layout()
    pdf.savefig()
    plt.show()
    plt.close()

    # Axis-specific plot
    plt.figure(figsize=(12, 4))
    plt.plot(df['accel_x'], label='X')
    plt.plot(df['accel_y'], label='Y')
    plt.plot(df['accel_z'], label='Z')
    plt.legend()
    plt.title(f"Axis-specific Vibration — {basename}")
    plt.xlabel("Sample Index")
    plt.grid(True)
    plt.tight_layout()
    pdf.savefig()
    plt.show()
    plt.close()

# --- Download PDF ---
print(f"📄 Saved multi-page PDF: {os.path.abspath(pdf_filename)}")
files.download(pdf_filename)
