---
title: Data Analysis for Vibration Detection Algorithm
version: 1.2
author: Pep Navarro / UVERD ltd
last_updated: 2025-05-02
linked_data: /data
colab notebook: https://colab.research.google.com/drive/1DWP2NGFHoYyxrpiyTw46pv7zPegXE_I-#scrollTo=ACheh23KPJBc&uniqifier=1
---

## Objective

To validate, refine, and prepare for embedded integration a vibration classification algorithm capable of accurately identifying chainsaw, machete, and non-event ambient activity using signals from a tree-mounted ESP32 sensor.

---

## Scenario Under Test

**Chainsaw && Machete cutting at 70, 2000 and 3000 cm from the sensor for 5 minutes**, across varied environmental and structural conditions.

---

## Evaluation Criteria

###  Ground Truth Validation
Cross-check machine classification against filename labels derived from structured field logging.

###  Baseline Calibration
Tune thresholds and extract high-resolution features across wavelet subbands and FFT windows.

###  Event Shape Matching
Ensure frequency and temporal domain markers (e.g., machete spikes, chainsaw density) correlate with classifier outputs.

###  Comparative Analysis
Use chainsaw traces as benchmark. Separate overlapping high-frequency classes (e.g., wind) with multidimensional filtering.

###  Model Robustness
Perform cross-validation and multi-model comparisons (RF, SVM, kNN) with PCA-based visualizations and confusion matrices.

---

## Workflow

1. Use real-world data files from `/mixed` folder (cleaned of outliers).
2. Extract 13 signal features including:
   - DWT energy ratios: D1, D2, D3–D5
   - FFT energy in 0–25 Hz and 125–200 Hz
   - Peak count, strike count
   - Axis offsets and max amplitude
3. Automatically infer **true labels** from filenames (`mac_`, `chain_`, `non_event_`).
4. Train and evaluate multiple classifiers (Random Forest, SVM, kNN).
5. Apply PCA for feature-space visualization and decision boundary inspection.
6. Perform stratified train/test split and k-fold cross-validation.
7. Rank feature importance by model.

---

## Real-World Field Data Conditions

- **Sensor Orientation:** Vertical (X dominant)
  - ⚠️ Y and Z nearly overlapping → Y and Z contain little differential info
  - ➕ Horizontal placement is suggested for future tests
- **Power Source:** Battery only
  - ⚠️ USB cable reduces vibration fidelity
- **New Variables Captured:**
  - 🌡️ Temperature
  - 📈 Total magnitude (XYZ combined)

---

## Model Performance Summary

### Random Forest Classifier

- **Train Accuracy:** 100%
- **Test Accuracy:** ~95%
- **Cross-Validation:** Mean accuracy ≈ 94.6% ± 2.2%
- **Confusion Matrix:**
  All three classes (chainsaw, machete, ambient) perfectly separated in final train/test split

### Support Vector Machine (RBF Kernel)

- Accuracy: 91–93%
- Notably accurate for clean boundaries but **misclassifies sparse machete signals near chainsaw zones**

### k-Nearest Neighbors (k=5)

- Accuracy: ~90%
- More sensitive to boundary jitter and noisy samples

---

## Feature Importance

Top contributors to classification according to Random Forest:

| Rank | Feature           | Impact            |
|------|-------------------|--------------------|
| 1    | strike_count       | Most powerful discriminator, esp. for machete |
| 2–4  | max_x, max_y, max_z | Reflects physical power of the event |
| 5    | fft_0_25_ratio     | Key for ambient rejection |
| 6    | d3_d5_ratio         | Used for machete vs chainsaw |
| 7–9  | peak_count, d1_ratio, high_freq_energy | All subtle but additive |

*Offset values showed low contribution and may be deprecated unless sensor orientation changes.*

---

## Visual Traits Summary

| Tool          | Time Domain       | Frequency Domain            | Observed Traits                      |
|---------------|------------------|-----------------------------|--------------------------------------|
| Machete       | Sparse high spikes | High in D1+D2, low elsewhere | ~3200-sample interval strike bursts  |
| Chainsaw      | Dense vibration   | Wideband 150–250 Hz peak     | High energy clusters                 |
| Ambient Noise | Flat / noisy      | Energy ~0 everywhere         | Low entropy                          |

---

## Key Visualizations

- 📊 **Feature Importance Bar Chart (RF + Logistic Regression)**
- 🧭 **PCA Scatter Plot of All 3 Classes**
- 🧱 **Confusion Matrices (RF vs SVM)**
- 🗺️ **Decision Boundaries: RF, SVM, kNN**

> These visuals confirm *clear separability* across classes, especially when projected to 2D via PCA.

---

## Embedded Integration Plan

The feature extraction pipeline has been tested and validated in Python. Moving forward:

1. Convert Python logic to embedded C++:
   - **Strike count**, **max amplitudes**, and **band ratios** will be derived from onboard signal streams
   - Use **threshold rules** and **lookup logic** in place of ML trees if memory-limited
2. Ensure extraction runs on 333Hz sensor sampling and under 12-second collection windows.
3. Only the top ~6 features may be embedded to save compute power.
4. Model weights or thresholds can be exported as constants or tables into firmware files.

---

## Scripts Used

| Script | Purpose |
|--------|---------|
| `extract_features_for_training.py` | Parses `.csv`, runs wavelet + FFT + axis analytics, infers true labels |
| `random_forest_eval.py`            | Loads labeled dataset, trains/test RF, shows accuracy + importance |
| `svm_knn_eval.py`                  | Runs SVM and kNN models, plots PCA decision boundaries |
| `batch_plot_vibrations.py`        | PDF generation for time/FFT plots |
| `esp_data_retrieval.py`           | Copies `.csv` from ESP32 unit to `results/` |
| `classifier_validate.py`          | Older rule-based classifier script for comparison purposes |

---




## Real-World Field Data

This folder contains real vibration logs collected from the TreeGuard sensor.

### Folder Structure

| Folder Name             | Vibration Source | Distance from Sensor | Duration | Notes             |
|-------------------------|------------------|----------------------|----------|-------------------|
| `ambient_noise/`        | None             | N/A                  | 5 min    | Ambient noise     |
| `chain_short/`          | Chainsaw         | 70 cm                | 5 min    | Short distance    |
| `chain_med/`            | Chainsaw         | 200 cm               | 5 min    | Medium distance   |
| `chain_far/`            | Chainsaw         | 300 cm               | 5 min    | Far distance      |
| `mac_short/`            | Machete          | 70 cm                | 5 min    | Short distance    |
| `mac_med/`              | Machete          | 200 cm               | 5 min    | Medium distance   |
| `mac_far/`              | Machete          | 300 cm               | 5 min    | Far distance      |

Each folder contains:
- Raw CSV logs (`accel_sd_<session><index>.csv`)
- Corresponding visualizations (`_plotted.pdf`)
  - Magnitude of X, Y, Z
  - FFT spectrum
  - Axis-specific plots


### Field Setup
- Sensor mounted around 4 - 6 m above ground on native tree trunk
- Enclosure: PVC rigid tube with no foam inner lining or any vibration damping material
- Tools used: gas chainsaw, machete, ambient conditions (wind,rain,etc)

---

## Test Checklist

- [ ] Sensor is mounted vertically, with antenna socket at the top
- [ ] Sensor is installed **above** the source of vibration  
- [ ] Device is powered by battery (no USB connected)  
- [ ] Temperature and total magnitude data are included in each recording  
- [ ] Classifier outputs are recorded for each test in the MicroSD
- [ ] Plots and subband energy breakdowns are saved for review in /data folder 

---

## Planned Test Runs

| Run ID | Vibration Source | Distance from Sensor | Duration | Notes                    |
|--------|------------------|----------------------|----------|--------------------------|
| 001    | None             | N/A                  | 5 min    | Ambient noise            |
| 002    | Chainsaw         | 70 cm                | 5 min    | Short distance           |
| 003    | Chainsaw         | 2000 cm              | 5 min    | Medium-distance          |
| 004    | Chainsaw         | 3000 cm              | 5 min    | Far distance             |
| 005    | Machete          | 70 cm                | 5 min    | Short distance           |
| 006    | Machete          | 2000 cm              | 5 min    | Medium distance          |
| 007    | Machete          | 3000 cm              | 5 min    | Far distance             |

---

## Suggested Improvements

- [ ] Name CSV files consistently (e.g., `run_001.csv`, `run_002.csv`) and edit accordingly 
- [ ] Mount sensor horizontally and repeat test runs for comparison  
- [ ] Include visual outputs (signal plots, spectrograms) in future versions of this document  
- [ ] Automate classification steps via Python batch script  
- [ ] Track temperature vs. amplitude correlations in ambient conditions  
- [ ] Store results and plots in a centralized Git repository or shared folder  
- [ ] Annotate `.csv` files with metadata (test ID, date, conditions) in header  
- [ ] Document battery level and voltage at the start and end of each test  

---

## Change Log

| Version | Date       | Description |
|---------|------------|-------------|
| 1.0     | 2025-04-23 | Initial draft |
| 1.1     | 2025-04-24 | Structured doc, added checklist/test table |
| 1.2     | 2025-05-02 | Full ML model evaluation, PCA, visualizations, embedded plan |