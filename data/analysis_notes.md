---
title: Data Analysis for Vibration Detection Algorithm
version: 1.1
author: Pep Navarro / UVERD ltd
last_updated: 2025-04-24
linked_data: /data
colab notebook: https://colab.research.google.com/drive/1DWP2NGFHoYyxrpiyTw46pv7zPegXE_I-#scrollTo=ACheh23KPJBc&uniqifier=1
---

## Objective

To validate and calibrate the vibration classification algorithm by comparing simulated outputs with controlled real-world vibration data.

---

## Scenario Under Test

**Chainsaw && Machete cutting at 70, 2000 and 3000 cm from the sensor for 5 minutes**

---

## Evaluation Criteria

###  Ground Truth Validation
Confirm whether the classifier matches real-world vibration patterns.

###  Baseline Calibration
Fine-tune thresholds and band energy parameters using known, noisy data.

###  Event Shape Matching
Ensure frequency patterns (e.g., machete = D3–D4 peaks) apply to real wood strikes.

###  Comparative Analysis
Use chainsaw patterns as a benchmark to distinguish other events like machete or wind.

---

## Workflow

> **Note:** The following steps are supported by Python scripts described in the [Scripts Used for Analysis](#scripts-used-for-analysis) section.


1. Select 5 random `.csv` files from the designated dataset folder.
2. For each file:
   - Plot the time-domain signal for all three axes (X, Y, Z).
   - Perform Discrete Wavelet Transform (DWT) up to level 5 using `db4`.
   - Compute energy in D1 to D5 subbands.
   - Normalize and analyze energy distribution.
   - Apply classifier logic to infer vibration source.
   - Record classifier output alongside known event label (e.g., "Chainsaw").
3. Summarize classification accuracy and mismatches.
4. Add observations or anomalies to the analysis notes.

The following Python scripts support the analysis workflow described below.
---

## Data Collection Conditions

- Sensor orientation: **Vertical**, aligned to X-axis  
  - ⚠️ Y and Z signals are nearly superimposed  
  - ➕ Suggest trying **horizontal placement**

- Power supply: **Battery only**  
  - ⚠️ USB cable introduces mechanical damping

- New variables now logged:
  - 🌡️ Temperature
  - 📈 Total acceleration magnitude

---

## Visual facts Observable from plotted graphs



| Tool            | Visual Trait      | Frequency Domain      | Time Domain  |
|-----------------|-------------------|----------------------|------------|
| Machete         | Few sharp spikes    | High energy in D1+D2 (wavelet)    | Sparse, strong peaks    |
| Chainsaw        | Dense buzzing        | High energy between 150–250 Hz     | Dense oscillation |
| Ambient noise   | Flat/noisy        | Low energy all around   | no structure    |

## Scripts Used for Analysis

Below is a summary of the Python scripts used throughout the vibration data collection and analysis process. Each script focuses on a different phase, from initial visualization to feature extraction and classification validation.

| Script Name                  | Purpose |
|-------------------------------|---------|
| `batch_plot_vibrations.py`    | Processes all `.csv` files in a selected folder and generates a `_plotted.pdf` for each file. Each PDF includes: (1) combined vibration magnitude, (2) FFT spectrum, and (3) individual X, Y, Z axis plots. Useful for quick visual inspection of new data batches. |
| `build_feature_dataset.py`    | Extracts features such as high-frequency energy ratios (D1, D2), low-frequency energy (D3–D5), dominant FFT frequency, and peak counts from labeled datasets (machete, chainsaw, ambient). Saves the compiled features into a single `vibration_profile.csv` file to support classifier calibration and threshold tuning. |
| `classifier_validate.py`      | Validates the classification algorithm by automatically running feature extraction and classification on all `.csv` files in the mixed test dataset. Prints detection results for each file, helping verify real-world classifier performance. |
| `esp_data_retrieval.py`        | Connects to an ESP32 device via serial port and automatically saves incoming vibration `.csv` files to a local `results/` directory. Used during fieldwork to retrieve vibration data without manually accessing the microSD card. |

Each script is located inside the `/scripts` folder of the repository. Paths inside the scripts may need manual updates to point to the correct dataset folders during operation.


## Extraction of Visual Traits as Features

- Machete
    Short spikes, separated by ~3250 samples. Height: -2.5 to 15, usually ~10.
    Machete shows consistently high sharp peaks > 5.
    Y and Z nearly superimposed (high energy), X offset at higher baseline with sharp peaks.
  
*TODO Candidate Features for Machete detection in .md HERE
  
- Chainsaw
- Ambient noise

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

| Version | Date       | Description                                |
|---------|------------|--------------------------------------------|
| 1.0     | 2025-04-23 | Initial draft                              |
| 1.1     | 2025-04-24 | Structured doc, added checklist/test table |
