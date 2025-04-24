---
title: Data Analysis for Vibration Detection Algorithm
version: 1.1
author: Pep Navarro / UVERD ltd
last_updated: 2025-04-24
linked_data: /data
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

## Test Checklist

- [ ] Sensor is mounted vertically  
- [ ] Sensor is installed **above** the source of vibration  
- [ ] Device is powered by battery (no USB connected)  
- [ ] Temperature and total magnitude data are included in each recording  
- [ ] Classifier outputs are recorded for each test in MicroSD 
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
