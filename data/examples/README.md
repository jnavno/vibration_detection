# Example Vibration Data Samples

This folder contains a **curated set of representative vibration logs** used to demonstrate and test the vibration classification algorithm.

Each CSV file here corresponds to a different class of vibration detected by the tree-mounted ESP32 sensor system.

## Categories Included

| File Name           | Vibration Class     | Description                                 |
|---------------------|---------------------|---------------------------------------------|
| `chainsaw_1.csv`    | Likely Chainsaw     | Dense, continuous vibration (150–250 Hz)    |
| `machete_1.csv`     | Likely Machete      | Isolated, high-amplitude strikes            |
| `ambient_1.csv`     | Ambient Noise       | Background wind or still conditions         |
| `non_event_1.csv`   | Non-Event           | Small or accidental bumps, low significance |
| `ambiguous_1.csv`   | Ambiguous Vibration | Indeterminate — does not fit core classes   |

These samples are small in size and are safe to keep in the repository for illustrative and testing purposes.

## Usage

You can use these files to:

- Test the feature extraction pipeline
- Train a mini version of the classifier
- Explore the classifier decision-making logic
- Generate visualizations (FFT, DWT, etc.)

## Full Dataset

The full dataset used to train and evaluate the complete model contains **dozens of sessions** across multiple distances and conditions.

📦 **Download Full Dataset:**
[Google Drive (read-only)](https://your-shared-link-here)

Or refer to the main analysis document for more details:
📄 [`/data/Data_Analysis.md`](../Data_Analysis.md)

## Notes

- All files were recorded with the ESP32-based TreeGuard prototype, using an MPU6050 sensor at 333 Hz.
- Raw values represent acceleration in 3 axes (X, Y, Z) and derived magnitude.
