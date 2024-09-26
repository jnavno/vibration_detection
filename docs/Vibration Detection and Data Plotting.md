# Prerequisites

### Hardware:
- ESP32 development board
- MPU6050 accelerometer
- USB cable for connection to your computer

### Software:
- Arduino IDE or PlatformIO for code development
- Python installed on your local machine
- `pySerial` Python library: Install with `pip3 install pyserial`
- LibreOffice Calc (or another tool) for plotting data

---

# Steps to Set Up and Start Collecting Data

## 1. Clone the Project and Flash the Code

1. Clone the project repository.

    ```bash
    git clone <your-repo-url>
    cd <project-directory>
    ```

2. Open the `real_time_calibration.cpp` file in your preferred IDE (Arduino IDE/PlatformIO).

3. Connect the ESP32 to your computer using the USB cable.

4. Flash the `real_time_calibration.cpp` to the ESP32.

---

## 2. Setting Up the Python Script for Data Retrieval

1. Save the `esp_data_retrieval.py` Python script in your project folder.

2. Install `pyserial` if not already installed:

    ```bash
    pip3 install pyserial
    ```

3. Run the Python script to listen to the serial port and capture the incoming CSV data:

    ```bash
    python3 esp_data_retrieval.py
    ```

    The script will automatically save the CSV files into a local folder called `results`.

---

## 3. Retrieve and Plot the Data

1. Once the ESP32 finishes collecting data (3 phases of 30 seconds), the data will be transferred to your local machine and stored in the `results` folder.

2. Open the `.csv` files in LibreOffice Calc:
   - Navigate to the `results` folder.
   - Open the desired CSV file in LibreOffice Calc and choose to plot the time (seconds) against the g-force data on the X-axis.

3. Analyze the plot to observe vibration patterns and events such as tree logging.
