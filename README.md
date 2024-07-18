### Accelerometer-Based Vibration Pattern Detection System

---

#### Overview
This project implements an accelerometer-based system to detect tree shaking, a phenomenon indicating potential threats to trees due to external mechanical forces, such as logging. It utilizes an ESP32 S3 microcontroller, Adafruit MPU6050 accelerometer, and SPIFFS filesystem for data storage.

---

#### Key Features
- **Accelerometer Sensing**: The system measures acceleration along three axes to detect tree shaking events.
- **Threshold Sensitivity**: Adjustable threshold values enable fine-tuning of sensitivity to detect shaking events.
- **Data Logging**: Accelerometer data and computed work values are logged into a CSV file on the SPIFFS filesystem.
- **Alarm System**: An alarm is triggered if the detected shaking surpasses predefined thresholds, indicating potential threats to trees.
- **Low Power Mode**: The system utilizes low power modes to conserve energy when not actively sensing.

---

#### Components
- **ESP32 Microcontroller**: Manages system operations, sensor interfacing, and data logging.
- **Adafruit MPU6050**: Accelerometer sensor for measuring tree movements.
- **SPIFFS Filesystem**: Used for storing accelerometer data in CSV format.
- **LED Indicator**: Provides visual feedback, indicating system status and alarm activation.

---

### Prototype Images

Here are some images of the current prototype:

![Prototype Front View](docs/images/shake_detection_proto_esp32s3.jpg)
*Front view of the prototype.*

![Prototype Side View](docs/images/shake_detection_proto_side_esp32s3.jpg)
*Side view of the prototype.*

#### Setup and Operation
1. **Hardware Setup**: Connect the MPU6050 sensor to the ESP32 over I2C. Add a simple diode scheme to disable the shake detector upon wakeup of the module.
2. **Software Configuration**: Define threshold values and sampling duration in the code.
3. **Deployment**: Rigidity and weight of the enclosure matter. You'll need to hardcode the exact weight of your enclosure. Also, install the prototype onto the tree with a rigid device, such as PVC, so that vibrations in the trunk can transfer to the accelerometer.
4. **Operation**: The system continuously samples accelerometer data, logging it to the SPIFFS filesystem. If shaking surpasses predefined thresholds, an alarm is activated. No vibration will put the system into deep sleep. A simple IRFZ44N acts as an off switch for the shake detector while the accelerometer computes the vibration pattern. The LED does the alarm functionality indicating whether or not a logging is happening to the tree.
5. **Analysis**: Analyze logged data to identify shaking events and assess potential threats to trees.
6. **Maintenance**: Periodically check and maintain the system to ensure accurate monitoring and alarm functionality.

---

#### License
This project is released under the [MIT License](LICENSE.md).

---

#### Author
Josep Navarro, UVERD ltd

---

#### Acknowledgments
- Adafruit Industries for the MPU6050 library.
- ESP32 Arduino Core developers for the development tools and libraries.

---

#### Note
This README provides a brief overview of the project. For detailed information, including code implementation and usage instructions, refer to the project documentation and source code files.
