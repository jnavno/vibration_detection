import serial
import os
import time

# Specify your USB port and baud rate
SERIAL_PORT = '/dev/ttyUSB0'  # Make sure this is the correct port
BAUD_RATE = 115200
RESULTS_FOLDER = './results'  # Folder to save the CSV files

# Ensure the results folder exists
if not os.path.exists(RESULTS_FOLDER):
    os.makedirs(RESULTS_FOLDER)
    print(f"Created folder: {RESULTS_FOLDER}")

def save_csv(filename, data):
    try:
        with open(os.path.join(RESULTS_FOLDER, filename), 'w') as file:
            file.write(data)
        print(f"Data saved to {os.path.join(RESULTS_FOLDER, filename)}")
    except Exception as e:
        print(f"Error saving file {filename}: {e}")

def listen_to_serial():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port {SERIAL_PORT}: {e}")
        return

    filename = None
    data_buffer = []
    last_data_time = time.time()

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                last_data_time = time.time()  # Reset timeout

                # Check for file transfer start
                if line.startswith("Sending file:"):
                    filename = line.split(':')[1].strip()
                    print(f"Receiving file: {filename}")
                    data_buffer = []  # Clear buffer for new file

                elif line == "File transfer complete.":
                    if filename:
                        # Save the received data to a CSV file
                        save_csv(filename, "\n".join(data_buffer))
                        filename = None

                elif filename:
                    # Accumulate the data in the buffer
                    data_buffer.append(line)

            # If no data is received for 30 seconds, assume the transfer is done
            if time.time() - last_data_time > 30 and filename:
                print(f"File transfer timeout for {filename}, saving incomplete data.")
                save_csv(filename, "\n".join(data_buffer))
                filename = None

    except KeyboardInterrupt:
        print("Stopping file retrieval...")

    except Exception as e:
        print(f"Unexpected error: {e}")

    finally:
        ser.close()
        print(f"Closed serial connection on {SERIAL_PORT}")

if __name__ == "__main__":
    listen_to_serial()
