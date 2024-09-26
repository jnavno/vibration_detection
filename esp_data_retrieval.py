import serial
import os

# Specify your USB port and baud rate
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
RESULTS_FOLDER = './results'  # Folder to save the CSV files

# Ensure the results folder exists
if not os.path.exists(RESULTS_FOLDER):
    os.makedirs(RESULTS_FOLDER)

def save_csv(filename, data):
    with open(os.path.join(RESULTS_FOLDER, filename), 'w') as file:
        file.write(data)
    print(f"Data saved to {os.path.join(RESULTS_FOLDER, filename)}")

def listen_to_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    filename = None
    data_buffer = []

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                
                # Check for file transfer start
                if line.startswith("Sending file:"):
                    filename = line.split(':')[1].strip()
                    print(f"Receiving file: {filename}")
                    data_buffer = []  # Clear buffer

                elif line == "File transfer complete.":
                    if filename:
                        # Save the received data to a CSV file
                        save_csv(filename, "\n".join(data_buffer))
                        filename = None

                elif filename:
                    # Accumulate the data in the buffer
                    data_buffer.append(line)

    except KeyboardInterrupt:
        print("Stopping file retrieval...")

    finally:
        ser.close()

if __name__ == "__main__":
    listen_to_serial()
