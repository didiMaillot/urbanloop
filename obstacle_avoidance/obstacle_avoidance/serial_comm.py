
import serial  # Import the serial library for serial communication
import time

def main():
    # Initialize the serial connection to ESP32 (adjust the port and baud rate if necessary)
    serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

    # Check if the serial port is open
    if serial_port.is_open:
        print("Serial port opened successfully.")
    else:
        print("Failed to open serial port.")
        return

    try:
        while True:
            # Send '1' (obstacle detected)
            serial_port.write('1'.encode())  # Encoding as byte string
            print("Sent: '1'")
            serial_port.flush()  # Ensure data is written out

            # Wait for 2 seconds
            time.sleep(2)

            # Send '0' (clear path)
            serial_port.write('0'.encode())  # Encoding as byte string
            print("Sent: '0'")
            serial_port.flush()  # Ensure data is written out

            # Wait for 2 seconds
            time.sleep(2)

            # Read data from the serial port (if available)
            if serial_port.in_waiting > 0:
                incoming_data = serial_port.read(serial_port.in_waiting).decode('utf-8')  # Read the data and decode it
                print(f"Received from ESP32: {incoming_data}")
            else:
                print("No data received from ESP32.")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the serial port when the program is done
        serial_port.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()

