import serial

def read_stored_value():
    # Configure the serial port
    serial_port = "/dev/tty.usbmodem11201"  # Adjust this to your serial port
    baud_rate = 57600
    ser = serial.Serial(serial_port, baud_rate)

    try:
        while True:
            data = ser.readline().decode('utf-8').strip()
            try:
                
                stored_value = int(data)
                #print(stored_value)
                return stored_value  # Return the stored value
            except ValueError:
                pass  # Ignore non-integer data
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
#for i in range(1,10):
#    read_stored_value()