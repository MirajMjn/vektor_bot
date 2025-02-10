import struct
import machine
import time

# Initialize UART0 with an appropriate baud rate and pins (adjust as necessary)
uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))

def send_floats(val1, val2, val3):
    # Pack three 32-bit floats into binary format.
    # '<fff' means little-endian (adjust if needed) and three floats.
    data = struct.pack('<fff', val1, val2, val3)
    uart.write(data)

# Example: continuously send three float values
while True:
    # Example float values (adjust these as needed)
    x, y, z = 1.23, 4.56, 7.89
    send_floats(x, y, z)
    time.sleep(0.1)
