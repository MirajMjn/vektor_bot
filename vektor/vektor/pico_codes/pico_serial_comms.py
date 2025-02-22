import ustruct
from time import sleep
from machine import UART
from LM393Tachometr_PIO import Tachometer
from KY_040_renc_ruler import RotaryEncoder

# uart = machine.UART(0, baudrate=115200, tx=machine.Pin(0), rx=machine.Pin(1))
uart = UART(0, baudrate=115200)  # UART is used by the USB serial

taco1 = Tachometer(16, 0)  # GP16
taco2 = Tachometer(17, 1)  # GP17
taco3 = Tachometer(18, 2)  # GP18

rencl = RotaryEncoder(8, 9)
rencr = RotaryEncoder(10, 11)
rencb = RotaryEncoder(14, 15)

try:
    while True:
        try:
            # Pack three 32-bit floats into binary format.
            # '<fff' means little-endian (adjust if needed) and three floats.
            data = ustruct.pack(
                '<ffffff', 
                taco1.get_rpm(), 
                taco2.get_rpm(), 
                taco3.get_rpm(), 
                rencl.get_distance(), 
                rencr.get_distance(), 
                rencb.get_distance()
                )
            uart.write(data)
            # print(f"sent{data}")
        except Exception as e:
            print(f"[ERROR] : {e} , no data sent ... PASSING")
            pass
        sleep(0.4)
        
except Exception as e:
    print(f"[Exception] : {e} Occured ... Terminating")
finally:
    # Cleanup resources
    uart.deinit()
    print("Resources have been deinitialized.")