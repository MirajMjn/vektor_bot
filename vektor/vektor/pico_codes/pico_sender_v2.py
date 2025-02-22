from machine import Pin, Timer, UART
from LM393Tachometr_PIO import Tachometer
from KY_040_renc_ruler import RotaryEncoder

uart = UART(0, baudrate=115200) #UART setup

taco1 = Tachometer(16, 0)  
taco2 = Tachometer(17, 1)  
taco3 = Tachometer(18, 2)  

renc1 = RotaryEncoder(8, 9)
renc2 = RotaryEncoder(10, 11)
renc3 = RotaryEncoder(14, 15)

def send_data(timer):
    data = f"{taco1.get_rpm()},{taco2.get_rpm()},{taco3.get_rpm()},{renc1.get_distance()},{renc2.get_distance()},{renc3.get_distance()}\n"
    uart.write(data)
    print(f"sent{data}")

uart_timer = Timer()

uart_timer.init(period=333, mode=Timer.PERIODIC, callback=send_data)

print("Code running...")
