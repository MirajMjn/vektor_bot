import RPi.GPIO as GPIO

class DCMotor:
    def __init__(self, in1, in2, id = 0):
        self.in1 = in1
        self.in2 = in2
        self.id = id
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        self.pwm_in1 = GPIO.PWM(self.in1, 1000) # PWM frequency 1kHz
        self.pwm_in2 = GPIO.PWM(self.in2, 1000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)

    def stop(self):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(0)

    def forward(self, dty_cyc):
        self.pwm_in1.ChangeDutyCycle(dty_cyc)
        self.pwm_in2.ChangeDutyCycle(0)

    def backward(self, dty_cyc):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(dty_cyc)

    def destroy(self):
        self.pwm_in1.stop()
        self.pwm_in2.stop()
        # GPIO.cleanup() -> this is done at motor interface node.
        # print("GPIO cleaned up")