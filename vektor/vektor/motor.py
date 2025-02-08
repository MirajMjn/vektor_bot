import RPi.GPIO as GPIO # https://pypi.org/project/RPi.GPIO/

class DCMotor:
    def __init__(self, in1, in2, name="motor_0"):
        self.in1 = in1
        self.in2 = in2
        self.name = name
        self.pid_controller = PIDController()

        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)

        self.pwm_in1 = GPIO.PWM(self.in1, 1000) # pwm frequency 2kHz on pin in1
        self.pwm_in2 = GPIO.PWM(self.in2, 1000)
        self.pwm_in1.start(0)
        self.pwm_in2.start(0)
        

    def stop(self):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(0)

    def fwd(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(pwm_value)
        self.pwm_in2.ChangeDutyCycle(0)

    def rev(self, pwm_value):
        self.pwm_in1.ChangeDutyCycle(0)
        self.pwm_in2.ChangeDutyCycle(pwm_value)
    
    def rotate(self, u_n, current_rpm):
        target_rpm = abs(u_n)
        if target_rpm < 10:
            self.stop()
        else:
            pwm = self.pid_controller.compute(target_rpm, current_rpm)
            if u_n < 0:
                self.rev(pwm)
            else:
                self.fwd(pwm)

    def destroy(self):
        self.stop()
        self.pwm_in1.stop()
        self.pwm_in2.stop()


class PIDController:
    def __init__(self, Kp=1.0, Ki=0.1, Kd=0.05, min_output=0, max_output=100):
        self.Kp = Kp
        #self.Ki = Ki
        #self.Kd = Kd
        self.integral = 0
        self.error  = 0
        # self.last_error = 0
        self.min_output = min_output
        self.max_output = max_output

        self.pid_pwm = 0

    def compute(self, target, current):
        self.error = target - current
        #self.integral += self.error
        # derivative = error - self.last_error
        #self.last_error = self.error
        output = (self.Kp * self.error) #+ (self.Ki * self.integral) #+(self.kd * derivative) 
        output = max(min(output, self.max_output), self.min_output)

        # TODO scale the PID output to pwm output ??

        return output