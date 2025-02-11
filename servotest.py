from machine import Pin, PWM
from time import sleep
# Set up PWM Pin for servo control
servo_pin = Pin(15)
servo = PWM(servo_pin)
# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
servo_horizontal = 4833
servo_carry = 3300
servo_highest = 2800
half_duty = int(max_duty/2)
#Set PWM frequency
frequency = 50
servo.freq(frequency)

try:
    while True:
        servo.duty_u16(3300)
        sleep(1)
        """servo.duty_u16(3100)
        sleep(1)"""
except KeyboardInterrupt:
    print("Keyboard interrupt")
    # Turn off PWM
    servo.deinit()