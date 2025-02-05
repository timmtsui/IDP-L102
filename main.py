from machine import Pin, PWM
from time import sleep

class Motor:
    def __init__(self, pin1, pin2):
        self.m1Dir = Pin(pin1, Pin.OUT) 
        self.pwm1 = PWM(Pin(pin2)) 
        self.pwm1.freq(1000)
        self.pwm1.duty_u16(0)
        
    def off(self):
        self.pwm1.duty_u16(0)
        
    def forward(self, speed):
        self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
        self.pwm1.duty_u16(int(65535*speed/100)) # speed range 0-100 motor 1
        
    def reverse(self, speed):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535*speed/100))
        
    def turn(self, angle):
        motor_left.off()
        motor_right.off()
        if angle<0:
            motor_left.reverse(100)
            motor_right.forward(100)
            sleep(angle*-0.009)
            motor_left.off()
            motor_right.off()
            
        if angle>0:
            motor_left.forward(100)
            motor_right.reverse(100)
            sleep(angle*0.009)
            motor_left.off()
            motor_right.off()
            
        
    
               
motor_left = Motor(7, 6)
motor_right = Motor(4, 5)



S1 = Pin(21, Pin.IN, Pin.PULL_DOWN)
S2 = Pin(20, Pin.IN, Pin.PULL_DOWN)
S3 = Pin(19, Pin.IN, Pin.PULL_DOWN)
S4 = Pin(18, Pin.IN, Pin.PULL_DOWN)

def LineFollow(): 
    # Will need to implement ghosting prevention
    if S2.value() == 0 and S3.value() == 0:
        motor_left.forward(50)
        motor_right.forward(50)
        print("Move forward")
    elif S3.value() == 1:
        motor_left.forward(50)
        motor_right.forward(0)
        print("Turn right")
    else:
        motor_left.forward(0)
        motor_right.forward(50)
        print("Turn left")
        
button = Pin(22, Pin.IN, Pin.PULL_DOWN)
on = 0
while True:
    sleep(0.1)
    #print("left: ", S1.value())
    #print("centre left: ", S2.value())
    #print("centre right: ", S3.value())
    #print("right: ", S4.value())
    if on == 0 and button.value() == 1:
        on = 1
    elif on == 1 and button.value() == 1:
        on = 0
        
    if on:
        LineFollow()
    if not on:
        motor_left.off()
        motor_right.off()
                                                                                                                              
