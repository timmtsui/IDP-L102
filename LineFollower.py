from machine import pin
from time import sleep
import movement

S1 = Pin(21, Pin.IN)
S2 = Pin(20, Pin.IN)
S3 = Pin(19, Pin.IN)
S4 = Pin(18, Pin.IN)

# 0 is black, 1 is white
def LineFollow(): 
    # Will need to implement ghosting prevention
    if S2 and S3 == 0:
        movement.forward()
    elif S3 == 1:
        movement.rightcorrect()
    else: 
        movement.leftcorrect()
    



