from machine import pin
from time import sleep
import movement

S1 = Pin(7, Pin.IN)
S2 = Pin(8, Pin.IN)
S3 = Pin(9, Pin.IN)
S4 = Pin(10, Pin.IN)

# 0 is black, 1 is white
def LineFollow(): 
    # Will need to implement ghosting prevention
    if S2 and S3 == 0:
        movement.forward()
    elif S3 == 1:
        movement.rightcorrect()
    else: 
        movement.leftcorrect()
    


