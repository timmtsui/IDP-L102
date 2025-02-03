from machine import Pin
from movement import forward

S1 = Pin(7, Pin.IN)
S2 = Pin(8, Pin.IN)
S3 = Pin(9, Pin.IN)
S4 = Pin(10, Pin.IN)

M1 = Pin(11, Pin.OUT)
M2 = Pin(12, Pin.OUT)


def Junction():
    junction_type = None
    if S1 == 0 and S4 == 0:
        # Not at junction
        junction_type = "line"
        return "Line"
    elif S1 == 1:
        if S4 == 1:
            # T junction
            junction_type = "T"
        else:
            # L junction
            junction_type = "L"
    else:
        # Right junction
        junction_type = "R"
    
    expected_junction = instruction_set[0]

    if junction_type == expected_junction:
        next_instruction = instruction_set[1]
        if next_instruction == "turn":
            turn()
        time.sleep(2)
    else:
        return "Lost"
