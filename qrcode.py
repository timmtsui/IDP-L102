import struct  # This module converts between Python values and C structs represented as Python bytes objects.
from time import sleep
import machine

# Red---3v3
# Black---ground
# Blue---sda
# Yellow---scl

# The code reader has the I2C ID of hex 0c, or decimal 12.
TINY_CODE_READER_I2C_ADDRESS = 0x0C
# How long to pause between sensor polls.
TINY_CODE_READER_DELAY = 0.05

TINY_CODE_READER_LENGTH_OFFSET = 0
TINY_CODE_READER_LENGTH_FORMAT = "H"
TINY_CODE_READER_MESSAGE_OFFSET = TINY_CODE_READER_LENGTH_OFFSET + struct.calcsize(TINY_CODE_READER_LENGTH_FORMAT)
TINY_CODE_READER_MESSAGE_SIZE = 254
TINY_CODE_READER_MESSAGE_FORMAT = "B" * TINY_CODE_READER_MESSAGE_SIZE
TINY_CODE_READER_I2C_FORMAT = TINY_CODE_READER_LENGTH_FORMAT + TINY_CODE_READER_MESSAGE_FORMAT
TINY_CODE_READER_I2C_BYTE_COUNT = struct.calcsize(TINY_CODE_READER_I2C_FORMAT)

# Set up for the Pico, pin numbers will vary according to your setup.
i2c = machine.I2C(1,
                  scl=machine.Pin(11),  # yellow
                  sda=machine.Pin(10),  # blue
                  freq=400000)

print(i2c.scan())

# Keep looping and reading the sensor results until we get a QR code

def scan():
    try:
        # Read data from the I2C device
        read_data = i2c.readfrom(TINY_CODE_READER_I2C_ADDRESS, TINY_CODE_READER_I2C_BYTE_COUNT)
        
        # Unpack the length of the message
        message_length, = struct.unpack_from(TINY_CODE_READER_LENGTH_FORMAT, read_data, TINY_CODE_READER_LENGTH_OFFSET)
        
        # Unpack the message bytes
        message_bytes = struct.unpack_from(TINY_CODE_READER_MESSAGE_FORMAT, read_data, TINY_CODE_READER_MESSAGE_OFFSET)
        
        if message_length == 0:
            # No data to process, continue to the next iteration
            return None
        
        # Try to decode the message as a UTF-8 string
        try:
            message_string = bytearray(message_bytes[0:message_length]).decode("utf-8")
            return message_string
        except Exception as e:
            return "Error: Couldn't decode as UTF-8:", e
    
    except Exception as e:
        return "Error: Error reading from I2C:", e


while True:
    sleep(TINY_CODE_READER_DELAY)
    
    print(scan())
