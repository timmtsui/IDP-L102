import network
import struct # This module converts between Python values and C structs represented as Python bytes objects.
from time import sleep
import machine
# NOTE INCORRECT CONNECTIONS WILL DESTROY THE SENSOR. CHECK WITH BENCH MULTIMETER BEFORE POWER/USE
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
i2c = machine.I2C(  1, 
                    scl=machine.Pin(19), # yellow
                    sda=machine.Pin(18), # blue
                    freq=400000)
print(i2c.scan())
# Keep looping and reading the sensor results until we get a QR code
while True:
    sleep(TINY_CODE_READER_DELAY)
    read_data = i2c.readfrom(TINY_CODE_READER_I2C_ADDRESS,
    TINY_CODE_READER_I2C_BYTE_COUNT)
    # print('raw data',read_data)
    message_length, = struct.unpack_from(TINY_CODE_READER_LENGTH_FORMAT, read_data, TINY_CODE_READER_LENGTH_OFFSET)
    message_bytes = struct.unpack_from(TINY_CODE_READER_MESSAGE_FORMAT, read_data, TINY_CODE_READER_MESSAGE_OFFSET)
    if message_length == 0:
    # print('nothing')
        continue
    try:
        message_string = bytearray(message_bytes[0:message_length]).decode("utf-8")
        print('barcode:', message_string)
    except:
        print("Couldn't decode as UTF 8")
    pass