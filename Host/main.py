import serial
import time

from serial.serialutil import EIGHTBITS

"""
Packet Structure: [start_bit,  // BEGIN
                   control_code, 
                   data_length, 
                   data,  [] 
                   end_bit]
"""

    
SERIAL_PORT = "/dev/cu.usbmodem1301"
BAUD_RATE = 9600

############## Protocol Codes ############
ACK   = b'\x06'
END   = b'\x00'
START = b'\x01'
DEBUG = b'\x08'

############## Control Codes #############
SENDMOTOR_DATA   = b'\x32'
REQUEST_IMU_DATA = b'\x33'
REQUEST_RANGE_DATA = b'\x34'

############## State Codes ################
STATE_IR_SENSOR_TIMEOUT = b'\x40'


def startConnection(com):
    print("Connecting to device...")
    time.sleep(2) 
    com.write(ACK) # send handshake byte to arduino
    while(com.read() != ACK): # wait for arduino to acknowledge
        print(".", end="")
    print("Serial Handshake Successful")

def sendPacket(ser, control_code, data_length, data):
    ser.write(START)
    ser.write(control_code)
    ser.write(data_length)
    if data_length > 0: 
        for i in data:
            ser.write(i)
    ser.write(END)
    

if __name__ == "__main__":
    com = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, bytesize=EIGHTBITS)
    startConnection(com) 
    #com.write(DEBUG)
    sendPacket(com, REQUEST_RANGE_DATA, 4, [1,2,3,4])
    #while(True):
        #com.write(START)
        #sendPacket(com, REQUEST_RANGE_DATA, 4, [1,2,3,4])
        #com.write(SENDMOTOR_DATA)
        #if(com.read() == START):
    time.sleep(1)
    for i in range(com.in_waiting):
        print(com.read())        
         #   for i in range(com.in_waiting):
          #      print(com.read())