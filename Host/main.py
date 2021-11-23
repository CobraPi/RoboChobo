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

    
SERIAL_PORT = "/dev/cu.usbmodem2142201"
BAUD_RATE = 9600

ACK   = b'\x06'
END   = b'\x00'
BEGIN = b'\x01'
DEBUG = b'\x08'

SENDMOTOR_DATA   = b'\x32'
REQUEST_IMU_DATA = b'\x33'



def startConnection(com):
    print("Connecting to device...")
    time.sleep(2) 
    com.write(ACK) # send handshake byte to arduino
    while(com.read() != ACK):
        print(".", end="")
    print("Serial Handshake Successful")

def sendPacket(ser, control_code, data_length, data):
    ser.write(BEGIN)
    ser.write(control_code)
    ser.write(data_length)
    if data_length > 0: 
        for i in data:
            ser.write(i)
        ser.write(END)
    

if __name__ == "__main__":
    com = serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, bytesize=EIGHTBITS)
    startConnection(com) 
    com.write(DEBUG)
    
    while(True):
        sendPacket(com, DEBUG, 3, [1,2,3])
        print(com.read_until(END))