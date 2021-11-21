import serial
import time

"""
Packet Structure: [start_bit, 
                   control_code, 
                   data_length, 
                   data, 
                   end_bit]
"""

    
SERIAL_PORT = "/dev/cu.usbmodem2142201"
BAUD_RATE = 9600

MOTOR_DATA = b'\x00'
IMU_DATA   = b'\x01'
ACK        = b'\x02'


if __name__ == "__main__":
    com = serial.Serial(SERIAL_PORT, BAUD_RATE)
    time.sleep(3) 
    a = ""
    print("Connecting to device...")
    com.write(ACK)
    while(a != ACK):
        a = com.read()
        print(a)
    print("Connection Successful")

    while(True):
        s = com.read()
        print(s)