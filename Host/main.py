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

SENDMOTOR_DATA   = b'\x00'
REQUEST_IMU_DATA = b'\x01'
ACK              = b'\x02'


if __name__ == "__main__":
    com = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print("Connecting to device...")
    time.sleep(2) 
    com.write(ACK)
    while(com.read() != ACK):
        print(a)
    print("Connection Successful")

    while(True):
        
        com.write(b'50')
        #while(com.read() != ACK):
        #    print("waiting")
        #time.sleep(.1)

        
