
"""
* Team Id : eYRC#972
* Author List : Shivam Grover, Harshita Diddee, Shivani Jindal, Anuj Trehan
* Filename: echo.py
* Theme: Thirsty Crow (TC)(eYRC)
"""

import serial
import time
if __name__ == "__main__":
    ser = serial.Serial("COM4", 9600, timeout=0.005)
    while True:
        if(ser.isOpen()):                       #Checking if input port is open ie capable of communication
            user_input = input("Enter key: ")   #Taking User input            
            ser.write(user_input.encode())      #Writing binary data onto the serial port 
            time.sleep(1)                       #letting the atmega recieve the character and send it back

            rec = ser.read(13)                  #recieving upto 13 characters sent by atmega  
            print(rec)                          #printing the recieved string



        
