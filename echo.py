import serial
import time
if __name__ == "__main__":
    ser = serial.Serial("COM4", 9600, timeout=0.005)
    while True:
        if(ser.isOpen()):
            user_input = input("Enter key: ")   #asking user for the character to be sent
            ########## ENTER YOUR CODE HERE ############
            ser.write(user_input.encode())      #sending the character
            time.sleep(1)                       #letting the atmega recieve the character and send it back

            rec = ser.read(13)                  #recieving upto 13 characters sent by atmega  
            print(rec)                          #printing the recieved string



        ############################################

