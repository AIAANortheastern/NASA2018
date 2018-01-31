import serial
import time
import keyboard

ser = serial.Serial('COM4', 9600, timeout=0)  # open serial port
print(ser.name)         # check which port was really used
ser.write(b'hello')  # write a string

timeCode = time.strftime("%Y_%m_%d_%H_%M") # gets a time code to use for the file name

fileName = timeCode + ".txt" # adds txt extention to the file name

f = open(fileName, 'w') # opens the text file

# Loop runs until a key is pressed
while True:
        x = ser.read(32) # Reads 32 bits from the xbee buffer
        string = x.decode(encoding='UTF-8') # decodes the bytes from the xbee
        f.write(string) # writes the decoded info to a text file
        print(string) # prints the same info to the terminal
        time.sleep(0.001) # waits for a small fraction of a second
        if keyboard.is_pressed('l'): # when the 'l' key is pressed it writes a marker to the text file
            f.write("StuffStuffStuff -----------------------------------z")
        if keyboard.is_pressed('q'): # exits the while loop
            break

q = 0
# this sends a '?' to the other xbee and was used for testing nasa's payload
while q < 10:
    ser.write(b'?')
    q += 1

#closes the stuff I opened earlier
f.close()
ser.close()