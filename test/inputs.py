import RPi.GPIO as GPIO
from time import sleep

PINNUM = 8

GPIO.setmode(GPIO.BCM)
GPIO.setup(PINNUM, GPIO.IN)

print("Pin is ready")

pinRec = False
while True:
    readVal = GPIO.input(PINNUM) 
    
    if pinRec and not readVal: 
        pinRec = False
        print("Input deactivated", readVal)
    elif not pinRec and readVal:
        pinRec = True
        print("Input activated", readVal)
