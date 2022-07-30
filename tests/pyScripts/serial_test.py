import serial
from time import sleep
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)

ser = serial.Serial("/dev/serial0", 9600, timeout=None)

while True:
  received_data = ser.read()
  sleep(0.1)
  data_left = ser.inWaiting()
  received_data += ser.read(data_left)
  print(received_data)

