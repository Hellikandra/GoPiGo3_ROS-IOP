# this code work without using ROS / IOP => need to be changed in the Pi and after, in the ROS C2
# https://superuser.com/questions/954665/how-to-redirect-route-ip-address-to-another-ip-address


import cv2
import urllib
import requests
import numpy as np

#stream = urllib.urlopen('http://localhost:8080/frame.mjpg')
stream = urllib.request.urlopen('http://192.168.0.43:8000/stream.mjpg')
bytes = ''
print("ready to read")
az = 0
while True:
    #bytes += stream.read(1024)
    bytes += stream.read(8192).decode('latin1')
    a = bytes.find('\xff\xd8')
    b = bytes.find('\xff\xd9')
    #print('az = ' + str(az) +' a = ' + str(a) + ' and b = ' + str(b))
    if a != -1 and b != -1:
        jpg = bytes[a:b+2]
        bytes = bytes[b+2:]
        # i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
        i = np.asarray(bytearray(jpg, encoding='latin1'), dtype="uint8")
        i = cv2.imdecode(i, cv2.IMREAD_COLOR)
        try:
            az += 1
            cv2.imshow('i', i) 
        except:
            print("cannot open imshow")
        if cv2.waitKey(10) == 27:
            exit(0)
        
