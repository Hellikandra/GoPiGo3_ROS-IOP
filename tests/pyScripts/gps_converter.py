import math 
def dm(x):
    print("int(x)       : ",int(x))
    print("int(x) / 100 : ", int(x) / 100)
    degrees = int(x) // 100
    print("degrees      : ", degrees)
    minutes = x - (100*degrees)
    print("minutes      : ", minutes)
    return degrees, minutes

def decimal_degrees(degrees, minutes):
    return degrees + minutes/60 

print ("Latitude  (°)   : ", decimal_degrees(*dm(5041.7932)))
print ("Longitude (°)   : ", decimal_degrees(*dm(00515.5229)))
print ("Longitude (rad) : ", (decimal_degrees(*dm(00515.5229))* math.pi/180))
print ("Longitude (rad) : ", (decimal_degrees(*dm(00515.5229))* math.pi/180))