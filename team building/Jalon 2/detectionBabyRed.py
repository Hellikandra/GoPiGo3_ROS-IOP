import os
import os.path
from os import walk
from PIL import Image

def compute_average_image_color(img):
    width, height = img.size

    r_total = 0
    g_total = 0
    b_total = 0

    count = 0
    for x in range(0, width):
        for y in range(0, height):
            colors = img.getpixel((x,y))
            r_total += colors[0]
            g_total += colors[1]
            b_total += colors[2]
            count += 1

    return (r_total/count, g_total/count, b_total/count)


#path = r"C:\Users\<Username>\Downloads\training_set"
path = r".\dataset\test_set"

listeFichiers = []
for (repertoire, sousRepertoires, fichiers) in walk(path):
    for filename in fichiers:
        print(filename)
        img = Image.open("dataset\\test_set\\" + filename)
        r, g, b = compute_average_image_color(img)
        if(r > g and r > 110):
            print ("Dont kill")
        else:
            print("KILL")

