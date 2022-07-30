from PIL import Image
import numpy as np
from os import listdir

import csv


def initDb(trainFold, metaTrain):
    with open(metaTrain, 'w') as metaFile:
        print("writer setting")
        metaFile.write(trainFold+ "\n")
        print("Writer ready")
        imList = listdir(trainFold)
        for el in imList:
            #print(trainFold + el)
            try:
                imArr = np.array(Image.open(trainFold + el))-255
                type=0;up=0;left=0;sizeX=0;sizeY=0;
                #print(str(el.lower())[:8])
                
                #print(imArr[0])
                
                
                true_points = np.argwhere(imArr)
                top_left = true_points.min(axis=0)
                #print(top_left)
                bottom_right = true_points.max(axis=0)
                #print(bottom_right)
                
                if (str(el.lower())[:8]=="triangle"):
                    type=1
                elif(str(el.lower())[:6]=="circle"):
                    type=2
                elif(str(el.lower())[:6]=="square"):
                    type=3
                elif(str(el.lower())[:5]=="other"):
                    type=4
                
                #print("g")
                up = top_left[0]
                #print("g0")
                left = top_left[1]
                #print("g1")
                sizeY = bottom_right[0] - up + 1
                #print("g2")
                sizeX =    bottom_right[1] - left + 1
                #print("g3")
                #print(el+","+str(type)+","+str(up)+","+str(left)+","+str(sizeY)+","+str(sizeX))
                metaFile.write(el+","+str(type)+","+str(up)+","+str(left)+","+str(sizeY)+","+str(sizeX)+"\n")
                #print("w")
            except:
                pass


def crop(images):
    im_arr = np.empty([images.shape[0],28,28,1])
    for i in range(images.shape[0]):
        true_points = np.argwhere(images[i])
        top_left = true_points.min(axis=0)
        bottom_right = true_points.max(axis=0)
        im_arr[i,:,:] = np.array(Image.open(str(path) + str(name)))[top_left[0]:\
            top_left[0]+bottom_right[0] - top_left[0] + 1,top_left[1]: \
            top_left[1] + bottom_right[1] - top_left[1] + 1]
    return np.array(im_arr)
       
#data: name.png class up left sizeY sizeX
def loadMetaDB(metaTrain, trainFold):
    images = []
    classes = []

    with open(metaTrain, "r") as metaData:
        data = csv.reader(metaData, delimiter=',', quotechar='|')
        path = next(data)[0]
        for el in data:
            name=str(el[0]);type=int(el[1]);X=int(el[2]);Y=int(el[3]);sx=int(el[4]);sy=int(el[5])
            images.append(np.array(Image.open(str(path) + str(name)))[X:X+sx,Y:Y+sy])
            classes.append(np.array(type))
    return images, classes
        
        
        
    
    
def loadImage():
    pass

