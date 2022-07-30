import tensorflow as tf

#perso
from tfModel import *
from Helper import *

from os import listdir
from PIL import Image
import matplotlib.pyplot as plt

import numpy as np

print("good")

from os.path import isdir,isfile



path = "D:/tmp/desktop_old/Desktop/ElectroGame/Jalon 1/Project_Folder/"
trainFold = path + "Training_Database/"
testFold = path + "Testing_Database/"
metaTrain = path + "metaDb.inf"


#paramDebug
initializedDB = True;
onlyTest = False
trained = True
fineTunning = True
printError = False


if(not(initializedDB)):
    initDb(trainFold, metaTrain)

im, data = loadMetaDB(metaTrain, trainFold)
im = np.array(im)
data = np.array(data)

print("ploting")
print(im.shape)
print(im[0].shape)
print(im[0][0:5,0:5])

#plt.imshow(im[-1])
#plt.show()

model = tfModel()
im = model.preprocess(im)


with tf.Session() as sess: 
    ###init
    model.build(sess)
    init = tf.global_variables_initializer()
    
    
    loadPath = "./modelSaved"
    saver = tf.train.Saver()


    if (trained):
        saver.restore(sess, loadPath)
    else:
        sess.run(init)
    print("val", np.random.choice(im.shape[0],32))
    print("im", np.expand_dims(im[np.random.choice(im.shape[0],32)],axis=-1).shape)
    print("lab", data[np.random.choice(data.shape[0], 32)]-1)
    #model.train(np.expand_dims(im[np.random.choice(im.shape[0],32)], \
    #    axis=-1), data[np.random.choice(data.shape[0], 32)]-1, sess)
        
    #training loop
    if(not(onlyTest)):
        for i in range(10000):
            selection = np.random.choice(im.shape[0],32)
            
            imBatch = im[selection]
            labBatch = data[selection]-1
            
            if fineTunning:
                for j,el in enumerate(imBatch):
                    if np.random.randint(2, size=1)==1:
                        imBatch[j] = np.flip(np.rot90(imBatch[j], \
                            k=np.random.randint(4, size=1)), axis=0)
                        #np.flip
            
            if (i%500==0):
                loss = model.writeSummary(np.expand_dims(imBatch, axis=-1), labBatch, sess, i)
            else:
                loss = model.train(np.expand_dims(imBatch,axis=-1), labBatch, sess)
            
                
            if (i%100==0):
                print(loss)
    
        saver.save(sess, loadPath)

    #testFold
    total = 0
    correct = 0
    errorIms = []
    error = []
    print("starting test")
    for i in range(50):
        selection = np.random.choice(im.shape[0],32)
        res = model.test(np.expand_dims(im[selection], axis=-1), data[selection]-1, sess)
        
        labelsT = data[selection]-1
        resT = np.argmax(res, axis=1)
        print(labelsT)
        print(resT)
        print(np.sum(labelsT == resT),"/",labelsT.shape[0])
        total += labelsT.shape[0]
        correct += np.sum(labelsT == resT)
        for h, id in enumerate(selection):
            if labelsT[h]!=resT[h]:
                errorIms.append(im[id])
                error.append([labelsT[h],resT[h]])
    
    if (printError):
        dictShapes={0:"triangle",1:"circle",2:"square",3:"other"}
        for k, el in enumerate(errorIms):
            plt.imshow(el)
            plt.title("found: "+dictShapes[error[k][1]]+"  real: "+dictShapes[error[k][0]])
            plt.show()
    print("result: " + str(float(correct)/float(total)*100) + "%")
    
    
    

#plt.imshow(im[-1])
#plt.show()


exit()
#old
#
#print(isdir(path))
#print(isdir(testFold))
#
#import random
#
#imList = listdir(trainFold)
#print(imList[0:5])
#
#
#print(random.choice(imList))
#im = Image.open(trainFold + random.choice(imList))
#plt.imshow(im)
#
##create model
#model = tfModel()
#
#
#
#for i in range(1000):
#    pass