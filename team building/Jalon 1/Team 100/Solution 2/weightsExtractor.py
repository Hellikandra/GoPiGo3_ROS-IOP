import tensorflow as tf
import numpy as np

numpyPath = "./numpyModel/"

print("ready to go")
with tf.Session() as sess:
    loadPath = "./modelSaved"
    
    print("restoring graph")
    saver = tf.train.import_meta_graph(
        loadPath+'.meta')
    print("restoring session")
    saver.restore(sess, loadPath)
    
    
    varList = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES)
    print(varList)
    for el in varList:
        print(el)
        print(el.name)
        arr = sess.run(el)
        np.save(numpyPath+el.name+".npy",arr)
        print(arr.shape)