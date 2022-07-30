#model
import tensorflow as tf
import numpy as np

from skimage.transform import resize
from skimage.filters import gaussian




class tfModel:
    def __init__(self,):
        self.imSize = 28
        self.batchS = 32
        self.trainingRate = 0.006
        pass
        #tf.placeholder()
        
    
    def build(self, session):
        print("building")
        self.input_layer = tf.placeholder(tf.float32, shape=(None,self.imSize, self.imSize,1))
        #tf.reshape(features["x"], [-1, self.imSize, self.imSize, 1])
        print(self.input_layer)
        # Convolutional Layer #1
        self.conv1 = tf.layers.conv2d(
            inputs=self.input_layer,
            filters=16,
            kernel_size=[5, 5],
            padding="same",
            activation=tf.nn.relu)
        tf.summary.image("conv1", self.conv1[:,:,:,0:3])
        print("c1",self.conv1)
        
        # Pooling Layer #1
        self.pool1 = tf.layers.max_pooling2d(inputs=self.conv1, pool_size=[2, 2], strides=2)
        print(self.pool1)
        # Convolutional Layer #2 and Pooling Layer #2
        conv2 = tf.layers.conv2d(
            inputs=self.pool1,
            filters=32,
            kernel_size=[3, 3],
            padding="same",
            activation=tf.nn.relu)
        tf.summary.image("conv2",conv2[:,:,:,0:3])
        print("c2",conv2)
        self.pool2 = tf.layers.max_pooling2d(inputs=conv2, pool_size=[2, 2], strides=2)
        print("p2",self.pool2)
        # Dense Layer
        pool2_flat = tf.reshape(self.pool2, [-1, 7 * 7 * 32])
        print("p2f",pool2_flat)
        denseL = tf.layers.dense(inputs=pool2_flat, units=128, activation=tf.nn.relu)
        
        

        # Logits Layer
        self.logits = tf.layers.dense(inputs=denseL, units=4)
        print(self.logits)
        self.softLogits = tf.nn.softmax(self.logits)
        tf.summary.histogram("softRes", self.softLogits)
        print(self.softLogits)
        
        self.labels = tf.placeholder(tf.int32, shape=(None))
        self.loss = tf.losses.sparse_softmax_cross_entropy(labels=self.labels, logits=self.softLogits)
        tf.summary.scalar("loss", self.loss )
        
        #optimizer
        optimizer = tf.train.GradientDescentOptimizer(learning_rate=self.trainingRate) 
        self.train_op = optimizer.minimize( \
            loss=self.loss, \
            global_step=tf.train.get_global_step())
        
        #summary
        self.merged = tf.summary.merge_all()
        self.train_writer = tf.summary.FileWriter('tensorboard', session.graph)

    def train(self, inputIms, labelClasses, sess):
        feed_dict = {self.input_layer : inputIms, \
                    self.labels : labelClasses}
        loss_, optimizer_ = sess.run((self.loss, self.train_op), feed_dict=feed_dict)
        return loss_
    
    def writeSummary(self, inputIms, labelClasses, sess, i):
        feed_dict = {self.input_layer : inputIms, \
                    self.labels : labelClasses}
        loss_, optimizer_, summar = sess.run((self.loss, self.train_op, self.merged), feed_dict=feed_dict)
        self.train_writer.add_summary(summar, i)
        return loss_
    def test(self, inputIms, sess):
        feed_dict = {self.input_layer : inputIms}
        logit_ = sess.run(self.logits, feed_dict=feed_dict)

        return logit_
    def testPerso(self, inputIms, sess):
        feed_dict = {self.input_layer : inputIms}
        logit_ = sess.run(self.conv1, feed_dict=feed_dict)

        return logit_
    
    def preprocess(self, ims):
        resIms = np.empty([ims.shape[0],self.imSize,self.imSize])
        for i,im in enumerate(ims):
            resIms[i] = resize(im, (self.imSize, self.imSize), anti_aliasing=True)
            resIms[i] = gaussian(resIms[i], sigma=0.4)
        return resIms
        
