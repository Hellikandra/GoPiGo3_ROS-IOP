from tfModel import tfModel

#init Algo1:
model = tfModel()
sess = tf.session()
model.build(sess)

modelPath = "./modelSaved"
saver = tf.train.import_meta_graph(loadPath+'.meta')
saver.restore(sess, loadPath)

def load():
	pass

def algo_1(pathTrain, PathTest):
	images, ids = load()
	images = crop(images)
	images = model.preprocess(images)
	
	res = model.test(images, sess)
	resT = np.argmax(res, axis=1)
	
	sess.close()

def algo_2(pathTrain, PathTest):
	return np.array([[0]])

def algo_3(pathTrain, PathTest):
	return np.array([[0]])
	