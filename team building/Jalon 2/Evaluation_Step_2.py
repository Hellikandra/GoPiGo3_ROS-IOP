import numpy as np
from Step_2 import step2_evaluation
import time

ANSWERS_FILENAME = 'answers.npy'
MAIN_CAT = 'Tank'        #reference of the main tag for the true/false positive evaluation
#other tag is 'Baby'


#team name is the name for the saved model : use 'team_010' (for a model team_010.h5, for instance)
def script_step_2(team_name):

    # call the function to predic the labels for the images in dataset/test_set. A model can be load to spare time.
    start = time.time()
    predictions = step2_evaluation(team_name)
    stop = time.time()
 
    predictions = predictions[predictions[:,0].argsort()]
        
    answers = np.load(ANSWERS_FILENAME)
    answers = answers[answers[:,0].argsort()]

    # evaluate the error ratios
    results = evaluate(predictions,answers)

    if (results['Validity'] == 0):
        
        return -1
    
    results['Time'] = stop-start
    results['Score'] = results['Error']
    
    print(results)
    
    return

def evaluate(r, a):
 
    results = {}
    results['Validity'] = 1
    
    # Check that r and a have the same lengths
    if len(r) != len(a):
        print("Error: bad lengths: results " + str(len(r)) + ", answers " + str(len(a)))
        return -1
    
    # set the initial error number to maximum
    item_number = len(a)
    error = item_number
    error_FP = 0
    error_FN = 0
    
    print('initial error:'+str(error))
    
    # for every item in array a
    for i in range(0, item_number):
        
        # find the related items in array r
        nb = np.where(r[:,0] == a[i][0])
        
        if (len(nb[0]) != 1):
            # if there are not exactly one item found in array r, generate an error
            print("Error: index " + str(nb[0][0]) + " exists multiple times: "+ str(len(nb[0])))
            results['Validity'] = 0
            return results
            
        elif(str.lower(r[nb[0][0]][1]) == str.lower(a[i][1])):
            # else, if the only item found has the same tag than in array a, decrement the error counter
            error = error - 1
        else:
            #evaluate the false positive and the false negative ratios
            if (str.lower(r[nb[0][0]][1]) == str.lower(MAIN_CAT)):
                error_FP = error_FP + 1
            elif (str.lower(a[i][1]) == str.lower(MAIN_CAT)):
                error_FN = error_FN + 1
    
    results['Nbr'] = item_number
    results['Error'] = error*100/item_number
    results['Error_FN'] = error_FN*100/item_number
    results['Error_FP'] = error_FP*100/item_number
    
    return results
