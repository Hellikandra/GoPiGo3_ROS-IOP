import numpy as np
from Step_1 import algo_1, algo_2, algo_3
import time

TRAINING_DB = '/Training_Database' # folder containing the training png
TESTING_DB = '/Testing_Database' # folder containing the evaluation png
ANSWERS_FILENAME = 'answers.npy' # file containing the test tags
RESULTS_FILENAME = 'scores.npy' # file where the prediction outputs will be dropped
ALGO = 1 # algorithm number (1 to 3) to be run

def script_step_1():
    
    # TRAINING_DB is a folder containing the training png in the working repository
    # TESTING_DB is a folder containing the evaluation png in the working repository
    # results_1 is built as below :
    #  results_1 = numpy.array([
    #  ['IDnumber','tag'],
    #  ['IDnumber','tag'],
    #  ...
    #  ])
    
    # where :
    #  'IDnumber' is a str(int) from the image name.
    #  'tag' is one of these strings :
    #     'circle'
    #     'square'
    #     'triangle'
    #     'other'
    
    # training png are given with this file
    # 'answer.npy' and the evaluation png ('png_IDnumber.png') won't be sent before the final evalutation.
    
    # /!\ Training results can be stored in a npy file and reloaded during the evaluation process to spare time.
    
    results = []
    
    comp_ans = np.array([100.0,0.0,0])
    # comparison matrix :
    # [error_%, execution_time, algo_ID]

    # process algo_1 (generate predictions + processing time)
    start = time.time()
    if (ALGO==1):
        results = algo_1(TRAINING_DB, TESTING_DB)
    elif (ALGO==2):
        results = algo_2(TRAINING_DB, TESTING_DB)
    elif (ALGO==3):
        results = algo_3(TRAINING_DB, TESTING_DB)
    else:
        print('ERROR: Set ALGO between 1 and 3')
        return
    stop = time.time()
    comp_ans[1] = stop-start
    results = results[results[:,0].argsort()]
    np.save('results', results)
    print('Results:'+str(results))
        
    # load "answers.npy" file with true labels for the evaluation file. 
    answers = np.load(ANSWERS_FILENAME)
    print('Answers:'+str(answers))
    
    # Evaluate the error ratio for each algo
    comp_ans[0] = evaluate(results,answers)
    comp_ans[2] = ALGO
    
    print('Error (%)   Time (s)       Algorithm')
    print(comp_ans)

    f = open(RESULTS_FILENAME,'a+')
    f.write(str(comp_ans)+'\n') #a améliorer
    f.close()

    return




def evaluate(r, a):
    
    # Check that r and a have the same lengths
    if len(r) != len(a):
        print("Error: bad lengths: results " + str(len(r)) + ", answers " + str(len(a)))
        return -1
    
    # set the initial error number to maximum
    error = len(a)
    print('initial error:'+str(error))
    
    # for every item in array a
    for i in range(0, len(a)):
        
        # find the related items in array r
        nb = np.where(r[:,0] == a[i][0])
        
        if (len(nb[0]) != 1):
            # if there are not exactly one item found in array r, generate an error
            print("Error: index " + str(nb[0][0]) + " exists multiple times: "+ str(len(nb[0])))
            return -1
            
        elif(str.lower(r[nb[0][0]][1]) == str.lower(a[i][1])):
            # else, if the only item found has the same tag than in array a, decrement the error counter
            error = error - 1
#        else:
#            print("Bad prediction at index "+str(nb[0][0]))
            
    # return the error rate
    return error/len(a)*100
