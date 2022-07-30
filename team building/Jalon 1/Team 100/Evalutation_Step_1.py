import numpy as np
from Step_1 import algo_1, algo_2, algo_3
import time

def script_step_1():
    
    # '/Training_Database' is a folder containing the training png in the working repository
    # '/Testing_Database' is a folder containing the evaluation png in the working repository
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
    #     'Other'
    
    # training png are given with this file
    # 'answer.npy' and the evaluation png ('png_IDnumber') won't be sent before the final evalutation.
    
    # /!\ Training results can be stored in a npy file and reloaded during the evaluation process to spare time.
    
    comp_ans = np.array([[100.0,0.0,1],[100.0,0.0,2],[100.0,0.0,3]])
    # comparison matrix :
    # [[error_%, execution_time, algo_ID]...]

    # process algo_1 (generate predictions + processing time)
    start = time.time()
    results_1 = algo_1('/Training_Database','/Testing_Database')
    stop = time.time()
    comp_ans[0][1] = stop-start
    results_1 = results_1[results_1[:,0].argsort()]
    
    # process algo_2 (generate predictions + processing time)
    start = time.time()
    results_2 = algo_2('/Training_Database','/Testing_Database')
    stop = time.time()
    comp_ans[1][1] = stop-start
    results_2[results_2[:,0].argsort()]
    
    # process algo_3 (generate predictions + processing time)
    start = time.time()
    results_3 = algo_3('/Training_Database','/Testing_Database')
    stop = time.time()
    comp_ans[2][1] = stop-start
    results_3[results_3[:,0].argsort()]
    
    # load "answers.npy" file with true labels for the evaluation file. 
    answers = np.load('answers.npy')
    
    #evaluate the error ratio for each algo
    comp_ans[0][0] = evaluate(results_1,answers)
    comp_ans[1][0] = evaluate(results_2,answers) 
    comp_ans[2][0] = evaluate(results_3,answers)  
 
    #sort the algo by error ratio
    comp_ans = comp_ans[comp_ans[:,0].argsort()]
    
    return comp_ans

def evaluate(array_1, array_2):
    
    if len(array_1) != len(array_2):
        
        return 101.0
    
    error = 0
    
    for i in range(0, len(array_1)):
        for j in range (0, len(array_2)) :
            
            if (array_1[i][1] != array_2[j][1]) and (array_1[i][0] == array_2[j][0]):
                error = error+1
                break               
            
    return error/len(array_1)*100