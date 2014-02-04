import os
from week1 import square40

results = []

iterations = 10
f = open('experiment.tmp', 'w')
print "Please enter all displacement values in the format x y"
for i in range(1,iterations+1) :
    square40()
    prompt_str = "Enter test " + str(i) + " value: "
    results.append(raw_input(prompt_str))
for i in results : f.write(str(i) + '\n')
f.write('\n')
f.close()
os.system('cat experiment.tmp | ./covariance')
