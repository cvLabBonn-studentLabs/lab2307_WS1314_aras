#!/bin/env python
import sys
import getopt
import random
import os
from PIL import Image

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    filename = sys.argv[1]
    output_directory = sys.argv[2]
    training_fraction = float(sys.argv[3])
    test_fraction = float(sys.argv[4])
    
    if training_fraction + test_fraction > 1.:
        print "Test + training fraction > 100%"
        return 1
    else:
        print "Cross Validation fraction: %f" % (1. - training_fraction - test_fraction)
        
    training_data = ""
    test_data = ""
    cv_data = ""
    input_file = open(filename)
    random.seed()
    for line in input_file:
        sample = random.random()
        if sample < training_fraction:
            training_data += "%s" % line
        elif sample < training_fraction + test_fraction:
            test_data += "%s" % line
        else:
            cv_data += "%s" % line
    
    training_file = open(os.path.join(output_directory, "data_training.txt"), 'w')
    training_file.write(training_data)
    training_file.close()
    
    test_file = open(os.path.join(output_directory, "data_test.txt"), 'w')
    test_file.write(test_data)
    test_file.close()
    
    cv_file = open(os.path.join(output_directory, "data_cv.txt"), 'w')
    cv_file.write(cv_data)
    cv_file.close()
    
if __name__ == "__main__":
    sys.exit(main())