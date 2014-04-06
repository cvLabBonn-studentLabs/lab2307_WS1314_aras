#!/bin/env python
import sys
import getopt
from PIL import Image

class Usage(Exception):
    def __init__(self, msg):
        self.msg = msg

def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    filename = sys.argv[1]
    size_string = sys.argv[2]
    norm_z = int(sys.argv[3])
    width, height = map(lambda x: int(x), size_string.split('x'))
    print "Width:Height: %d:%d" % (width, height)
    file = open(filename, 'r')
    
    im = Image.new('L', (width, height))
    for line in file:
        
        data = map(lambda x: int(round(float(x.split(":")[1])*norm_z)), line.split(" ")[2:])
        print "Num Pixels: %d" % len(data)
        im.putdata(data)
        im.show()
        cls = line.split(" ")[0]
        print "Class %s. Press any key to continue..." % cls
        raw_input()
    
    
if __name__ == "__main__":
    sys.exit(main())