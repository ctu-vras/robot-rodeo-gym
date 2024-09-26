#!/usr/bin/env python


from __future__ import print_function
from lxml import etree
import math
import argparse
import sys

if __name__ == '__main__':
    file_in = sys.argv[1]
    file_out = sys.argv[2]
    old = sys.argv[3]
    new = sys.argv[4]

    #print("Replace models path")
    #print("Input file: " + file_in)
    #print("Output file: " + file_in)
    #print("Old string: " + old)
    #print("New string: " + new)

    with open(file_in, 'r') as file:
      filedata = file.read()

    # Replace the target string
    filedata = filedata.replace( old, new)

    # Write the file out again
    with open(file_out, 'w') as file:
      file.write(filedata)
