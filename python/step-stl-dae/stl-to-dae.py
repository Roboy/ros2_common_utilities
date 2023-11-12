#!/usr/bin/env python
import sys
from subprocess import call
import time
from os import listdir, rename
from os.path import isfile, join, splitext

argv = sys.argv

print(argv)

if len(argv)!=3:
    print('USAGE: path/to/stl/directory/ outputpath/directory/')
else:
    t_start = time.time()
    files = [f for f in listdir(argv[1]) ]
    print('Converting '+ str(len(files)) +' files')
    i=0
    for file in files:
        if file.lower().endswith('.stl'):
        # using meshlab to convert stl files to dae
            pre, ext = splitext(file)
            print file
            file_dae = pre + ".dae"
            call(["meshlabserver", "-i" , join(argv[1],file) , "-o" , join(argv[2],file_dae)])

    t_end = time.time()

    print " "
    print "Started at ", time.ctime(t_start)
    print "Finished at ", time.ctime(t_end)
    print "Time elapsed: ", (t_end - t_start) / 60, " minutes"