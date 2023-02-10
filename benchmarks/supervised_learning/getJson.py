import json
import pandas as pd
import numpy as np
import settings

def getData():
    open("full_data.csv", "w").close()
    out = open("full_data.csv", "a")
    for iter in range(settings.demos):
        f = open(settings.folder + "data" + str(iter) + ".csv")
        Lines = f.readlines()
        for i in range(0, len(Lines)):
            Lines[i] = ''.join(Lines[i].split())
        if iter == 0:
            out.write(Lines[0] + "\n")
        for i in range(1, len(Lines)):
            out.write(Lines[i] + "\n")
        f.close()    
    out.close()
