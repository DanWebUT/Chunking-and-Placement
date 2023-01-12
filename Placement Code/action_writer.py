import os
import time
import numpy as np
"""
This file is to write the actions the robots take in order
"""
time_setting = int(np.round((time.time())/1000))

folder = "GA" + str(time_setting)

current_path = os.getcwd()
filepath = os.path.join(current_path,folder)
os.makedirs(filepath, exist_ok=True)

filename = "R" +str(0) +"actions"
complete_filename = os.path.join(filepath, filename +".txt")


#create empty file
with open(complete_filename, "w") as file:
    file.write("")

#append to existing file
with open(complete_filename, "a+") as file:
   file.write("subtract")
