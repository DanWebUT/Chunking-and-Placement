"""
for creating dependencies for many, indentical rectangular chunks
"""
import numpy as np

num_jobs = 2
num_chunks_in_row = 2
num_chunks_in_column = 3

num_chunks = num_jobs*num_chunks_in_column*num_chunks_in_row

chunk_dependencies = [[]]*num_chunks
chunk_job = [[]]*num_chunks

for chunk in range(0,num_chunks):
    if num_chunks_in_row == 2:
        if chunk < num_jobs*num_chunks_in_row:
            if chunk%2 != 0:
                chunk_dependencies[chunk] = [chunk-1]
        else:
            if chunk%2 == 0:
                chunk_dependencies[chunk] = [chunk-(num_jobs*num_chunks_in_row)]
            else:
                chunk_dependencies[chunk] = [chunk-(num_jobs*num_chunks_in_row),chunk-1]
    chunk_job[chunk] = [int(np.floor((chunk%(num_jobs*num_chunks_in_row))/num_chunks_in_row))]
    
    
        

