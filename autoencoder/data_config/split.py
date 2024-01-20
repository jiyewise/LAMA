from cgi import test
from fileinput import filename
from gc import collect
import random
import numpy as np
import sys, os
from IPython import embed

def collect_filenames(_root_path):
    dirnames = [(f.path, f.name) for f in os.scandir(_root_path) if f.is_dir()]
    filenames = []
    for dir in dirnames:
        if "preprocess" in dir:
            continue
        files_per_folder = [dir[1] + "/" + files for files in os.listdir(dir[0]) if files.endswith('bvh')]
        filenames.append(files_per_folder)
    return filenames

def split_filenames_and_write(_filenames):
    train_filenames = []
    test_filenames = []
    validation_filenames = []    

    for file_per_folder in _filenames:
        random.shuffle(file_per_folder)
        num_files = len(file_per_folder)

        # get idx of training, test, validation in 7:2:1 ratio
        train_end = int(num_files*0.7)
        test_end = int(num_files*0.9)
        
        train_filenames += file_per_folder[0:train_end]
        test_filenames += file_per_folder[train_end:test_end]
        validation_filenames += file_per_folder[test_end:]

    # write
    fnames_list = ['train', 'test', 'validation']
    # fnames_list = ['custom']
    for fnames in fnames_list:
        file = open(f"{fnames}_fnames.txt", 'w')
        file.write("\n".join(locals()[f"{fnames}_filenames"]))
        file.close()

    # training_file = open("./training_fnames.txt", 'w')
    # test_file = open("./test_fnames.txt", 'w')
    # validation_file = open("./validation_fnames.txt", 'w')
    
    # training_file.write("\n".join(train_filenames))
    # test_file.write("\n".join(test_filenames))
    # validation_file.write("\n".join(validation_filenames))

    # training_file.close()
    # test_file.close()
    # validation_file.close()

if __name__ == "__main__":
    filelist = collect_filenames("../data/")
    split_filenames_and_write(filelist)