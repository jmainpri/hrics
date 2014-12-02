import sys
import glob
import os

base_dir = '/home/rafi/aterm_experiment/'
blocks = sorted([ name for name in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, name)) ])
for block in blocks:
    path = os.path.join(base_dir, block)
    runs = sorted([ name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name)) ])

    for run in runs:
        file_path = os.path.join(path, run)
        out_path = os.path.join(file_path+'/images/')
        print
        print 'Calculating features for block : ', block, ' run : ', run
        print 'in folder : ', file_path
        print'================================================'
        print 'Outpath : ', out_path

        # Make output directory
        if not os.path.exists(out_path):
            os.makedirs(out_path)

        # Move every png
        for file in glob.glob(os.path.join(file_path+'/*.png')):
            os.rename(file, out_path+os.path.basename(file))