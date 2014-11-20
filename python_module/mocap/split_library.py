import SegmentCSV
import MocapCommon
import os

base_dir = '/home/rafi/aterm_experiment'

blocks = sorted([ name for name in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, name)) ])
for block in blocks:
    path = os.path.join(base_dir, block)
    runs = sorted([ name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name)) ])

    for run in runs:
        file_path = os.path.join(path, run)
        out_path = os.path.join('/home/rafi/Desktop/experiment_segments',block,run)
        print
        print 'Segmenting block : ', block, ' run : ', run
        print 'in folder : ', file_path
        print'================================================'
        print 'Outpath : ', out_path

        if not os.path.exists(out_path):
            os.makedirs(out_path)

        if os.path.isfile(os.path.join(file_path, 'Splits.csv')) :
            splits = MocapCommon.read_splits(file_path+'/')
            m_file = os.path.join(file_path,'markers_fixed.csv')
            o_file = os.path.join(file_path,'objects_fixed.csv')

            s = SegmentCSV.Segmenter(m_file, os.path.join(out_path,'markers.csv'))
            s.segment(splits)

            s = SegmentCSV.Segmenter(o_file, os.path.join(out_path,'objects.csv'))
            s.segment(splits)
