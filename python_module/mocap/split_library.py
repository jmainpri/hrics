import SegmentCSV
import MocapCommon
import os

# source_dir = '/home/rafi/Desktop/aterm_experiment'
# target_dir = '/home/rafi/Desktop/experiment_segments'


def split_library(source_dir, target_dir):

    blocks = sorted([name for name in os.listdir(source_dir) if os.path.isdir(os.path.join(base_dir, name))])
    for block in blocks:
        path = os.path.join(source_dir, block)
        runs = sorted([name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name))])

        for run in runs:
            file_path = os.path.join(path, run)
            out_path = os.path.join(target_dir, block, run)
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

if __name__ == "__main__":

    if len(sys.argv) >= 4:
        for index in range(1, len(sys.argv)):
            if sys.argv[index] == "-s" and index+1 < len(sys.argv):
                base_dir = str(sys.argv[index+1])
            if sys.argv[index] == "-t" and index+1 < len(sys.argv):
                base_dir = str(sys.argv[index+1])

        split_library(base_dir)
    else:
        print "USAGE : -s PATH_TO_SOURCE_LIBRARY -t PATH_TO_TARGET_LIBRARY"