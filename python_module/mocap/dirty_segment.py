import SegmentCSV


# splits = [ (0, 1700), (1750, 3200), (3250, 5000), (5050, 6100), (6150, 7700), (7750, 8900), (8950, 10400), (10450, 11600), ]

# splits = [(1000,3900), (5900,9000), (11700,14800), (17700,20500), (22400,25300), (28300,30800), (33000,35700), (37900,40400), (42600,44700), ]

# splits = [ (580, 680), (760, 900), (1060, 1180), (1300, 1420), (2160, 2280) ]

# 0446, 0578), (0780,0871), (2554, 2671)


# splits = [(1, 3406), (8657, 11482), (13712, 17083), (19458, 22078), (24495, 27070)]

splits = [ (800,3511) ]

# s = SegmentCSV.Segmenter('/home/rafi/logging_ten/1/markers.csv', './markers.csv')
# s.segment(splits)

# s = SegmentCSV.Segmenter('/home/rafi/logging_ten/1/objects.csv', './objects.csv')
# s.segment(splits)

s = SegmentCSV.Segmenter('/home/rafi/two_arm_test_data/markers_fixed.csv', './markers.csv')
s.segment(splits)

s = SegmentCSV.Segmenter('/home/rafi/two_arm_test_data/objects_fixed.csv', './objects.csv')
s.segment(splits)