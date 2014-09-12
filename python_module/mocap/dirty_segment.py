import SegmentCSV


# splits = [ (0, 1700), (1750, 3200), (3250, 5000), (5050, 6100), (6150, 7700), (7750, 8900), (8950, 10400), (10450, 11600), ]

# splits = [(1000,3900), (5900,9000), (11700,14800), (17700,20500), (22400,25300), (28300,30800), (33000,35700), (37900,40400), (42600,44700), ]

# splits = [ (580, 680), (760, 900), (1060, 1180), (1300, 1420), (2160, 2280) ]

# 0446, 0578), (0780,0871), (2554, 2671)


splits = [(2172,2249),(889,945), (1188,1256)]

s = SegmentCSV.Segmenter('/home/rafi/logging_nine/2/[11700-14800]markers_fixed.csv', './markers.csv')
s.segment(splits)

s = SegmentCSV.Segmenter('/home/rafi/logging_nine/2/[11700-14800]markers_dropped.csv', './markers_dropped.csv')
s.segment(splits)