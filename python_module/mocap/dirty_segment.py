import SegmentCSV


# splits = [ (0, 1700), (1750, 3200), (3250, 5000), (5050, 6100), (6150, 7700), (7750, 8900), (8950, 10400), (10450, 11600), ]

# splits = [(1000,3900), (5900,9000), (11700,14800), (17700,20500), (22400,25300), (28300,30800), (33000,35700), (37900,40400), (42600,44700), ]

splits = [ (580, 680), (760, 900), (1060, 1180), (1300, 1420), (2160, 2280) ]




s = SegmentCSV.Segmenter('/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[1000-3900]markers_fixed.csv', './markers.csv')
s.segment(splits)

s = SegmentCSV.Segmenter('/home/rafi/workspace/hrics-or-plugins/python_module/mocap/[1000-3900]objects_fixed.csv', './objects.csv')
s.segment(splits)