function  [data] = readMarkerData(fname)

% display(fname)
data_tmp = csvread(fname); % read matrix from file

nb_cols = size(data_tmp, 2);

data = zeros( 1, nb_cols + 2 );
data(1,1) = 0;
data(1,2) = 0;
data(1,3:end) = data_tmp;