function  [data] = readViconData(fname,nb_cols)

data_tmp = csvread(fname); % read matrix from file

nb_rows = size(data_tmp, 1);
if nargin < 2, % default
   nb_cols = size(data_tmp, 2) - ( size(data_tmp, 2) - 2 ) - 1;
end

data = zeros( nb_cols, nb_cols );

time_origin = data_tmp(1,1) + 1e-09 * data_tmp(1,2);

for i=1:nb_rows,
    data(i,1) = i;
    data(i,2) = data_tmp(i,1) + 1e-09 * data_tmp(i,2);
    data(i,2) = data(i,2) - time_origin;
    for j=0:data_tmp(i,3)-1,
        for k=0:2,
            data(i,j*3+k+3) = data_tmp(i,j*4+k+5);
        end
    end
end
