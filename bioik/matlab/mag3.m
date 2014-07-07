% this function finds the magnitude of a 3-d vector
function [mag] = mag3(vector)
mag=sqrt(vector(:,1).^2+vector(:,2).^2+vector(:,3).^2);