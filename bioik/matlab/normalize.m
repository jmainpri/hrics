% this function normilizes collumns of the matrix
function y = normalize(x)
%y = normc(x);
y(:,1) = x(:,1) / norm(x(:,1));
y(:,2) = x(:,2) / norm(x(:,2));
y(:,3) = x(:,3) / norm(x(:,3));
