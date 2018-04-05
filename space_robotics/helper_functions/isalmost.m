function output = isalmost(a,b,tol)
% check if two matrices of floats are almost equal to within tolerance tol.
logicArray = (a <= b+tol) & (a >= b-tol);
output = all(logicArray(:));
end