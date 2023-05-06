function geometricJacob = getGeoJacobBodySerial(twists, thetas)
%GETGEOJACOBBODYSERIAL get geometric body jacobian of a serial robot
%   Inputs: 
%       twists: 6 X N, body twist
%       thetas: 1 X N or N X 1
%   Outputs:
%       geometricJacob: 6 X N

N = length(thetas);
geometricJacob = zeros(6, N);

Tf = eye(4);
geometricJacob(:, N) = twists(:, N);

for i=N-1:-1:1
    Tf = Tf * expMapSE3(twists(:, i+1), -thetas(i+1));
    Ad = adjointMatrix(Tf);
    geometricJacob(:, i) = Ad * twists(:, i);
end