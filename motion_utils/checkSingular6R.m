function isSingular = checkSingular6R(config, screws, tol)
%CHECKSINGULAR6R check jacobian matrix singularity of a 6R serial robot
%   Inputs:
%       config
%       screws: 6 X 6
%       tol: tolerance

J_body = getGeoJacobBodySerial(screws, config);

% condition number
eigens = svd(J_body);

if eigens(end)/eigens(1) < tol
    isSingular = true;
else
    isSingular = false;
end

end