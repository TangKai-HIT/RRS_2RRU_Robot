function vec = se3Vec(se3)

vec = zeros(6,1);

vec(1:3) = skew2Vec(se3(1:3, 1:3));
vec(4:6) = se3(1:3, 4);