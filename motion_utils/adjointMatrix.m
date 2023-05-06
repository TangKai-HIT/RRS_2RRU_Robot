function Ad = adjointMatrix(SE3)

R = SE3(1:3, 1:3);
skewP = skewMatrix(SE3(1:3, 4));
Ad = [R,          zeros(3,3);
         skewP * R,       R];