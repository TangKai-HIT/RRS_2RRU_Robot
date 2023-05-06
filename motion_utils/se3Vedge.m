function se3 = se3Vedge(twist)

omega = twist(1:3);
v = twist(4:6);

if size(v,2)~=1
    v = v';
end

se3 = [skewMatrix(omega)    v;
                zeros(1,3)                  0];