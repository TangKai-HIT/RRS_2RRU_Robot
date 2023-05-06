function new_se3 = adjointMapSE3(SE3, se3)

inv_SE3 = [SE3(1:3,1:3)',   -SE3(1:3,1:3)' * SE3(1:3,4);
                        zeros(1,3),             1];

new_se3 = SE3 * se3 * inv_SE3;