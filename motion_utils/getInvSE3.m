function inv_SE3 = getInvSE3(SE3)

inv_SE3 = [SE3(1:3,1:3)',   -SE3(1:3,1:3)' * SE3(1:3,4);
                        zeros(1,3),             1];