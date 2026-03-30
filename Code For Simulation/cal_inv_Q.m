function Qinv = cal_inv_Q(Q)
%%%% 计算四元数的逆, 若Q=[thetai,qi], Q^-1=[thetai,-qi]
theta = Q(1,1);
q = Q(2:4,1);
Qinv = [theta; -q];

end

