function Q = cal_Q_mul_Q(Q1, Q2)
%%%% 计算两个单位四元数相乘
sigma1 = Q1(1);
q1 = Q1(2:4);
sigma2 = Q2(1);
q2 = Q2(2:4);

sigma = sigma1*sigma2 - (q1')*q2;
q = sigma1*q2 + sigma2*q1 + cal_askew_mat(q1)*q2;
Q = [sigma; q];

end

