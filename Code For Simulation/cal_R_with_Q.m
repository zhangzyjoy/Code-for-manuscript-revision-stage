function R = cal_R_with_Q(Q)
%%%% 由单位四元数计算姿态旋转矩阵
sigma = Q(1);
qvec = Q(2:4);
qvec = qvec(:);

R = (sigma^2-(qvec')*qvec)*eye(3,3) + ...
       2*qvec*(qvec') - 2*sigma*cal_askew_mat(qvec);

end
