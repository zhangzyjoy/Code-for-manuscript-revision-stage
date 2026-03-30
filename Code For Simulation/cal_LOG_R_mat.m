function logR = cal_LOG_R_mat(R, order)
%%%% 输入量<R>为SO(3)空间(R^T=R^(-1))中的姿态旋转矩阵
%%%% 输入量<order>为log泰勒展开简化计算的阶数

logR = zeros(3,3);
for i = 1:1:order
    logR = logR + ((-1)^(i+1)) * ((R - eye(3,3))^i) / i;
end

end

