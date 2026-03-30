function ax = cal_askew_mat(a)
%%%% 计算一个列向量a的反对称矩阵
a = a(:);
ax = [0,      -a(3),      a(2); ...
         a(3),     0,        -a(1);...
        -a(2),   a(1),         0   ];

end

