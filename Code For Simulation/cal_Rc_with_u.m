function Rc = cal_Rc_with_u(m, u)
%%%% 通过质量m和控制指令u
%%%% 计算指令姿态旋转矩阵Rc

T = m * norm(u,2);
ux = u(1);   uy = u(2);   uz = u(3);
Rc = (m/T) * [uz+(m*(uy^2))/(T+m*uz),         -(m*ux*uy)/(T+m*uz),             ux; ...
                      -(m*ux*uy)/(T+m*uz),                uz+(m*(ux^2))/(T+m*uz),      uy; ...
                      -ux,                                             -uy,                                         uz ];

end
