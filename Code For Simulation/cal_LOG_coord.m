function Phi = cal_LOG_coord(R)
%%%% 计算向量在Lie代数空间内的对数映射
phi = 0.5 * ( trace(R) - 1 );
Phi_X = ( phi / ( 2 * sin(phi) ) ) * ( R - R' );
Phi = zeros(3,1);
Phi(1,1) = -Phi_X(2,3);
Phi(2,1) = Phi_X(1,3);
Phi(3,1) = -Phi_X(1,2);
end

