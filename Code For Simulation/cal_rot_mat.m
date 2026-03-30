function Rmat = cal_rot_mat(eular)
%%%% ¼ÆËã×ËÌ¬Ðý×ª¾ØÕó
phi = eular(1);
theta = eular(2);
psi = eular(3);

cphi = cos(phi);      sphi = sin(phi);
ct = cos(theta);       st = sin(theta);
cpsi = cos(psi);       spsi = sin(psi);

Rmat = [ct*cpsi,     cpsi*st*sphi-cphi*spsi,     cphi*cpsi*st+sphi*spsi; ...
              ct*spsi,     spsi*st*sphi+cphi*cpsi,    cphi*spsi*st-sphi*cpsi; ...
              -st,           ct*sphi,                             ct*cphi];

end

