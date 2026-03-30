function [wci, dwci] = cal_angrate_wc_wcdot(Rci, ui, dui, d2ui)
%%%% 计算姿态角速率wc以及姿态角加速度dwc
%%%% 参考ZouYao-IJC-2016论文
%%%% <Nonlinear Hierarchical Control for Quad-Rotors with Rotation Matrix>
%%%% 或者参考ZouYao-IEEETCYB-2022论文
%%%% <Adaptive Coordinated Formation Control of Heterogeneous Vertical Takeoff and Landing UAVs Subject to Parametric Uncertainties>
%%%% 论文中附录给出了求解方法

ee1 = [1;0;0];
ee2 = [0;1;0];
ee3 = [0;0;1];
RciE3 = ui / norm(ui,2);
PiMat = eye(3,3) - ee3*(ee3');
dRciE3 = (1 / norm(ui,2))*Rci*PiMat*(Rci')*dui;
Themat = [-RciE3(2)/(1+RciE3(3)),         -RciE3(1)/(1+RciE3(3)),       (RciE3(1)*RciE3(2))/((1+RciE3(3))^2); ...
                   2*RciE3(1)/(1+RciE3(3)),                0,                               1 - (RciE3(1)^2)/((1+RciE3(3))^2); ...
                               0                                       -1                                                        0                                   ];
wcxe3 = (1 / norm(ui,2))*PiMat*(Rci')*dui;
wcxe2 = (1 / norm(ui,2))*(Rci')*Themat*Rci*PiMat*(Rci')*dui;
wci = [-wcxe3(2); wcxe3(1); -wcxe2(1)];

TempMat1 = [0,                           -1/(1+RciE3(3)),                   (RciE3(2))/((1+RciE3(3))^2); ...
                       2/(1+RciE3(3)),                0,                               (-2*RciE3(1))/((1+RciE3(3))^2); ...
                       0,                                     0,                                                   0                             ];
TempMat2 = [-1/(1+RciE3(3)),              0,                               (RciE3(1))/((1+RciE3(3))^2); ...
                       0,                                     0,                                                   0                      ; ...
                       0,                                     0,                                                   0                             ];
TempMat3 = [(RciE3(2))/((1+RciE3(3))^2),                 (RciE3(1))/((1+RciE3(3))^2),       (-2*RciE3(1)*RciE3(2))/((1+RciE3(3))^3); ...
                       (-2*RciE3(1))/((1+RciE3(3))^2),                             0,                               (2*(RciE3(1)^2))/((1+RciE3(3))^3); ...
                        0,                                                                          0,                                                   0                             ];
dTheMat = TempMat1*dRciE3*(ee1') + TempMat2*dRciE3*(ee2') + TempMat3*dRciE3*(ee3');
wcix = cal_askew_mat(wci);
dRciT = -wcix*(Rci');
dwcxe3 = (1 / norm(ui,2))*PiMat*(dRciT*dui+(Rci')*(d2ui-(((ui')*dui)/(norm(ui,2)^2))*dui));
dwcxe2 = (1 / norm(ui,2))*((dRciT*Themat+(Rci')*dTheMat) ...
                *(Rci*PiMat*(Rci'))*dui + (Rci')*Themat*(Rci*PiMat*(Rci'))*(d2ui-(((ui')*dui)/(norm(ui,2)^2))*dui));
dwci = [-dwcxe3(2);  dwcxe3(1);  -dwcxe2(1)];

end

