%% 绘制障碍物矩形
function V = car_plot(pose)
    global params_
    x = pose(1);
    y = pose(2);
    theta = pose(3); 
    % 参数
    vehicle_lw = params_.vehicle.lw; % wheelbase 轴距 m
    vehicle_lf = params_.vehicle.lf; % front hang length 前悬挂长度
    vehicle_lr = params_.vehicle.lr; % rear hang length 后悬挂长度
    vehicle_lb = params_.vehicle.lb; % width 车宽
    
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    vehicle_half_width = vehicle_lb * 0.5;
    AX = x + (vehicle_lf + vehicle_lw) * cos_theta - vehicle_half_width * sin_theta;
    BX = x + (vehicle_lf + vehicle_lw) * cos_theta + vehicle_half_width * sin_theta;
    CX = x - vehicle_lr * cos_theta + vehicle_half_width * sin_theta;
    DX = x - vehicle_lr * cos_theta - vehicle_half_width * sin_theta;
    AY = y + (vehicle_lf + vehicle_lw) * sin_theta + vehicle_half_width * cos_theta;
    BY = y + (vehicle_lf + vehicle_lw) * sin_theta - vehicle_half_width * cos_theta;
    CY = y - vehicle_lr * sin_theta - vehicle_half_width * cos_theta;
    DY = y - vehicle_lr * sin_theta + vehicle_half_width * cos_theta;
    V.x = [AX, BX, CX, DX, AX];
    V.y = [AY, BY, CY, DY, AY];
end