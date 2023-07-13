% 绘制障碍物矩形
function O = obs_plot(obs)
    x = obs.x0;
    y = obs.y0;
    theta = obs.theta0;
    l = obs.l;
    w = obs.w;
    % 参数
    % 参数
    vehicle_lw = l*0.25; % wheelbase 轴距 m
    vehicle_lf = l*0.25; % front hang length 前悬挂长度
    vehicle_lr = l*0.5; % rear hang length 后悬挂长度
    vehicle_lb = w; % width 车宽
    
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
    O.x = [AX, BX, CX, DX, AX];
    O.y = [AY, BY, CY, DY, AY];
end
