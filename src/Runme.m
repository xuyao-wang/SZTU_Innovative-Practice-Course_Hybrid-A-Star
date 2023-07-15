% clc;clear;cla;close
close;
%% 设置基本参数
k = 0.1;  % look forward gain观测前向增益
Kp = 1.0 ; % speed propotional gain
dt = 0.1  ;% 仿真步长[s]
Lfc = 1; % 纯跟踪基础预瞄距离
target_speed = 10/3.6; % 目标速度 单位m/s
% 车辆参数
vehicle_lw = 2.6; % wheelbase 轴距 m
vehicle_lf = 0.96; % front hang length 前悬长度
vehicle_lr = 0.87; % rear hang length 后悬长度
vehicle_lb = 2; % width 车宽

%% 规划模块提供路径
cx = 0:0.1:100; % sampling interception from 0 to 100, with step 0.1
for i = 1:500  % here we create a original reference line, which the vehicle should always follow when there is no obstacles;
    cy(i) = -sin(cx(i)/10)*cx(i)/8;
end
for i = 501: length(cx)
    cy(i) = -sin(cx(i)/10)*cx(i)/8; %cy(500);
end

% cx = px1.x;
% cy = px1.y;
lastIndex = length(cx);
%% 仿真设置  
T = 15; % 仿真时间
% 初始状态
x = 0; y = 2; yaw = 0; v = 0;
%x = cx(1); y = cy(1); yaw = 0; v = 0.5;
time = 0;
target_index = 0;
% 记录真实轨迹
tx=[];ty=[];
while T > time && target_index <length(cx)
   %% 根据参考点计算误差 
   [error, target_index] = calc_target_index(x,y,yaw,cx,cy,vehicle_lw);
    %% 根据误差进行跟踪
    % 纵向p控制器
    ai = PIDcontrol(target_speed, v,Kp);
    % 横向控制器
    %delta = stanley_control(x,y,yaw,v,cx,cy,target_index, lastIndex, error);    
    delta = pure_pursuit_control(x,y,yaw,v,cx,cy,target_index,k,Lfc,vehicle_lw);

    %% 根据控制量更新模型
    [x,y,yaw,v] = update(x,y,yaw,v, ai, delta,dt,vehicle_lw);
    time = time + dt;
    tx=[tx,x];
    ty=[ty,y];
   
    %% 绘制轨迹
    % 期望轨迹
    plot(cx,cy,'b',x,y,'r-*','LineWidth',0.2)
    hold on;
    %绘制车辆
    V = car_plot([x,y,yaw]);
    plot(V.x, V.y, 'g', 'LineWidth', 2);axis equal;
    % 车轮
    [w1x, w1y] = v2g_1(0,-vehicle_lb/2,x,y,yaw);
    [w2x, w2y] = v2g_1(0,vehicle_lb/2,x,y,yaw);
    [w3x, w3y] = v2g(vehicle_lw,-vehicle_lb/2,x,y,yaw);
    [w4x, w4y] = v2g(vehicle_lw,vehicle_lb/2,x,y,yaw);
    w1 = wheel_plot([w1x, w1y,yaw]);
    w2 = wheel_plot([w2x, w2y,yaw]);
    w3 = wheel_plot([w3x, w3y,delta+yaw]);
    w4 = wheel_plot([w4x, w4y,delta+yaw]);
    plot(w1.x, w1.y, 'k');
    plot(w2.x, w2.y, 'k');
    plot(w3.x, w3.y, 'k');
    plot(w4.x, w4.y, 'k');
    % 真实轨迹
    plot(tx, ty, 'r*');
    scatter(x,y,"r",'filled');
    xlim([0,100]);ylim([-25,25]);
    hold off;
    pause(dt);
end
%% 车辆模型
function [x, y, yaw, v] = update(x, y, yaw, v, a, delta,dt,L)
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;
    v = v + a * dt;
end

%% 速度p控制
function [a] = PIDcontrol(target_v, current_v, Kp)
    a = Kp * (target_v - current_v);
end

%% 横向stanley
function [delta] = stanley_control(x,y,yaw,v,cx,cy, ind, lastIndex, error)
    tx = cx( min(ind+1, lastIndex));% 防止超出索引
    ty = cy( min(ind+1, lastIndex));
    
    % delta_yaw = v * 0.1 * sin(yaw) / L;
    alpha = pipi(atan((ty-y)/(tx-x)) -yaw);
    gain_k  = 1;
    theta = atan(gain_k* error/v);
    delta = alpha + theta;
end

%% 横向pp
function [delta] = pure_pursuit_control(x,y,yaw,v,cx,cy,ind,k,Lfc,L)  %纯追踪控制器前轮转角方程的函数
    tx = cx(ind);                          %距离车辆当前最近的点的位置对应的cx值     ind是距离车辆当前最近的点的位置
    ty = cy(ind);                          %距离车辆当前最近的点的位置对应的cy值
    
    alpha = atan((ty-y)/(tx-x))-yaw;       %该处定义向左转为alpha=beta-Fai，所以向右转就输出-alpha
    
 Ld = k * v + Lfc ;                        % 预瞄距离Ld与车速成线性关系 
 delta = atan(2*L * sin(alpha)/Ld);                %前轮转角
end

%% 得到参考点索引
function [error, ind] = calc_target_index(x,y,yaw, cx,cy,L)
x = x + L * cos(yaw);
y = y + L * sin(yaw);
N =  length(cx);
Distance = zeros(N,1);
for i = 1:N
    Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);
end
[value, location]= min(Distance);
ind = location;
  if y<cy(ind) 
        error = -value;
  else
        error = value;
  end
end

%% 角度限制
function [angle] = pipi(angle)
    if (angle > pi)
        angle =  angle - 2*pi;
    elseif (angle < -pi)
        angle = angle + 2*pi;
    else
        angle = angle;
    end
end

%% 绘制车辆矩形
function V = car_plot(pose)
    x = pose(1);
    y = pose(2);
    theta = pose(3); 
    % 参数
    vehicle_lw = 2.6; % wheelbase 轴距 m
    vehicle_lf = 0.96; % front hang length 前悬长度
    vehicle_lr = 0.87; % rear hang length 后悬长度
    vehicle_lb = 1.9; % width 车宽
    
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

%% 绘制车轮矩形
function V = wheel_plot(pose)
    x = pose(1);
    y = pose(2);
    theta = pose(3); 
    % 参数
    vehicle_wl = 0.35; % 轮长一半
    vehicle_lb = 0.4; % 轮宽
    
    cos_theta = cos(theta);
    sin_theta = sin(theta);
    vehicle_half_width = vehicle_lb * 0.5;
    AX = x + vehicle_wl * cos_theta - vehicle_half_width * sin_theta;
    BX = x + vehicle_wl * cos_theta + vehicle_half_width * sin_theta;
    CX = x - vehicle_wl * cos_theta + vehicle_half_width * sin_theta;
    DX = x - vehicle_wl * cos_theta - vehicle_half_width * sin_theta;
    AY = y + vehicle_wl * sin_theta + vehicle_half_width * cos_theta;
    BY = y + vehicle_wl * sin_theta - vehicle_half_width * cos_theta;
    CY = y - vehicle_wl * sin_theta - vehicle_half_width * cos_theta;
    DY = y - vehicle_wl * sin_theta + vehicle_half_width * cos_theta;
    V.x = [AX, BX, CX, DX, AX];
    V.y = [AY, BY, CY, DY, AY];
end
%% 车辆转全局
function [gx,gy] = v2g(cx, cy, x, y, theta)
    gx = x + cos(theta)*cx - sin(theta)*cy;
    gy = y + sin(theta)*cx + cos(theta)*cy;
end
% 矩阵形式
function [gx,gy] = v2g_1(cx, cy, x, y, theta)
    ctheta = 1;
    g = [cos(theta), -sin(theta), 0; ...
        sin(theta), cos(theta), 0; ...
        0, 0, 1] * [cx; cy; ctheta] + [x,; y; theta];
    gx = g(1);
    gy = g(2);
end



