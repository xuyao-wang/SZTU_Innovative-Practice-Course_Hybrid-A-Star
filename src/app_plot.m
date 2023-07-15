function app_plot(case_data, plt, gs)

ax = plt.ax;
if plt.run == "runme"
    plt.park = 1;
    plt.obs = 1;
    plt.path = 1;
    plt.opt_path = 1;
    plt.mark = [];
    plt.interfere = 0;
    plt.costmap = 0;
    plt.hold = 0;
end
    
% 绘制车辆线框
disp(['绘制起末点车辆线框']);
V = car_plot(case_data.start_pose);
plot(plt.ax, V.x, V.y, 'r', 'LineWidth', 2);
hold(ax,'on');
scatter(ax,case_data.start_pose(1),case_data.start_pose(2),"r",'filled');
V = car_plot(case_data.goal_pose);
plot(plt.ax, V.x, V.y, 'g', 'LineWidth', 2);
scatter(ax,case_data.goal_pose(1),case_data.goal_pose(2),"g",'filled');

% costmap

% 如果有停车位则绘制
if plt.park && (~isempty(case_data.parking)) %检查是否需要绘制停车位线框，且确保停车位数据不为空
    disp(['绘制停车位线框']);
    for i=1:length(case_data.parking)
        park = case_data.parking{i};
        fill(ax, park.x, park.y, [0 0.4470 0.7410], 'LineStyle', '-.','FaceAlpha',0.1);
    end
end

% 如果有障碍物则绘制
if plt.obs && (~isempty(case_data.obstacle))
    disp(['绘制障碍物线框']);
    for i=1:length(case_data.obstacle)
        obs = case_data.obstacle{i};
        o = fill(ax, obs.x, obs.y, 'k', 'FaceColor',[0.6,0.6 0.6]);%, 'FaceAlpha',0.5);[0.2 0.2 0.2]

    end
end

% 如果有规划路径则绘制
if plt.path && isfield(case_data.has,'x')
    disp(['绘制搜索路径线框']);
    has = case_data.has;
    plot(ax,has.x,has.y); 
end

% 如果有优化路径则绘制
if plt.opt_path && isfield(case_data.opt,'x')
    disp(['绘制优化路径线框']);
    opt = case_data.opt;
    plot(ax,opt.x,opt.y); 
end

% 标记选择的障碍物或者停车位
if (length(plt.mark) > 1)
        plot(ax, plt.mark(1,:), plt.mark(2,:), 'r', 'LineWidth', 2);
        fill(ax, plt.mark(1,:), plt.mark(2,:), 'r', 'FaceAlpha',0.2);

end

% 绘制干涉图形
if plt.interfere && (exist('gs','var'))
    cyn = gs(2);
    if (gs(1) == 1)
            path = case_data.has;
    elseif (gs(1) == 2)
            path = case_data.opt;
    end
    %循环绘制每个路径点的线框
    for i=1:cyn:length(path.x)
        path_i = [path.x(i), path.y(i),path.theta(i)];
        V_i = car_plot(path_i);
        plot(ax, V_i.x, V_i.y, 'k', 'LineWidth', 1);
    end
    hold(ax,'off');
end


% 绘制动画
%if plt.motion && (exist('gs','var'))
    
    
% 绘制结束，下次更新清屏
if plt.box || plt.hold
    hold(ax,'on');
else
    hold(ax,'off');
end
end 

%% 绘制车辆矩形框
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

%% 绘制障碍物矩形
function O = obs_plot(obs)
    x = obs.x;
    y = obs.y;
    theta = obs.theta;
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

