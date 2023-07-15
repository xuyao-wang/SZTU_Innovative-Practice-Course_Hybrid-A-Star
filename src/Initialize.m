function params_ = Initialize()
disp("参数 初始化")
global params_
%% 画布大小 m
params_.demo.xmin = -20;
params_.demo.xmax = 20;
params_.demo.ymin = -20;
params_.demo.ymax = 20;
%% 参数
params_.vehicle.lw = 2.6; % wheelbase 轴距 m
params_.vehicle.lf = 0.96; % front hang length 前悬长度
params_.vehicle.lr = 0.87; % rear hang length 后悬长度
params_.vehicle.lb = 1.9; % width 车宽
params_.vehicle.length = params_.vehicle.lw + params_.vehicle.lf + params_.vehicle.lr; % 车总长
params_.vehicle.hypotenuse_length = hypot(params_.vehicle.length, params_.vehicle.lb); % 车矩形弦长
params_.vehicle.radius = hypot(0.25 * params_.vehicle.length, 0.5 * params_.vehicle.lb); % Dual disk radius 双盘半径 1.46
params_.vehicle.radius = 1.3;
params_.vehicle.r2p = 0.25 * params_.vehicle.length - params_.vehicle.lr; % 后圆盘圆心和后轴中心的距离
params_.vehicle.f2p = 0.75 * params_.vehicle.length - params_.vehicle.lr; % 前圆盘圆心和后轴中心的距离
params_.vehicle.vmax = 1.0; % 最大线速度 p4.0 c2.5
params_.vehicle.amax = 0.5; % 最大线加速度 p4.0 c1
params_.vehicle.phymax = 0.7; % 前轮最大转角，车辆最大横摆角度 p0.85， c0.75
params_.vehicle.wmax = 0.7; % 前轮最大转弯角速度 p1，
params_.vehicle.kappa_max = tan(params_.vehicle.phymax) / params_.vehicle.lw; % 曲率=1/半径=tan(车轮转角)/轴距
params_.vehicle.turning_radius_min = abs(1.0 / params_.vehicle.kappa_max); % 最小转弯半径
params_.vehicle.threshold_s = (params_.vehicle.vmax^2) / params_.vehicle.amax;% 达到最大速度需要的路径距离
% 混合 A* 分辨率
params_.hybrid_astar.dx = 0.1; % p0.4
params_.hybrid_astar.dy = 0.1; % p0.4
params_.hybrid_astar.dtheta = 0.2; % p0.2
% 根据分辨率划分节点
params_.hybrid_astar.num_nodes_x = ceil((params_.demo.xmax - params_.demo.xmin) / params_.hybrid_astar.dx) + 1;
params_.hybrid_astar.num_nodes_y = ceil((params_.demo.ymax - params_.demo.ymin) / params_.hybrid_astar.dy) + 1;
params_.hybrid_astar.num_nodes_theta = ceil(2 * pi / params_.hybrid_astar.dtheta) + 1;
end