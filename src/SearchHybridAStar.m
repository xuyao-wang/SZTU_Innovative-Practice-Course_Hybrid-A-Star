%% 混合a star 路径搜索
function [x, y, theta, costmap, planner] = SearchHybridAStar(start_pose,goal_pose,costmap)
global params_
tic; %计时
disp("Search via HAS matlab")
vehicleDims = vehicleDimensions('Length', params_.vehicle.length, ...
                                'Width', params_.vehicle.lb, ...
                                'Wheelbase', params_.vehicle.lw, ...
                                'RearOverhang', params_.vehicle.lr, ...
                                'FrontOverhang', params_.vehicle.lf);
ccConfig = inflationCollisionChecker(vehicleDims, 'InflationRadius',params_.vehicle.radius);%%碰撞检测器
map = vehicleCostmap(costmap.*0.8, ...%%降低代价地图中的障碍物代价，使规划的路径更倾向于绕过障碍物
                          'CellSize', params_.hybrid_astar.dx,...%%代价地图单元格大小
                          'CollisionChecker',ccConfig, ...
                          'MapLocation',[-20.05 -20.05]); % [-19.95 -19.95]起始位置
% 验证器
ss = stateSpaceSE2;
validator = validatorVehicleCostmap(ss,"Map",map);
validator.ValidationDistance = 0.1;
validator.StateSpace.StateBounds=[-20,20;-20,20;-pi,pi];

% isValid = isStateValid(validator,[start_pose;goal_pose]);
planner = plannerHybridAStar(validator,...
                            'MinTurningRadius',2.5,...% MinTurningRadius 最小转弯半径  3 %2.5
                            'MotionPrimitiveLength',2,...% MotionPrimitiveLength 运动基元长度 4*a m %1.5
                            'NumMotionPrimitives',5,...% NumMotionPrimitives 要生成的运动基元个数 >= 3 奇数
                            'ReverseCost',1,'ForwardCost',1,...% ForwardCost 正向权重 默认1% ReverseCost 反向权重 默认1
                            'DirectionSwitchingCost', 0,...% DirectionSwitchingCost 切换方向 默认0
                            'InterpolationDistance',0.1,...% InterpolationDistance 路径插值距离 默认1
                            'AnalyticExpansionInterval',5);% AnalyticExpansionInterval 尝试扩展 默认5
%show(planner)
refPath = plan(planner,start_pose,goal_pose,'SearchMode','exhaustive');
path = refPath.States;
x = path(1:end,1);
y = path(1:end,2);
theta = path(1:end,3);

toc;
end

