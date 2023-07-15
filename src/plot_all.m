% 绘制需要的图
function plot_all(case_data,costmap)
close all;
%% 绘制场景

f4 = figure;ax = axes(f4);hold on;%box on
plot_costmap(costmap);

show(case_data.planner,'Path','off','HeadingLength',0);hold on;

plot_obs(ax,case_data);
plot_car(ax,case_data);
p1 = plot(case_data.has.x,case_data.has.y,color=[0.85,0.33,0.10]);
axis equal;xlim([-10,10]);ylim([-6,9]);
% 线条
p1.LineWidth = 3;%设置第一个线条的线宽为2磅


end

function plot_costmap(map)
global params_
    vehicleDims = vehicleDimensions('Length', params_.vehicle.length, ...
                                    'Width', params_.vehicle.lb, ...
                                    'Wheelbase', params_.vehicle.lw, ...
                                    'RearOverhang', params_.vehicle.lr, ...
                                    'FrontOverhang', params_.vehicle.lf);
    ccConfig = inflationCollisionChecker(vehicleDims, 'InflationRadius',params_.vehicle.radius-0.1);
    costmap1 = vehicleCostmap(map.*0.8, ...
                              'CellSize', params_.hybrid_astar.dx,...
                              'CollisionChecker',ccConfig, ...
                              'MapLocation',[-20.05 -20.05]); % [-19.95 -19.95]

    plot(costmap1, 'Inflation', 'on');
    legend('hide')
    hold on;
end

function plot_car(ax,case_data)
    disp(['绘制起末点车辆线框']);
    V = car_plot(case_data.start_pose);
    plot(ax, V.x, V.y, 'r', 'LineWidth', 2);
    hold(ax,'on');
    scatter(ax,case_data.start_pose(1),case_data.start_pose(2),"r",'filled');
    V = car_plot(case_data.goal_pose);
    plot(ax, V.x, V.y, 'g', 'LineWidth', 2);
    scatter(ax,case_data.goal_pose(1),case_data.goal_pose(2),"g",'filled');
end

function plot_obs(ax,case_data)
    disp(['绘制障碍物线框']);
    for i=1:length(case_data.obstacle)
        obs = case_data.obstacle{i};
        o = fill(ax, obs.x, obs.y, 'k', 'FaceColor',[0.6,0.6 0.6]);%, 'FaceAlpha',0.5);[0.2 0.2 0.2]
    end
end

function plot_gs(ax,path,cyn,c)
    disp(['绘制干涉线框']);
    p = plot(ax,path.x,path.y); 
    p.LineWidth = 2;
    p.Color = c;
    %循环绘制每个路径点的线框
    for i=1:cyn:length(path.x)
        path_i = [path.x(i), path.y(i),path.theta(i)];
        V_i = car_plot(path_i);
        p = plot(ax, V_i.x, V_i.y);
    	p.LineWidth = 1;
        p.Color = 'k';
    end
end

function curv = PJcurvature(x,y)
    h1 = abs(diff(x));
    h = [h1 h1(end)];
    ht = h;
     
    y1 = gradient(y)./ht;%%y的梯度除以ht，得到曲线在每个店的曲率值
    y2 = gradient(y1)./ht;
    curv = abs(y2)./sqrt((1+y1.^2).^3);%%计算曲率
end