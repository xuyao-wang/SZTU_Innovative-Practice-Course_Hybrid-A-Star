% 绘制需要的图
function plot_all(case_data,costmap)
close all;
%% 绘制场景

f4 = figure;ax = axes(f4);%%创建坐标轴对象
hold on;%box on %将坐标轴框架显示在当前坐标轴上
plot_costmap(costmap);

%显示路径规划器的可视化，但不显示路径本身和方向箭头
show(case_data.planner,'Path','off','HeadingLength',0);%设置path不显示路径，HeadingLength设置箭头长度为0，即不显示
hold on;

plot_obs(ax,case_data);
plot_car(ax,case_data);
% p1 = plot(case_data.has.x,case_data.has.y,color=[0.85,0.33,0.10]);
p1 = plot(case_data.has.x, case_data.has.y, 'Color', [0.85, 0.33, 0.10]);%指定RGB绘图颜色
axis equal;
xlim([-10,10]);ylim([-6,9]);
% 线条
p1.LineWidth = 3;%设置第一个线条的线宽为2磅

end

function plot_costmap(map)
global params_
    vehicleDims = vehicleDimensions('Length', params_.vehicle.length, ...
                                    'Width', params_.vehicle.lb, ...
                                    'Wheelbase', params_.vehicle.lw, ...
                                    'RearOverhang', params_.vehicle.lr, ...
                                    'FrontOverhang', params_.vehicle.lf);%设置车辆尺寸
    ccConfig = inflationCollisionChecker(vehicleDims, 'InflationRadius',params_.vehicle.radius-0.1);
    %代价地图
    costmap1 = vehicleCostmap(map.*0.8, ...
                              'CellSize', params_.hybrid_astar.dx,...
                              'CollisionChecker',ccConfig, ...
                              'MapLocation',[-20.05 -20.05]); % [-19.95 -19.95]
    
    plot(costmap1, 'Inflation', 'on');%膨胀显示
    legend('hide')%添加图例，hide表示隐藏图例
    hold on;
end

function plot_car(ax,case_data)
    disp(['绘制起末点车辆线框']);%disp()函数用于显示字符串或数值，此处显示字符串，可用于debug
    V = car_plot(case_data.start_pose);%初始start_pose设为0,0,0
    plot(ax, V.x, V.y, 'r', 'LineWidth', 2);%r表示绘制的线条为红色
    hold(ax,'on');
    %scatter()函数用于绘制散点图，每个散点的位置可以由x和y指定
    scatter(ax,case_data.start_pose(1),case_data.start_pose(2),"r",'filled');%filled表示填充散点，使其显示为实心圆点
    V = car_plot(case_data.goal_pose);%目标goal_pose设为-7,4,0.7
    plot(ax, V.x, V.y, 'g', 'LineWidth', 2);%灰色
    scatter(ax,case_data.goal_pose(1),case_data.goal_pose(2),"g",'filled');
end

function plot_obs(ax,case_data)
    disp(['绘制障碍物线框']);
    for i=1:length(case_data.obstacle)
        obs = case_data.obstacle{i};
        %fill函数用于绘制填充的多边形，k表示边框黑色，FaceColor表示填充颜色，FaceAlpha表示填充透明度
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
        V_i = car_plot(path_i);%调用carplot函数绘制车辆线框
        p = plot(ax, V_i.x, V_i.y);%在指定坐标轴上绘制车辆的图形
    	p.LineWidth = 1;%设置车辆轨迹线宽度
        p.Color = 'k';
    end
end

function curv = PJcurvature(x,y)%定义函数用于计算平面曲线的曲率
    h1 = abs(diff(x));%diff求相邻点之间的差，abs保证为正
    h = [h1 h1(end)];
    ht = h;
     
    y1 = gradient(y)./ht;%%y的梯度除以ht，得到曲线在每个点的曲率值
    y2 = gradient(y1)./ht;
    curv = abs(y2)./sqrt((1+y1.^2).^3);%%计算曲率
end