close;
% %Create a reedsSheppConnection object.
% reedsConnObj = reedsSheppConnection;
% 
% %Define start and goal poses as [x y theta] vectors.
% startPose = [5 1.3 -pi/2];
% goalPose = [0 -1.3 pi/2];
% reedsConnObj.MinTurningRadius = 4;
% 
% %禁用此特定运动序列
% reedsConnObj = reedsSheppConnection('DisabledPathTypes',{'LpRnLp'});
% % 增加逆行成本
% reedsConnObj.ReverseCost = 1;
% reedsConnObj.ForwardCost = 1;
% 
% 
% %Calculate a valid path segment to connect the poses.
% [pathSegObj,pathCosts] = connect(reedsConnObj,startPose,goalPose);
% 
% %Show the generated path. Notice the direction of the turns.
% show(pathSegObj{1})
% hold on;axis equal;
% pathSegObj{1}.MotionTypes
% pathSegObj{1}.MotionDirections
% 
% % 绘制车辆
% V = car_plot(startPose);
% plot(V.x, V.y, 'r', 'LineWidth', 2);
% scatter(goalPose(1),goalPose(2),"r",'filled');
% V = car_plot(goalPose);
% plot(V.x, V.y, 'g', 'LineWidth', 2);
% scatter(goalPose(1),goalPose(2),"g",'filled');
% xlim([-5 30]);
% ylim([-5 20]);



% 多目标位置
reedsConnObj = reedsSheppConnection;  %%使用默认属性值创建对象
reedsConnObj.MinTurningRadius = 6;  %%最小转弯半径为6
%reedsConnObj = reedsSheppConnection('DisabledPathTypes',{'LpRnLp'});

startPose = [0 0 -pi/2];
for i=0:10
    goalPose = [sin(pi/5*i)*10 cos(pi/5*i)*10 -pi/5*i+pi/2+0.1];%%目标位置
    [pathSegObj,pathCosts] = connect(reedsConnObj,startPose,goalPose);
    show(pathSegObj{1})
    hold on;
    % 绘制车辆
    V = car_plot(startPose);
    plot(V.x, V.y, 'r', 'LineWidth', 2);
    scatter(goalPose(1),goalPose(2),"r",'filled');
    V = car_plot(goalPose);
    plot(V.x, V.y, 'g', 'LineWidth', 2);
    scatter(goalPose(1),goalPose(2),"g",'filled');
end
xlim([-15 30]);
ylim([-15 20]);
axis equal;
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
    %% 以后轴中点为原点，建立车辆坐标系
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