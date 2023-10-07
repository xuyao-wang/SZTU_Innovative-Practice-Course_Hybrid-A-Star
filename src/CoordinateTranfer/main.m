clc;clear;cla
v = 3;          % 速度，单位m/s
delta = 0.1745; % 前轮转向角度，单位rad，约为10°
x_init = 0;     % 车辆后轴中心点初始x坐标
y_init = -10;     % 车辆后轴中心点初始x坐标
theta_init = 0; % 车辆横摆角初始角度（车身与x方向夹角）
L = 3;          % 轴距，单位m
B = 1.6;     % 轮距，单位m
Sf = 1;         % 前悬长，单位m
Sr = 1;         % 后悬长，单位m

% 虚拟轨迹（chapter3控制用）
% virtual_Traj = [];

% 地图（chapter4规划用）
% map = [];

% 仿真参数设置
ts = 0.1;      % 仿真步长，0.1s
t = 10;         % 仿真总时长，10s


% 主函数

%%%%%%%%%%%%%%% 【Chapter4】 路径规划 %%%%%%%%%%%%%%%
%%% 目标：完成route_plan函数
%%% 输入：地图、起点、终点；输出：路径轨迹点

% [route_X,route_Y] = route_plan(map,start,destination);


for i = 0.1:ts:t

    %%%%%%%%%%%%%%% 【Chapter1】 车辆运动学模型 %%%%%%%%%%%%%%%
    %%% 假设固定输入速度和方向盘转角，计算车辆运动轨迹
    %%% 目标：完善vehicleKineticModel函数，
    %%% 输入：车速v，前轮转角delta，车辆位置x_init,yinit，横摆角theta_init，轴距L，仿真步长ts
    %%% 输出：下一时刻车辆位置x_next,y_next，下一时刻横摆角theta_next
    %%% 使得输入车辆的当前位置、速度、方向盘转角即可计算下一时刻的车辆位置和航向角。

    [x_next,y_next,theta_next]=vehicleKineticModel(v,delta,x_init,y_init,theta_init,L,ts);

    % 绘制代表车辆的点的位置变化图（动图）
    plot(x_next,y_next,'o');axis equal;
    xlim([-20 20]);ylim([-10.5 25]);
    hold off;pause(ts);
    % 绘制轨迹图
    % plot(x_next,y_next,'o');hold on


%     %%%%%%%%%%%%%%% 【Chapter2】 利用坐标变换，计算车辆四个端点位置 %%%%%%%%%%%%%%%
%     %%% 目标：完善coordinateTransfer函数
%     %%% 输入：车辆位姿x,y,theta，轴距L，车宽B，前悬长Sf,后悬长Sr
%     %%% 输出：车辆四个顶点：左前：(x1,y1),右前：(x2,y2),左后：(x3,y3),右后：(x4,y4)
% 
    [x1,y1,x2,y2,x3,y3,x4,y4]=coordinateTransfer(x_next,y_next,theta_next,L,B,Sf,Sr);
%
%     % 绘制车辆四个顶点
    draw_X = [x1,x2,x3,x4;x2,x3,x4,x1];
    draw_Y = [y1,y2,y3,y4;y2,y3,y4,y1];
    line(draw_X,draw_Y,'Color','black');

    hold off;pause(ts);
    %%%%%%%%%%%%%%% 【Chapter3-1】 利用pid控制车辆沿着规定轨迹行驶【可选】 %%%%%%%%%%%%%%%
    %%% 在进行chapter3的调试过程中，需要先注注释掉main.m中的chapter1和chapter2。
    %%% 只需要保留写好的vehicleKineticModel和coordinateTranfer函数即可。
    %%% Tips:可以使用ctrl+R整段注释，ctrl+T取消注释。
    %%% 

%     delta_next = pid_Control(x_next,y_next,virtual_Traj);
%     % 绘制真实轨迹
%     plot(x_control_next,y_control_next,'*');
%     [xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4]=coordinateTransfer(x_control_next,y_control_next);
%     % 绘制车辆四个顶点
%     draw_Xc = [xc1,xc2,xc3,xc4;xc2,xc3,xc4,xc1];
%     draw_Yc = [yc1,yc2,yc3,yc4;yc2,yc3,yc4,yc1];
%     line(draw_Xc,draw_Yc,'Color','red');
    
    %%%%%%%%%%%%%%% 【Chapter3-2】 利用stanley控制车辆沿着规定轨迹行驶【可选】 %%%%%%%%%%%%%%%
    %%% 在进行chapter3的调试过程中，需要先注注释掉main.m中的chapter1和chapter2。
    %%% 只需要保留写好的vehicleKineticModel和coordinateTranfer函数即可。
    %%% Tips:可以使用ctrl+T整段注释，ctrl+R取消注释。

%     [x_control_next,y_control_next]=stanley_Control(x_next,y_next,virtual_Traj);
%     % 绘制真实轨迹
%     plot(x_control_next,y_control_next,'*');
%     [xc1,yc1,xc2,yc2,xc3,yc3,xc4,yc4]=coordinateTransfer(x_control_next,y_control_next);
%     % 绘制车辆四个顶点
%     draw_Xc = [xc1,xc2,xc3,xc4;xc2,xc3,xc4,xc1];
%     draw_Yc = [yc1,yc2,yc3,yc4;yc2,yc3,yc4,yc1];
%     line(draw_Xc,draw_Yc,'Color','red');

    % 迭代
    x_init = x_next;
    y_init = y_next;
    theta_init = theta_next;
%     delta = delta_next;
end
axis equal;






