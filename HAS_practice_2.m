close all
global case_data 

% 初始化参数 
Initialize();

%载入场景
[start_pose, goal_pose, costmap] = Load_case(case_data); 

% 混合a*
[case_data.has.x, case_data.has.y, case_data.has.theta, costmap1, planner] = SearchHybridAStar(start_pose,goal_pose,costmap);% matlab
case_data.planner = planner;
%绘图
px1 = case_data.has;
case_data.px1 = px1;
plot_all(case_data,costmap);
