function [start_pose, goal_pose, costmap] = Load_case(case_data)
global params_ 

params_.obstacle.obs = case_data.obstacle;
params_.obstacle.num_obs = length(case_data.obstacle); % 障碍个数
params_.dilated_map = CreateDilatedCostmap(case_data.obstacle); % 创建costmap


start_pose = case_data.start_pose;
goal_pose = case_data.goal_pose;
costmap = params_.dilated_map;
end

%% 创建成本地图 
function costmap = CreateDilatedCostmap(obstacle)
global params_
xmin = params_.demo.xmin; % 画布大小
ymin = params_.demo.ymin;

% 初始化为零
costmap = zeros(params_.hybrid_astar.num_nodes_x, params_.hybrid_astar.num_nodes_y);

for ii = 1 : size(obstacle, 2)
    V = obstacle{ii};
    x_lb = min(V.x);
    x_ub = max(V.x);
    y_lb = min(V.y);
    y_ub = max(V.y);
    [Nmin_x, Nmin_y] = ConvertXYToIndex(x_lb,y_lb);
    [Nmax_x, Nmax_y] = ConvertXYToIndex(x_ub,y_ub);
    for jj = Nmin_x : Nmax_x
        for kk = Nmin_y : Nmax_y
            if (costmap(jj,kk) == 1)
                continue;
            end
            cur_x = xmin + (jj - 1) * params_.hybrid_astar.dx;
            cur_y = ymin + (kk - 1) * params_.hybrid_astar.dy;
            if (inpolygon(cur_x, cur_y, obstacle{ii}.x, obstacle{ii}.y) == 1)
                costmap(jj,kk) = 1;
            end
        end
    end
end
length_unit = ( params_.hybrid_astar.dx + params_.hybrid_astar.dy ) / 2;
basic_elem = strel('disk', ceil(params_.vehicle.radius / length_unit),8); % strel 形态学结构元素
%costmap = imdilate(costmap, basic_elem); % 膨胀运算
costmap = rot90(costmap);
end

%% 将X Y转换为索引
function [ind1,ind2] = ConvertXYToIndex(x,y)
global params_
ind1 = ceil((x - params_.demo.xmin) / params_.hybrid_astar.dx) + 1;
ind2 = ceil((y - params_.demo.ymin) / params_.hybrid_astar.dy) + 1;
if ((ind1 <= params_.hybrid_astar.num_nodes_x)&&(ind1 >= 1)&&(ind2 <= params_.hybrid_astar.num_nodes_y)&&(ind2 >= 1))
    return;
end
if (ind1 > params_.hybrid_astar.num_nodes_x)
    ind1 = params_.hybrid_astar.num_nodes_x;
elseif (ind1 < 1)
    ind1 = 1;
end
if (ind2 > params_.hybrid_astar.num_nodes_y)
    ind2 = params_.hybrid_astar.num_nodes_y;
elseif (ind2 < 1)
    ind2 = 1;
end
end
