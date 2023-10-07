function [x_next,y_next,theta_next] = vehicleKineticModel(v,delta,x_init,y_init,theta_init,L,ts)
%vehicleKineticModel 通过运动学模型计算车辆下一时刻位置
%   此处显示详细说明


x_next = x_init + v*cos(theta_init)*ts;
y_next = y_init + v*sin(theta_init)*ts;
theta_next = theta_init + v*tan(delta)/L*ts;

end

