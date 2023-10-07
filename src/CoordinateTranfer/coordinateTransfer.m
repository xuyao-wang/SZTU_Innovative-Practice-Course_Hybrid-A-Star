%璋冪敤coordinateTransfer.m鍑芥暟锛岃�＄畻杞﹁締鍧愭爣
function [x1,y1,x2,y2,x3,y3,x4,y4] = coordinateTransfer(x_next,y_next,theta_next,Lf,Lr,L,B)

%theta_next鎸囩殑鏄�杞﹁締鐨勬í鎽嗚��
%宸﹀墠杞�
x1 = x_next - B*sin(theta_next)*0.5 + (L+Lf)*cos(theta_next);
y1 = y_next + B*cos(theta_next)*0.5 + (L+Lf)*sin(theta_next);
%鍙冲墠杞�
x2 = x_next + B*sin(theta_next)*0.5 + (L+Lf)*cos(theta_next);
y2 = y_next - B*cos(theta_next)*0.5 + (L+Lf)*sin(theta_next);
%鍙冲悗杞�
x3 = x_next + B*sin(theta_next)*0.5 - Lr*cos(theta_next);
y3 = y_next - B*cos(theta_next)*0.5 - Lr*sin(theta_next);
%宸﹀悗杞�
x4 = x_next - B*sin(theta_next)*0.5 - Lr*cos(theta_next);
y4 = y_next + B*cos(theta_next)*0.5 - Lr*sin(theta_next);

end