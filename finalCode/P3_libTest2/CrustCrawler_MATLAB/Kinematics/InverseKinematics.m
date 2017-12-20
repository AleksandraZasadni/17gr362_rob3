%Inverse Kinematics for CrustCrawler%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, close all

syms x y z;
% x =
% y =
% z =

%Lenghts
a3 = 0.2198;
d1 = 0.0528;
d2 = 0.00315;
d3 = 0.003;
%Tool
variableLenght = 0.08047024; %0.08047024 - default
toolOffset = 0.1473+variableLenght;
toolLength = 0.001675;
toolStartEnd = sqrt(toolLength^2+toolOffset^2);

%Theta1
alpha = asin((d2+d3)/(sqrt(x^2+y^2)));
theta1Front = atan2(y, x)+alpha;
theta1Rear = atan2(y, x)-alpha+-pi;

%Theta2
projectedXY = sqrt(sqrt(x^2+y^2)^2-(d2+d3)^2);
projectedT1EE = sqrt(projectedXY^2+(z-d1)^2);
beta = atan2(z-d1, projectedXY);
gamma = acos((a3^2+projectedT1EE^2-toolStartEnd^2)/(2*a3*projectedT1EE));
theta2Up = pi/2-beta-gamma;
theta2Down = pi/2-beta+gamma;

%Theta3
delta = atan2(toolLength, toolOffset);
epsilon = acos((a3^2+toolStartEnd^2-projectedT1EE^2)/(2*a3*toolStartEnd));
theta3Up = pi-delta-epsilon;
theta3Down = -pi-delta+epsilon;

Solution1 = [theta1Front; theta2Up  ; theta3Up  ]
Solution2 = [theta1Front; theta2Down; theta3Down]
Solution3 = [theta1Rear; -theta2Up  ; theta3Down]
Solution4 = [theta1Rear; -theta2Down; theta3Up  ]