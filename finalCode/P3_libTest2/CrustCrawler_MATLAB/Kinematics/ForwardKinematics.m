%Forward Kinematics for CrustCrawler%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%requirements: Robotics Toolbox (petercorke.com/wordpress/toolboxes/robotics-toolbox)
clear all, close all

syms theta1 theta2 theta3;
% theta1 = 
% theta2 = 
% theta3 = 

%Link Length
a1 = 0;
a2 = 0;
a3 = 0.2198;
%Link Twist
alpha1 = 0;
alpha2 = -pi/2;
alpha3 = 0;
%Link Offset
d1 = 0.0528;
d2 = -0.00315;
d3 = -0.003;
%Joint Angle
offset1 = 0;
offset2 = -pi/2;
offset3 = -pi/2;
%Tool
syms dtvar
% dtvar = 0.08047024; %0.08047024 - default
toolTranslX = -0.001675;
toolTranslZ = 0.1473+dtvar;
toolRotX = -pi/2;
toolRotZ = pi; %arbitrary

%Transformation Matrices
T01 = trotx(alpha1)*transl(a1,0,0)*trotz(theta1+offset1)*transl(0,0,d1);
T12 = trotx(alpha2)*transl(a2,0,0)*trotz(theta2+offset2)*transl(0,0,d2);
T23 = trotx(alpha3)*transl(a3,0,0)*trotz(theta3+offset3)*transl(0,0,d3);
Tool = trotx(toolRotX)*transl(toolTranslX,0,0)*trotz(toolRotZ)*transl(0,0,toolTranslZ);

%Forward Kinematics
T=vpa(T01*T12*T23*Tool)


%%SIMPLIFIED%%
% r11 = cos(theta2 + theta3) * cos(theta1)
% r12 = -sin(theta1)
% r13 = sin(theta2 + theta3) * cos(theta1)
% r21 = cos(theta2 + theta3) * sin(theta1)
% r22 = cos(theta1)
% r23 = sin(theta2 + theta3) * sin(theta1)
% r31 = -sin(theta2 + theta3)
% r32 = 0
% r33 = cos(theta2 + theta3)
% dx  = (0.1473 + dtvar)*sin(theta2 + theta3)*cos(theta1) + 0.2198*cos(theta1)*sin(theta2) + 0.00615*sin(theta1) + 0.001675*cos(theta2 + theta3)*cos(theta1)
% dy  = (0.1473 + dtvar)*sin(theta2 + theta3)*sin(theta1) + 0.2198*sin(theta1)*sin(theta2) - 0.00615*cos(theta1) + 0.001675*cos(theta2 + theta3)*sin(theta1)
% dz  = (0.1473 + dtvar)*cos(theta2 + theta3) + 0.2198*cos(theta2) - 0.001675*sin(theta2 + theta3) + 0.0528