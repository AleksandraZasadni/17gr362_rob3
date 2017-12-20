%Forward Dynamics for CrustCrawler%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, close all

%M, V and G are obtained from Inverse Dynamics
syms theta1 theta2 theta3 theta1_d theta2_d theta3_d
syms g

M11 = 0.0249399-0.0000286118*sin(2.0*theta2)-0.00309537*cos(2.0*(theta2 + theta3))-0.0215198*cos(theta2)^2-0.0000176105*sin(2.0*(theta2 + theta3))+0.0000323888*(sin(theta3)-sin(2.0*theta2 + theta3))+0.00883349*(cos(theta3)-cos(2.0*theta2 + theta3));
M12 = 0.000519874*cos(theta2)+0.000247161*cos(theta2 + theta3)+4.29566e-7*sin(theta2)+9.0626e-7*sin(theta2 + theta3);
M13 = 0.000247161*cos(theta2 + theta3)+9.0626e-7*sin(theta2 + theta3);
M21 = -0.000247161*(2.0*sin(0.5*theta2 + 0.5*theta3)^2 - 1.0)+4.29566e-7*sin(theta2)+9.0626e-7*sin(theta2 + theta3)+0.000519874*cos(theta2);
M22 = 0.0278138+0.017667*cos(theta3)+0.0000647775*sin(theta3);
M23 = 0.00626001+0.00883349*cos(theta3)+0.0000323888*sin(theta3);
M31 = 0.000247161*cos(theta2 + theta3)+9.0626e-7*sin(theta2 + theta3);
M32 = 0.00626001+0.00883349*cos(theta3)+0.0000323888*sin(theta3);
M33 = 0.00626001;
M = [M11 M12 M13; M21 M22 M23; M31 M32 M33];

V1 = theta2_d^2*(+4.29566e-7*cos(theta2)-0.000519874*sin(theta2))+theta2_d*theta3_d*(0.00000181252*cos(theta2+theta3)-0.000494322*sin(theta2+theta3))+theta1_d*theta2_d*((0.0215198*sin(2.0*theta2)-0.0000572236*cos(2.0*theta2))+(0.017667*sin(2.0*theta2+theta3)-0.0000647775*cos(2.0*theta2+theta3)))+theta1_d*theta3_d*((0.0000323888*(cos(theta3)-cos(2.0*theta2+theta3)))+(0.00883349*(sin(2.0*theta2+theta3)-sin(theta3))))+theta1_d*(theta2_d+theta3_d)*(0.00619074*sin(2.0*(theta2+theta3))-0.0000352211*cos(2.0*(theta2+theta3)))+(theta2_d^2+theta3_d^2)*(9.0626e-7*cos(theta2+theta3)-0.000247161*sin(theta2+theta3));
V2 = theta3_d^2*(0.0000323888*cos(theta3)-0.00883349*sin(theta3))+theta2_d*theta3_d*(0.0000647775*cos(theta3)-0.017667*sin(theta3))-theta1_d^2*(0.00309537*sin(2.0*theta2+2.0*theta3)+0.00883349*sin(2.0*theta2+theta3)+0.0000286118*(2.0*sin(theta2)^2-1.0)+0.0000176105*(2.0*sin(theta2+theta3)^2-1.0)+0.0107599*sin(2.0*theta2)+0.0000323888*(2.0*sin(theta2+0.5*theta3)^2-1.0));
V3 = theta2_d^2*(0.00883349*sin(theta3)-0.0000323888*cos(theta3))+theta1_d^2*(0.0000161944*(cos(2.0*theta2+theta3)-cos(theta3))+0.0000176105*cos(2.0*(theta2+theta3))+0.00441675*sin(theta3)-0.00441675*sin(2.0*theta2+theta3)-0.00309537*sin(2.0*(theta2+theta3)));
V = [V1; V2; V3];

G1 = 0;
G2 = -0.0401888*g*sin(theta2 + theta3)-0.10171*g*sin(theta2);
G3 = -0.0401888*g*sin(theta2 + theta3);
G = [G1; G2; G3];


Torque = sym('Torque', [3 1]);
thetaX_dd = inv(M)*(Torque-V-G);