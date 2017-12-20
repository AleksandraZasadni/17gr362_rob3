%Inverse Dynamics for CrustCrawler (Symbolic)%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, close all

%Joints
    syms t1 t2 t3 t1_d t2_d t3_d
    syms t theta1(t) theta2(t) theta3(t) theta1_d(t) theta2_d(t) theta3_d(t) theta1_dd(t) theta2_dd(t) theta3_dd(t)

%Gravity
    syms g
    gravity = [0; 0; -g];

%Dynamic Properties:
    %BODY 1:
        syms m1 Ixx1 Ixy1 Ixz1 Iyx1 Iyy1 Iyz1 Izx1 Izy1 Izz1
        Inertia1_local = [Ixx1, Ixy1, Ixz1; Iyx1, Iyy1, Iyz1; Izx1, Izy1, Izz1];
    %BODY 2:
        syms m2 Ixx2 Ixy2 Ixz2 Iyx2 Iyy2 Iyz2 Izx2 Izy2 Izz2
        Inertia2_local = [Ixx2, Ixy2, Ixz2; Iyx2, Iyy2, Iyz2; Izx2, Izy2, Izz2];
    %BODY 3:
        syms m3 Ixx3 Ixy3 Ixz3 Iyx3 Iyy3 Iyz3 Izx3 Izy3 Izz3
        Inertia3_local = [Ixx3, Ixy3, Ixz3; Iyx3, Iyy3, Iyz3; Izx3, Izy3, Izz3];

% heights for potential energy
    CoM1AoR2_home_position = sym('CoM1AoR2_home_position', [3 1]);
    AoR2CoM2_home_position = sym('AoR2CoM2_home_position', [3 1]);
    AoR2AoR3_home_position = sym('AoR2AoR3_home_position', [3 1]);
    AoR3CoM3_home_position = sym('AoR3CoM3_home_position', [3 1]);

    height_CoM1AoR2 = CoM1AoR2_home_position;
    height_AoR2CoM2 = cos(t2)*AoR2CoM2_home_position;
    height_AoR2AoR3 = cos(t2)*AoR2AoR3_home_position;
    height_AoR3CoM3 = cos(t2+t3)*AoR3CoM3_home_position;

% position vectors for kinetic energy
    P1C1_static_global = sym('P1C1_static_global', [3 1]);
    P12_static_global  = sym('P12_static_global', [3 1]);
    P2C2_static_global = sym('P2C2_static_global', [3 1]);
    P23_static_global  = sym('P23_static_global', [3 1]);
    P3C3_static_global = sym('P3C3_static_global', [3 1]);

    P1C1 = rotz(t1)*P1C1_static_global;
    P12  = rotz(t1)*P12_static_global;
    P2C2 = rotz(t1)*roty(t2)*P2C2_static_global;
    P23  = rotz(t1)*roty(t2)*P23_static_global;
    P3C3 = rotz(t1)*roty(t2+t3)*P3C3_static_global;


%Angular velocities
    omega1wrt1 = [0; 0; t1_d];
    omega2wrt1 = [-t2_d*sin(t1); t2_d*cos(t1); t1_d]; %omega1wrt1+rotz(t1)*rotx(-pi/2)*[0; 0; t2_d]
    omega3wrt1 = [-sin(t1)*(t2_d+t3_d); cos(t1)*(t2_d+t3_d); t1_d]; %omega2wrt1+rotz(t1)*rotx(-pi/2)*[0; 0; t3_d];

    Inertia1_global = (rotz(t1))*Inertia1_local*transpose(rotz(t1));
    Inertia2_global = (rotz(t1)*roty(t2))*Inertia2_local*transpose(rotz(t1)*roty(t2));
    Inertia3_global = (rotz(t1)*roty(t2+t3))*Inertia3_local*transpose(rotz(t1)*roty(t2+t3));
    
%Linear velocities
    velocityC1wrt1 = cross(omega1wrt1, P1C1);
    velocity2wrt1  = cross(omega1wrt1, P12);
    velocityC2wrt1 = velocity2wrt1+cross(omega2wrt1, P2C2);
    velocity3wrt1  = velocity2wrt1+cross(omega2wrt1, P23);
    velocityC3wrt1 = velocity3wrt1+cross(omega3wrt1, P3C3);

    
%Potential energy
    p1 = 0;
    p2 = -m2*transpose(gravity)*(height_CoM1AoR2+height_AoR2CoM2);
    p3 = -m3*transpose(gravity)*(height_CoM1AoR2+height_AoR2AoR3+height_AoR3CoM3);

%Kinetic enery
    k1 = 1/2*m1*transpose(velocityC1wrt1)*velocityC1wrt1+1/2*transpose(omega1wrt1)*Inertia1_global*omega1wrt1;
    k2 = 1/2*m2*transpose(velocityC2wrt1)*velocityC2wrt1+1/2*transpose(omega2wrt1)*Inertia2_global*omega2wrt1;
    k3 = 1/2*m3*transpose(velocityC3wrt1)*velocityC3wrt1+1/2*transpose(omega3wrt1)*Inertia3_global*omega3wrt1;

%Lagrangian
    L = (k1+k2+k3)-(p1+p2+p3);

%Partial Derivatives
    displacementDerivative1 = diff(L, t1);
    displacementDerivative2 = diff(L, t2);
    displacementDerivative3 = diff(L, t3);
    velocityDerivative1     = diff(L, t1_d);
    velocityDerivative2     = diff(L, t2_d);
    velocityDerivative3     = diff(L, t3_d);

%Sustitution For Time Dependent Variables
    displacementDerivative1 = subs(displacementDerivative1, [t1 t2 t3 t1_d t2_d t3_d], [theta1 theta2 theta3 theta1_d theta2_d theta3_d]);
    displacementDerivative2 = subs(displacementDerivative2, [t1 t2 t3 t1_d t2_d t3_d], [theta1 theta2 theta3 theta1_d theta2_d theta3_d]);
    displacementDerivative3 = subs(displacementDerivative3, [t1 t2 t3 t1_d t2_d t3_d], [theta1 theta2 theta3 theta1_d theta2_d theta3_d]);
    velocityDerivative1     = subs(velocityDerivative1, [t1 t2 t3 t1_d t2_d t3_d], [theta1 theta2 theta3 theta1_d theta2_d theta3_d]);
    velocityDerivative2     = subs(velocityDerivative2, [t1 t2 t3 t1_d t2_d t3_d], [theta1 theta2 theta3 theta1_d theta2_d theta3_d]);
    velocityDerivative3     = subs(velocityDerivative3, [t1 t2 t3 t1_d t2_d t3_d], [theta1 theta2 theta3 theta1_d theta2_d theta3_d]);

%Torques
    torque1 = diff(velocityDerivative1, t)-displacementDerivative1;
    torque2 = diff(velocityDerivative2, t)-displacementDerivative2;
    torque3 = diff(velocityDerivative3, t)-displacementDerivative3;

%Substitution Of Time Derivatives
    torque1 = subs(torque1, [diff(theta1(t), t) diff(theta2(t), t) diff(theta3(t), t) diff(theta1_d(t), t) diff(theta2_d(t), t) diff(theta3_d(t), t)], [theta1_d theta2_d theta3_d theta1_dd theta2_dd theta3_dd]);
    torque2 = subs(torque2, [diff(theta1(t), t) diff(theta2(t), t) diff(theta3(t), t) diff(theta1_d(t), t) diff(theta2_d(t), t) diff(theta3_d(t), t)], [theta1_d theta2_d theta3_d theta1_dd theta2_dd theta3_dd]);
    torque3 = subs(torque3, [diff(theta1(t), t) diff(theta2(t), t) diff(theta3(t), t) diff(theta1_d(t), t) diff(theta2_d(t), t) diff(theta3_d(t), t)], [theta1_d theta2_d theta3_d theta1_dd theta2_dd theta3_dd]);

%Simplification
%     torque1simplified = simplify(torque1, 'Steps', 5000);
%     torque2simplified = simplify(torque2, 'Steps', 5000);
%     torque3simplified = simplify(torque3, 'Steps', 5000);

%Print Results To Files
%     torque1split = children(torque1);
%     torque2split = children(torque2);
%     torque3split = children(torque3);
%     torque1split = children(torque1simplified);
%     torque2split = children(torque2simplified);
%     torque3split = children(torque3simplified);
%     f1 = fopen('tau1Simplified.txt', 'wt');
%     fprintf(f1, '%s', char(vpa(torque1split, 6)));
%     fclose(f1);
%     f2 = fopen('tau2Simplified.txt', 'wt');
%     fprintf(f2, '%s', char(vpa(torque2split, 6)));
%     fclose(f2);
%     f3 = fopen('tau3Simplified.txt', 'wt');
%     fprintf(f3, '%s', char(vpa(torque3split, 6)));
%     fclose(f3);