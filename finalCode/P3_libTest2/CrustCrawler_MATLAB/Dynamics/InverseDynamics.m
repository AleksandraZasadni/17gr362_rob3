%Inverse Dynamics for CrustCrawler%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all, close all

displacement1 = 0;
displacement2 = 0;
displacement3 = 0;
velocity1     = 0;
velocity2     = 0;
velocity3     = 0;
acceleration1 = 0;
acceleration2 = 0;
acceleration3 = 0;

%Gravity
    g = 9.81;
    gravity = [0; 0; -g];

%Joints
    syms t1 t2 t3 t1_d t2_d t3_d
    syms t theta1(t) theta2(t) theta3(t) theta1_d(t) theta2_d(t) theta3_d(t) theta1_dd(t) theta2_dd(t) theta3_dd(t)

%Dynamic Properties:
    %BODY 1:
        m1 = 225.01148645*0.001; %kg
        Ixx1 = 149119.31955080*10^(-9);
        Ixy1 = 5.18406707*10^(-9);
        Ixz1 = 7.17204066*10^(-9);
        Iyx1 = 5.18406707*10^(-9);
        Iyy1 = 129174.94348645*10^(-9);
        Iyz1 = -2842.43371985*10^(-9);
        Izx1 = 7.17204066*10^(-9);
        Izy1 = -2842.43371985*10^(-9);
        Izz1 = 91824.74119788*10^(-9);
        Inertia1_local = [Ixx1, Ixy1, Ixz1; Iyx1, Iyy1, Iyz1; Izx1, Izy1, Izz1]; %kg*m^2
    %BODY 2:
        m2 = 212.85774246*0.001; %kg
        Ixx2 = 707153.55659730*10^(-9);
        Ixy2 = -40.08635980*10^(-9);
        Ixz2 = 8511.78812642*10^(-9);
        Iyx2 = -40.08635980*10^(-9);
        Iyy2 = 686135.52872223*10^(-9);
        Iyz2 = -3867.55733879*10^(-9);
        Izx2 = 8511.78812642*10^(-9);
        Izy2 = -3867.55733879*10^(-9);
        Izz2 = 54906.34272141*10^(-9);
        Inertia2_local = [Ixx2, Ixy2, Ixz2; Iyx2, Iyy2, Iyz2; Izx2, Izy2, Izz2]; %kg*m^2
    %BODY 3:
        m3 = 287.25035108*0.001; %kg
        Ixx3 = 732441.15568666*10^(-9);
        Ixy3 = -0.02314694*10^(-9);
        Ixz3 = -3005.78074024*10^(-9);
        Iyx3 = -0.02314694*10^(-9);
        Iyy3 = 637177.79489095*10^(-9);
        Iyz3 = -0.00263270*10^(-9);
        Izx3 = -3005.78074024*10^(-9);
        Izy3 = -0.00263270*10^(-9);
        Izz3 = 164382.84681068*10^(-9);
        Inertia3_local = [Ixx3, Ixy3, Ixz3; Iyx3, Iyy3, Iyz3; Izx3, Izy3, Izz3]; %kg*m^2

% heights for potential energy %m
    CoM1AoR2_home_position = [0; 0; 20.60030945]*0.001;
    AoR2CoM2_home_position = [0; 0; 181.21408288]*0.001;
    AoR2AoR3_home_position = [0; 0; 219.8]*0.001;
    AoR3CoM3_home_position = [0; 0; 139.90855040]*0.001;

    height_CoM1AoR2 = CoM1AoR2_home_position;
    height_AoR2CoM2 = cos(t2)*AoR2CoM2_home_position;
    height_AoR2AoR3 = cos(t2)*AoR2AoR3_home_position;
    height_AoR3CoM3 = cos(t2+t3)*AoR3CoM3_home_position;

% position vectors for kinetic energy %m
    P1C1_static_global = [0; 0; 32.19969055]*0.001;
    P12_static_global  = [0; -3.15; 52.8]*0.001;
    P2C2_static_global = [0; 0; 181.21408288]*0.001;
    P23_static_global  = [0; -3; 219.8]*0.001;
    P3C3_static_global = [0; 0; 139.90855040]*0.001;

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

%Substitute Specific Values
    torque1 = vpa(subs(torque1, [theta1(t) theta2(t) theta3(t) theta1_d(t) theta2_d(t) theta3_d(t) theta1_dd(t) theta2_dd(t) theta3_dd(t)], [displacement1 displacement2 displacement3 velocity1 velocity2 velocity3 acceleration1 acceleration2 acceleration3]), 5);
    torque2 = vpa(subs(torque2, [theta1(t) theta2(t) theta3(t) theta1_d(t) theta2_d(t) theta3_d(t) theta1_dd(t) theta2_dd(t) theta3_dd(t)], [displacement1 displacement2 displacement3 velocity1 velocity2 velocity3 acceleration1 acceleration2 acceleration3]), 5);
    torque3 = vpa(subs(torque3, [theta1(t) theta2(t) theta3(t) theta1_d(t) theta2_d(t) theta3_d(t) theta1_dd(t) theta2_dd(t) theta3_dd(t)], [displacement1 displacement2 displacement3 velocity1 velocity2 velocity3 acceleration1 acceleration2 acceleration3]), 5);

Torques = [torque1 torque2 torque3]