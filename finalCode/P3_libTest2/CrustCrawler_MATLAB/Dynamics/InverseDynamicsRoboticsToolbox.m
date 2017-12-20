%Robotics Toolbox simulation of CrustCrawler Inverse Dynamics%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%requirements: Robotics Toolbox (petercorke.com/wordpress/toolboxes/robotics-toolbox)
clear all, close all

% syms theta1 theta2 theta3 theta1_d theta2_d theta3_d theta1_dd theta2_dd theta3_dd
theta1    = 0;
theta2    = 0;
theta3    = 0;
theta1_d  = 0;
theta2_d  = 0;
theta3_d  = 0;
theta1_dd = 0;
theta2_dd = 0;
theta3_dd = 0;

%Kinematic Properties:
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
        variableLenght = 0.08047024; %0.08047024 - default
        toolTranslX = -0.001675;
        toolTranslZ = 0.1473+variableLenght;
        toolRotX = -pi/2;
        toolRotZ = pi; %arbitrary
        Tool = trotx(toolRotX)*transl(toolTranslX,0,0)*trotz(toolRotZ)*transl(0,0,toolTranslZ);


%Dynamic Properties:
    %BODY 1:
        P1C1_local = [0; 0; 32.19969055]*0.001; %m, local
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
        Inertia1 = [Ixx1, Ixy1, Ixz1; Iyx1, Iyy1, Iyz1; Izx1, Izy1, Izz1]; %kg*m^2, local
    %BODY 2:
        P2C2_local = [181.21408288; 0; 0]*0.001; %m, local
        m2 = 212.85774246*0.001; %kg
        Ixx2 = 54906.34272141*10^(-9);
        Ixy2 = 8511.78812642*10^(-9);
        Ixz2 = -3867.55733879*10^(-9);
        Iyx2 = 8511.78812642*10^(-9);
        Iyy2 = 707153.55659730*10^(-9);
        Iyz2 = -40.08635980*10^(-9);
        Izx2 = -3867.55733879*10^(-9);
        Izy2 = -40.08635980*10^(-9);
        Izz2 = 686135.52872223*10^(-9);
        Inertia2 = [Ixx2, Ixy2, Ixz2; Iyx2, Iyy2, Iyz2; Izx2, Izy2, Izz2]; %kg*m^2, local
    %BODY 3:
        P3C3_local = [0; 139.90855040; 0]*0.001; %m, local
        m3 = 287.25035108*0.001; %kg
        Ixx3 = 732441.15568666*10^(-9);
        Ixy3 = 3005.78074024*10^(-9);
        Ixz3 = 0.02314694*10^(-9);
        Iyx3 = 3005.78074024*10^(-9);
        Iyy3 = 164382.84681068*10^(-9);
        Iyz3 = -0.00263270*10^(-9);
        Izx3 = 0.02314694*10^(-9);
        Izy3 = -0.00263270*10^(-9);
        Izz3 = 637177.79489095*10^(-9);
        Inertia3 = [Ixx3, Ixy3, Ixz3; Iyx3, Iyy3, Iyz3; Izx3, Izy3, Izz3]; %kg*m^2, local


%Links
    L(1) = Link('offset', offset1, 'alpha', alpha1, 'a', a1, 'd', d1, 'modified');
    L(2) = Link('offset', offset2, 'alpha', alpha2, 'a', a2, 'd', d2, 'modified');
    L(3) = Link('offset', offset3, 'alpha', alpha3, 'a', a3, 'd', d3, 'modified');
%CoM Translation Vector
    L(1).r = P1C1_local;
    L(2).r = P2C2_local;
    L(3).r = P3C3_local;
%Mass
    L(1).m = m1;
    L(2).m = m2;
    L(3).m = m3;
%Inertia
    L(1).I = Inertia1;
    L(2).I = Inertia2;
    L(3).I = Inertia3;

Robot = SerialLink(L, 'name', 'CrustCrawler', 'tool', Tool);

displacement = [theta1    theta2    theta3];
velocity     = [theta1_d  theta2_d  theta3_d];
acceleration = [theta1_dd theta2_dd theta3_dd];

torque = vpa(Robot.rne(displacement, velocity, acceleration))