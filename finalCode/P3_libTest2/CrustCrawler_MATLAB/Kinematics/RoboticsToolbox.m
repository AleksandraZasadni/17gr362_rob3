%Robotics Toolbox simulation of CrustCrawler%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%requirements: Robotics Toolbox (petercorke.com/wordpress/toolboxes/robotics-toolbox)
clear all, close all

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


%Links
K(1) = Link('offset', offset1, 'alpha', alpha1,'a', a1, 'd', d1, 'modified');
K(2) = Link('offset', offset2, 'alpha', alpha2,'a', a2, 'd', d2, 'modified');
K(3) = Link('offset', offset3, 'alpha', alpha3,'a', a3, 'd', d3, 'modified');

Robot = SerialLink(K, 'name', 'CrustCrawler', 'tool', Tool);
Robot.plotopt = {'workspace' [-0.25, 0.25, -0.25, 0.25, 0, 0.75]};
Robot.teach('eul');
set(gcf, 'Position', get(0, 'Screensize'));