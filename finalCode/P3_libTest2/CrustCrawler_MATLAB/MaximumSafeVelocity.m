%Calculation of the Maximum TCP Velocity for the CrustCrawler%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all, clear all

% PARAMETERS:
% Maximum contact pressure for specific body area (Table A.2)
p_max=110; %N*cm^(-2)
% Effective mass of the human body region (Table A.3)
m_H=4.4; %kg
% Effective spring constant for specific body region (Table A.3)
k=75; %N*mm^(-1)
% Total mass m of the M moving parts of the robot (CAD model)
M=0.21286+0.28598; %kg
% Area of contact between robot and body region
A=22; %mm^2
% Effective payload of the robot system, including tooling and workpiece (load)
m_L=0; %kg


% UNIT CONVERSIONS:
k=k*1e3; % to N*m^(-1)
p_max=p_max*1e4; % to N*m^(-2)
A=A*1e-6; % to m^2

% CALCULATIONS:
% Effective mass of the robot as a function of robot posture and motion (Figure A.3, Equation A.4)
m_R=M/2+m_L; %kg
% Reduced mass of the two-body system (Equation A.3)
mu=inv(1/m_H+1/m_R);

% Maximum relative velocity (Equation A.6)
v_relmax=vpa((p_max*A)/(sqrt(mu*k))*1000)