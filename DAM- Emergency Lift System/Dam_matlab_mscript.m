

%========This Matlab file is code by Aravind Ayyamperumal Kumar===========%
%--This is code for control system of dam to work on emegency situations--%
%--This is coded for practice modeling and simulation assingment----------%
%--The output on command windows have been supressed, the required results
%can be viwed but removing the semicolon by end of line-------------------%
%--This code produces all graphs mentioned the report in 8 figures--------%
%The user can change the input paramters from conditional comments section% 

%% Preparing the workspace
close all
clear all
clc

%% Defining the parameters and inputs
%---------------Motor Parameters-------------------%
%Electrical constants
R  = 0.025;     % Resistance in [N/m]
L  = 0.00023;   % Inductance in [H]
Kb = 0.33;      % Back emf constant
%Mechanical constants
J  = 0.0005;    % Inertia 
D  = 1.265;     % damping coeffficient [Ns/m]
Kt = 2.47;      % torque Constant
%gearset and pulley
rp = 0.375;     % Radius of pulley [m]
N2 = 1250;      % No of teeth in N2
N1 = 125;       % No of teeth in N1
%---------------gate parameters--------------------%
M = 3800;       % mass of the gate   [Kg]
lm = 2;         % Length of the gate [m]
wm = 0.3;       % Width of the gate  [m]
hm = 12;        % Height of the gate [m]
%-------------- reservoir parameters---------------%
lr = 500;       % Length of the reservoir [m]
wr = 20;        % Width of the reservoir  [m]
hr = 20;        % Height of the reservoir [m]
vw = 75;        % Exit velocity of water  [m/sec]
Ar = lr*wr;     % Area of the reservoir [m2]
%-------------- Other Constants--------------------%
g = 9.81;       % Gravitational Constant

%-------------- conditional commenents-------------%
Step_input = 400;           % Step input
step_time = 0.1;            % Step time
sim_time =  50;             % Simulation time  
slope = 4550;                % slope for ramp input
w = Simulink.Parameter(1);  % Actual given input 


%% DC Motor
%%state space of motor
A_mat_m =  [-D/J   Kt/J                             %defining  matrices
            -Kb/L   -R/L];                         
B_mat_m = [0
           1/L];
C_mat_m = [0  Kt];
D_mat_m = 0;

motor_ss = ss(A_mat_m,B_mat_m,C_mat_m,D_mat_m);     %making ss

%%Transfer Funtion OF DC motor
[num,den] = ss2tf(A_mat_m,B_mat_m,C_mat_m,D_mat_m); 
motor_tf = tf(num,den);                             %making tf

%% Lift Of Gate
%%state space of the gate 
P = ((N2)/(N1*rp*M));   
A_mat_d = [0   1                                    %defining  matrices
           0   0];
B_mat_d = [0  0
           P -1];
C_mat_d = [1 0];
D_mat_d = [0 0];          
door_ss = ss(A_mat_d,B_mat_d,C_mat_d,D_mat_d);      %making ss

%%Transfer Funtion OF the gate
[num1, den1] = ss2tf(A_mat_d,B_mat_d,C_mat_d,D_mat_d,1);  % in = 1
[num2, den2] = ss2tf(A_mat_d,B_mat_d,C_mat_d,D_mat_d,2);  % in = 2
doortf1 = tf(num1, den1);  %tf of first input
doortf2 = tf(num2, den2);  %tf of Second input

%% Water Level
%%state space of Waterlevel
Q = (lm*vw)/(lr*wr);
A_mat_wl = [0];                                      %defining  matrices
B_mat_wl = [Q];
C_mat_wl = [1];
D_mat_wl = [0];

Wlevel_ss = ss(A_mat_wl,B_mat_wl,C_mat_wl,D_mat_wl); %making ss

%%Transfer Funtion of Waterlevel
[num3,den3] = ss2tf(A_mat_wl,B_mat_wl,C_mat_wl,D_mat_wl);
wlevel_tf = tf(num3,den3);                           %making tf

%% Starting the simulator
sim('Damsim');

%% Ploting results for actual prameters
figure('Name','Output of the system')               % Naming The figure
tiledlayout(2,2)                                    % Defining the blocks
set(gcf,'color','cyan')
%-------------------------plot for lift of gate---------------------------%
nexttile
plot(glift(:,1),glift(:,2),'-b','linewidth',2)      % DE of lift
hold on
plot(sslift(:,1),sslift(:,2),'--g','linewidth',2)   % SS of Lift
hold on
plot(tflift(:,1),tflift(:,2),':r','linewidth',2)    % TF of Lift
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Lift of Gate [m]')
grid on
title('Output graph of Gate lift')
legend('De','ss','tf')

%-------------------------plot for Water level----------------------------%
nexttile
plot(gwl(:,1),gwl(:,2),'-b','linewidth',2)          % DE of Water Level 
hold on
plot(sswl(:,1),sswl(:,2),'--g','linewidth',2)       % SS of Water Level
hold on
plot(tfwl(:,1),tfwl(:,2),':r','linewidth',2)        % TF of Water Level
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level')                
legend('De','ss','tf')

%------------------------plot for Torque vs current-----------------------%
nexttile
plot(tm(:,2),i(:,2),'g')                       % Plot for Torque vs Current
xlabel('Torque (N/m)')                         % Labeling axis
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
legend('Torque vs current')

%------------------------plot for input-----------------------------------%
nexttile
plot(in(:,1),in(:,2),'g')                      % Plot for Input
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Input Signal [V]')
title('Input graph of voltage')
grid on
legend('Input Signal')
movegui('northwest');

%% ploting graph with Validation data
%------------------------Import data and read data------------------------%
Data=xlsread('InputOutputData'); 
time=Data(:,1);                      %Time vector   [s]
input=Data(:,2);                     %Input voltage [v]
output1=Data(:,3);                   %Output displacement of gate[m]
output2=Data(:,4);                   %Output displacement of waterlevel[m]

%%Plot results with given data
figure('Name','Output Graphs with test result data')% Naming The figure 
tiledlayout(2,2)                                    % Defining the blocks
%-------------------------plot for lift of gate---------------------------%
nexttile
plot(glift(:,1),glift(:,2),'-b','linewidth',2)      % DE of lift
hold on
plot(sslift(:,1),sslift(:,2),'--g','linewidth',2)   % SS of Lift
hold on
plot(tflift(:,1),tflift(:,2),':r','linewidth',2)    % TF of Lift
hold on
plot(time,output1,':c')                             % validation data plot
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Lift of Gate [m]')
grid on
title('Output graph of Gate lift')
legend('De','ss','tf','Given data')

%-------------------------plot for Water level----------------------------%
nexttile
plot(gwl(:,1),gwl(:,2),'-b','linewidth',2)          % DE of Water Level 
hold on
plot(sswl(:,1),sswl(:,2),'--g','linewidth',2)       % SS of Water Level
hold on
plot(tfwl(:,1),tfwl(:,2),':r','linewidth',2)        % TF of Water Level
hold on
plot(time,output2,':c')                             % validation data plot
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level')                
legend('De','ss','tf','Given data')

%------------------------plot for Torque vs current-----------------------%
nexttile
plot(tm(:,2),i(:,2),'g')                       % Plot for Torque vs Current
xlabel('Torque (N/m)')                         % Labeling axis
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
legend('Torque vs current')

%------------------------plot for input-----------------------------------%
nexttile
plot(in(:,1),in(:,2),'g')                      % Plot for Input
hold on
plot(time,input,'r')                           % validation data plot
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Input Signal [V]')
title('Input graph of voltage')
grid on
legend('Input Signal','given data')
movegui('southwest');

%% Plot results with given data and given data input
w = Simulink.Parameter(3);                              % given data input
sim_time = 5; 
%%Starting the simulator
sim('Damsim');
%%ploting the rseults
figure('Name','Output Graphs with input from data') % Naming The figure 
tiledlayout(2,2)                                    % Defining the blocks
%-------------------------plot for lift of gate---------------------------%
nexttile
plot(glift(:,1),glift(:,2),'-b','linewidth',2)      % DE of lift
hold on
plot(sslift(:,1),sslift(:,2),'--g','linewidth',2)   % SS of Lift
hold on
plot(tflift(:,1),tflift(:,2),':r','linewidth',2)    % TF of Lift
hold on
plot(time,output1,':c')                             % validation data plot
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Lift of Gate [m]')
grid on
title('Output graph of Gate lift')
legend('De','ss','tf','Given data')

%-------------------------plot for Water level----------------------------%
nexttile
plot(gwl(:,1),gwl(:,2),'-b','linewidth',2)          % DE of Water Level 
hold on
plot(sswl(:,1),sswl(:,2),'--g','linewidth',2)       % SS of Water Level
hold on
plot(tfwl(:,1),tfwl(:,2),':r','linewidth',2)        % TF of Water Level
hold on
plot(time,output2,':c')                             % validation data plot
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level')                
legend('De','ss','tf','Given data')

%------------------------plot for Torque vs current-----------------------%
nexttile
plot(tm(:,2),i(:,2),'g')                       % Plot for Torque vs Current
xlabel('Torque (N/m)')                         % Labeling axis
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
legend('Torque vs current')

%------------------------plot for input-----------------------------------%
nexttile
plot(tin(:,1),tin(:,2),'g')                    % Plot for Input
hold on
plot(time,input,':r')                          % validation data plot
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Input Signal [V]')
title('Input graph of voltage')
grid on
legend('Input Signal','given data')
movegui('west');


%% Sensitive Analysis

w = Simulink.Parameter(1);                     % Actual given input
sim_time =  50;

%------------------------For varying mass---------------------------------%
for M = [3420 3800 4180]
%%Start the simulator
sim('Damsim');
%Ploting the De with varying mass
figure(4)                                      % Naming The figure
%-----------------------plot for lift of gate-----------------------------%
subplot(2,2,1)
plot(glift(:,1),glift(:,2))                    % DE of lift 
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Lift of Gate [m]')
grid on
title('output graph of Gate lift with Diffrent mass')
%-----------------------plot for Water level------------------------------%
subplot(2,2,2)
plot(gwl(:,1),gwl(:,2))                        % DE of Water Level 
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level with Diffrent mass')
%-----------------------plot for Torque vs current------------------------%
subplot(2,2,3)
plot(tm(:,2),i(:,2),'g')
xlabel('Torque (N/m)')
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
legend('Torque vs Current')
%------------------------plot for input-----------------------------------%
subplot(2,2,4)
plot(in(:,1),in(:,2),'g')                      % Plot for Input
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')
ylabel('Input Signal [V]')
title('input graph of voltage')
grid on
legend('Input Signal')
hold on
end
%------------------------giving legends-----------------------------------%
subplot(2,2,1)
Legend=cell(3,1); 
 Legend{1}=' M = 3420';
 Legend{2}=' M = 3800';
 Legend{3}=' M = 4180';
legend(Legend);

subplot(2,2,2)
 Legend=cell(3,1); 
 Legend{1}=' M = 3420';
 Legend{2}=' M = 3800';
 Legend{3}=' M = 4180';
legend(Legend);
movegui('northeast');

%------------------------For varying torque constant----------------------%
for Kt = [2.223 2.717 2.47]
M = 3800;                                      %re-intialising mass value
%%Start the simulator
sim('Damsim');
%Ploting the De with varying torque constant
figure(5)    % Naming The figure

%----------------------plot for lift of gate------------------------------%
subplot(2,2,1)
plot(glift(:,1),glift(:,2))                    % DE of lift 
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Lift of gate [m]')
grid on
title('output graph of gate lift With different Kt')
%------------------plot for Water level-----------------------------------%
subplot(2,2,2)
plot(gwl(:,1),gwl(:,2))                        % DE of Water Level 
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level With different Kt')
hold on
%------------------plot for Torque vs current-----------------------------%
subplot(2,2,3)
plot(tm(:,2),i(:,2))
xlabel('Torque (N/m)')
ylabel('Current [A]')
title('output graph of Torque vs current With different Kt')
grid on
hold on
%------------------plot for input-----------------------------------------%
subplot(2,2,4)
plot(in(:,1),in(:,2),'g')
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')
ylabel('Input Signal [V]')
title('input graph of voltage')
grid on
legend('Input Signal')

end
%------------------------giving legends-----------------------------------%
subplot(2,2,1)
Legend=cell(3,1); 
 Legend{1}=' kt = 2.223';
 Legend{2}=' kt = 2.717';
 Legend{3}=' kt = 2.47';
legend(Legend);

subplot(2,2,2)
Legend=cell(3,1); 
 Legend{1}=' kt = 2.223';
 Legend{2}=' kt = 2.717';
 Legend{3}=' kt = 2.47';
legend(Legend);

subplot(2,2,3)
Legend=cell(3,1); 
 Legend{1}=' kt = 2.223';
 Legend{2}=' kt = 2.717';
 Legend{3}=' kt = 2.47';
legend(Legend);
movegui('southeast');

%------------------------For varying Resviour Area------------------------%
for Ar = [9000 11000 10000]

%%Start the simulator
sim('Damsim');
figure(6)                                      % Naming The figure
                            

%-------------------plot for lift of gate---------------------------------%
subplot(2,2,1)
plot(glift(:,1),glift(:,2))                    % DE of lift
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Lift of Gate [m]')
grid on
title('output graph of Gate lift with different Area')
% legend('Lift of gate');
%------------------plot for Water level-----------------------------------%
subplot(2,2,2)
plot(gwl(:,1),gwl(:,2))                        % DE of Water Level
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level with different Area')
hold on
%------------------plot for Torque vs current-----------------------------%
subplot(2,2,3)
plot(tm(:,2),i(:,2))
xlabel('Torque (N/m)')
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
%------------------plot for input-----------------------------------------%
subplot(2,2,4)
plot(in(:,1),in(:,2),'g')
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')
ylabel('Input Signal [V]')
title('input graph of voltage')
grid on
end
%------------------------giving legends-----------------------------------%
% subplot(2,2,1)
% Legend=cell(3,1); 
%  Legend{1}='Lift of gate';
% legend(Legend);
% 
% 
% subplot(2,2,2)
%  Legend=cell(3,1); 
%  Legend{1}=' Ar = 9000';
%  Legend{2}=' Ar = 11000';
%  Legend{3}=' Ar = 10000';
% legend(Legend);
% 
% subplot(2,2,3)
% Legend=cell(1,1); 
%  Legend{1}='Torque vs current';
% legend(Legend);
% 
% subplot(2,2,4)
%  Legend=cell(1,1); 
%  Legend{1}='Input Signal';
% legend(Legend);
% movegui('east');

%% simulating model with Own Ramp input
w = Simulink.Parameter(2);                     % ramp input
%%Start the simulator
sim('Damsim');
%Ploting the De with Ramp input
figure('Name','output with Ramp (own input)')  % Naming The figure])
subplot(2,2,1)
plot(glift(:,1),glift(:,2))                    % DE of Water Level 
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level with Ramp input')
legend('Water level')
%-----------------------plot for Water level------------------------------%
subplot(2,2,2)
plot(gwl(:,1),gwl(:,2))                        % DE of Water Level 
hold on
xlim([-0.1 sim_time])                          % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level with Ramp input')
legend('Water level')
%-----------------------plot for Torque vs current------------------------%
subplot(2,2,3)
plot(tm(:,2),i(:,2),'g')
xlabel('Torque (N/m)')
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
legend('Torque vs Current')
%-----------------------plot for input------------------------------------%
subplot(2,2,4)
plot(in2(:,1),in2(:,2),'g')                    % Plot for Input
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')
ylabel('Input Signal [V]')
title('input graph of voltage')
grid on
legend('Input Signal')
hold on
movegui('south');

Simulink.Parameter(1);           % re-instalising the previous input

%% Bonus Question

%---------------Motor Parameters-------------------%
%Electrical constants
R  = 0.00025;   % Resistance in [N/m]
L  = 0.0023;    % Inductance in [H]
Kb = 0.23;      % Back emf constant
%Mechanical constants
J  = 0.005;     % Inertia 
D  = 1.565;     % damping coeffficient [Ns/m]
Kt = 2.50;      % torque Constant
Step_input = 230; 
w = Simulink.Parameter(1);  % Actual given input
%%Starting the simulator
sim('Damsim');
%ploting results
figure('Name','Bonus question graph')               % Naming The figure
tiledlayout(2,2)                                    % Defining the blocks
%-------------------------plot for lift of gate---------------------------%
nexttile
plot(glift(:,1),glift(:,2),'-.b','linewidth',2)     % DE of lift
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.6])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Lift of Gate [m]')
grid on
title('Output graph of Gate lift')
legend('gate lift')

%-------------------------plot for Water level----------------------------%
nexttile
plot(gwl(:,1),gwl(:,2),'-.b','linewidth',2)         % DE of Water Level 
hold on
xlim([-0.1 sim_time])                               % setting axis limit
ylim([-0.1 1.2])
xlabel('Time [sec]')                                % Labeling axis
ylabel('Water level [m]')
grid on
title('output graph of water level')                
legend('waterlevel')

%------------------------plot for Torque vs current-----------------------%
nexttile
plot(tm(:,2),i(:,2),'g')                       % Plot for Torque vs Current
xlabel('Torque (N/m)')                         % Labeling axis
ylabel('Current [A]')
title('output graph of Torque vs current')
grid on
legend('Torque vs current')

%------------------------plot for input-----------------------------------%
nexttile
plot(in(:,1),in(:,2),'g')                      % Plot for Input
hold on
xlim([-3 sim_time])                            % setting axis limit
ylim([-20 450])
xlabel('Time [sec]')                           % Labeling axis
ylabel('Input Signal [V]')
title('Input graph of voltage')
grid on
legend('Input Signal')
movegui('north');
%---------------------Restoring to actual values--------------------------%
%Electrical constants
R  = 0.025;     % Resistance in [N/m]
L  = 0.00023;   % Inductance in [H]
Kb = 0.33;      % Back emf constant
%Mechanical constants
J  = 0.0005;    % Inertia 
D  = 1.265;     % damping coeffficient [Ns/m]
Kt = 2.47;      % torque Constant
Step_input = 400; 