%% ---------------Automotive Air conditioner code------------------------%%
%%----------------HAN Minor Project-January 2020-------------------------%%
%%-----------This m-script produces all graph on single run--------------%% 

%% preparing work space
clear all
close all
clc
 
%% cabinspace
mr = 9000.50;  % mass of air inside cabin [g]
cpr = 1.008;   % specificheat at constant pressure at room in kJ/kg K
Tin  = 27;     % Intial Temperature of cabin space 

Cpe = 1.008;   % Constant pressure at evaprator in kJ/kg K
P = 1.2 ;      % density of air in kg/m³

Uo = 4;        % heat transfer co-efficient of vechile wall
Ao = 30;       % surface area of cabin
Ta = 21;       % Temperature of ambient air

Mf = 0.001 ;   % mass flow rate to cabin g/s 
Cpa =  1.008;  % Sp.heat at constant pressure at ambient point
v = 8;         % volume of cabin

Qs  = 50 ;     % heat load due to solar radiation
Qps = 70 ;     % sensible heat load per passengers
N = 1  ;       % no of passanger

Kf = 1.1 ;     % Heat gain co-efficient of fan
V = 0.115 ;    % volumetricair flow

%------------------------------------------%
Wm = 0.058;    % moisture content of the mixture air 
Ws = 0.009;    % moisture content of the supply air
Wa = 0.0158;   % Moisture content of the ambient Air
Wr = 0.0113;   % Moisture content of the air inside the vehicle


%% compressor parameters
n = 6;         % number of cyclinders in compressor
D = 0.14;      % diameter of the cylinder
sp = 0.49;     % relative piston stroke
sp1 = 10;      % piston stroke
Nc =11.50;     % speed of compressor (rps)
nv = (0.3596+(1.1072*sp)-(0.4025*sp*sp)+(0.0001175*Nc)-(2.449*1.0000e-08*Nc*Nc));

               % volumetric efficiency of compressor
vs = 0.087;    % Refrigerant specific volume
f1 = 0.281;    % flow rate1
f2 = 0.115;    % flow rate2
cpr1 = 1.436;  % Specific heat of Refrigirant
cpr2 = 0.955;  % Specific heat of Refrigirant
%% Evaporator parameters
Cp_E = 1.366;  % Specific heat of Refrigirant in Evaprator
hfg = .2450;   % Latent heat of vaporization of water in kJ/kg
Wm = 0.0113;   % moisture content of the mixture air
Wg = 0.009;    % moisture content of the mixture air
a1 = 0.125;    % Heat transfer coefficient between air and evaporator wall
a2 = 0.027;    % Heat transfer coefficient between air and evaporator wall
A1 = 0.53;     % Heat transfer area of dry – cooling region in m
A2 = 3.265;    % Heat transfer area of wet – cooling region in m
hr2 = .3938;   % Enthalpy of refrigerant at the evaporator core inlet
hr1 = .2335;   % Enthalpy of refrigerant at the evaporator core outlet
Vh1 = 0.004;   % Air side volume of the evaporator core
Vh2 = 0.016;   % Air side volume of the evaporator core

%% Thermal Expansion valve
Mfv = 0.01;    % mass flow of vapour  
Mfl = 0.001;   % mass flow of liquid 
cpv = 0.906;   % Specific heat of Refrigirant in vapour phase
cpl = 1.166;   % Specific heat of Refrigirant in liquid phase

%% condensor parameters
Cp_C = 1.147;  % Specific heat of Refrigirant in condensor
hfg = .2450;   % Latent heat of vaporization of water in kJ/kg
Wm = 0.0113;   % moiture content
Wg = 0.009;    % moiture content
a1 = 0.125;    % Heat transfer coefficient between air and evaporator wall
a2 = 0.027;    % Heat transfer coefficient between air and evaporator wall
A1 = 2.83;     % Heat transfer area of dry – cooling region in m
A2 = 0.55;     % Heat transfer area of wet – cooling region in m
hr2 = .4288;   % Enthalpy of refrigerant at the evaporator core inlet
hr1 = .2335;   % Enthalpy of refrigerant at the evaporator core outlet
Vh1 = 0.004;   % Air side volume of the evaporator core
Vh2 = 0.016;   % Air side volume of the evaporator core  


%% Connecting model
sim_time = 1000;
sim('AC_Vfinal')

%% plotting graph results
%% Ploting results Cabin
figure('Name','Output of the System')               % Naming The figure
set(gcf,'color','cyan')
%-----------------------plot for Cabin Temperature------------------------%
plot(T(:,1), T(:,2),':b','linewidth',3)             % DE of of temp
hold on
plot(T_tf1(:,1), T_tf1(:,2),'-r','linewidth',1)   
xlabel('Time [sec]')                                % Labeling axis
ylabel('Temperature [°C]')                       
ylim([21 28])                                       % setting axis limit
grid on
title('Output graph of cabin temperature')
legend('De','Tf')

%% Ploting results Evaporator
figure('Name','Output of the evaporator')           % Naming The figure
set(gcf,'color','cyan')
%------------------plot for Evaporator temperature------------------------%
plot(Evp(:,1), Evp(:,2),':b','linewidth',3)         % DE of of temp
hold on
plot(Evp_tf(:,1),Evp_tf(:,2),'-r','linewidth',1)
xlabel('Time [sec]')                                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Output graph of refrigirant temperature in evaporator')
legend('De','Tf')

%% Ploting results Compressor
figure('Name','Output of the compressor temperature')  % Naming The figure
set(gcf,'color','cyan')
%----------------------plot for Compressor Temperature--------------------%
plot(Comp(:,1), Comp(:,2),':b','linewidth',3)          % DE of temp
hold on
plot(Comp_tf(:,1),Comp_tf(:,2),'-r','linewidth',1)
xlabel('Time [sec]')                                   % Labeling axis
ylabel('Temperature [°C]')
ylim([0 70])
grid on
title('Output graph of refrigirant temperature in compressor')
legend('De','Tf')


%% Ploting results Condenser
figure('Name','Output of the Condenser')            % Naming The figure
set(gcf,'color','cyan')
%-------------------plot for Condensor Temperature------------------------%
plot(Con(:,1), Con(:,2),':b','linewidth',3)         % DE of temp
hold on
plot(Con_tf(:,1),Con_tf(:,2),'-r','linewidth',1)    % Tf of Temp
xlabel('Time [sec]')                                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Output graph of refrigirant temperature in condenser')
legend('De','Tf')

%% Ploting results TEV
figure('Name','Output of the TEV')                  % Naming The figure

set(gcf,'color','cyan')
%-------------------------plot for Tev Temperature------------------------%
plot(Tev(:,1), Tev(:,2),':b','linewidth',3)         % DE of Temp
hold on
plot(Tev_tf(:,1), Tev_tf(:,2),'-r','linewidth',1)   %Tf of Temp
xlabel('Time [sec]')                                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Output graph of refrigirant temperature in TEV')
legend('De','Tf')

%% Sensitivity analysis
%-----------------------system response-analysis--------------------------%
%---For differrent cabin space i.e with different quantity of air --------%
for mr = [6000 9000 12000]
sim_time = 1000;
sim('AC_Vfinal')
figure(6)                                           % Naming The figure
set(gcf,'color','cyan')
%----------------------plot for Cabin Temperature-------------------------%
% nexttile
plot(T(:,1), T(:,2))                                % DE of temp
hold on   
xlabel('Time [sec]')                                % Labeling axis
ylabel('Temperature [°C]')
ylim([21 28])
grid on
title('Sensitivity analysis with different cabin space')
end
figure(6)
Legend=cell(3,1); 
 Legend{1}=' Hatchback';
 Legend{2}=' Sedan';
 Legend{3}=' SUV';
legend(Legend);
% resetting the value to intial condition
mr = 9000;
%-------------------------------------------------------------------------%
%%
%---------- Response for different no of passenger inside the car---------%
for N = [1 2 3]
sim_time = 1000;
sim('AC_Vfinal')
figure(7)                                         % Naming The figure
set(gcf,'color','cyan')
%----------------------plot for Cabin Temperature-------------------------%
plot(T(:,1), T(:,2))                              % DE of of temp
hold on
xlabel('Time [sec]')                              % Labeling axis
ylabel('Temperature [°C]')
grid on
ylim([21 28])
title('Sensitivity analysis with number of passengers')
end
figure(7)
Legend=cell(3,1); 
 Legend{1}=' 1 passenger';
 Legend{2}=' 2 passengers';
 Legend{3}=' 3 passengers';
legend(Legend);
% resetting the value
N = 1;

%-------------------Components response-analysis--------------------------%
%---------------------For different power input---------------------------% 
%% 
for Nc = [8.6 11.75 16.00]
sim_time = 1000;
sim('AC_Vfinal')
figure(8)                           % Naming The figure
% subplot(2,2,1)
plot(Evp(:,1), Evp(:,2))            % DE of  temp
hold on
xlabel('Time [sec]')                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Comparison of refrigirant temperature in evaporator')
figure(9)
plot(Con(:,1), Con(:,2))            % DE of temp
hold on
xlabel('Time [sec]')                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Comparison of refrigirant temperature in condensor')
figure(10)
plot(Comp(:,1), Comp(:,2))          % DE of temp
hold on
xlabel('Time [sec]')                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Comparison of refrigirant temperatures in compressor')
figure(11)
plot(Tev(:,1), Tev(:,2))            % DE of Temp
hold on
xlabel('Time [sec]')                % Labeling axis
ylabel('Temperature [°C]')
grid on
title('Comparison of refrigirant temperatures in TEV')
end
figure(8)
Legend=cell(3,1); 
 Legend{1}=' Lower power input';
 Legend{2}=' Given power input';
 Legend{3}=' Higher Power input';
legend(Legend);
figure(9)
Legend=cell(3,1); 
 Legend{1}=' Lower power input';
 Legend{2}=' Given power input';
 Legend{3}=' Higher Power input';
legend(Legend);
figure(10)
Legend=cell(3,1); 
 Legend{1}=' Lower power input';
 Legend{2}=' Given power input';
 Legend{3}=' Higher Power input';
legend(Legend);
figure(11)
Legend=cell(3,1); 
 Legend{1}=' Lower power input';
 Legend{2}=' Given power input';
 Legend{3}=' Higher Power input';
legend(Legend);
Nc = 11.75;

%--------------------------presenting graphs------------------------------%

 figure(1)
 movegui('northwest')
 figure(2)
 movegui('west')
 figure(3)
 movegui('southwest')
 figure(4)
 movegui('north')
 figure(5)
 movegui('center')
 figure(6)
 movegui('south')
 figure(7)
 movegui('northeast')
 figure(8)
 movegui('east')
 figure(9)
 movegui('southeast')
 figure(10)
 movegui('southeast') 
 figure(11)
 movegui('southeast')
