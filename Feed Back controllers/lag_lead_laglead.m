%% Prepare the workspace
clear 
clc
close all


%% ---------------------lead compensator ------------------------------
%% defining the system
% G = zpk([],[0 -4 -6],1)
% % rltool(G)
% % figure(1)
% % rlocus(G)
% z = -log(0.30)/(sqrt((pi*pi)+(log(0.30)*log(0.30)))) %Z=damping ratio
% % sgrid(z,0)
% %----------------------------------
% Gu = zpk([],[0 -4 -6],63)
% Ts = 4/1.005 %setteling time(real part)
% NTs = Ts/2 %threefold reduction in the settling time
% sigma = 4/(NTs)
% the = 180-(atand((sqrt(1-(z*z)))/z))
% wd = sigma*(tan(deg2rad(180-the)))
% s = -sigma + wd*i
% %----------------------------------
% Gu1 = (s+5)/(s*(s+4)*(s+6)); %add the required zero and find angle
% Theta=((180/pi)*angle(Gu1))
% pole = sigma+(wd/tan(deg2rad(180+Theta)))
% % lead = zpk([-5],[0 -4 -6 -pole],1423)
% 
% Gce = zpk([-5],[0 -4 -6 -pole],1423)
% Tu=feedback(Gu,1);
% Tc=feedback(Gce,1);
% figure(1)
% step(Tu)
% hold on
% step(Tc)
% grid on
% legend('un-compensated','Lead compensated')
% title('Lead controller')

% 
% %% --------------------------lag compensator-----------------------------%%
% %% defining the system
% Gu = zpk([],[-1 -2 -10],164.6)
% Kp = dcgain(Gu)
% e = 1/(1+Kp)
% e1 = e/10
% 
% Kp2 = (1-e1)/e1
% pc = 0.01 %arbitrarily selecting as 0.01
% zc = (Kp2/Kp)*pc
% 
% Gc = zpk([-zc],[-pc],0.9805)
% lag = Gu*Gc
% Tu=feedback(Gu,1);
% Tc=feedback(lag,1);
% figure(2)
% step(Tu)
% hold
% step(Tc)
% legend('un-compensated','Lag compensated')
% title('Lag controller')

%% -----------------lag-lead compensator---------------
%% defining the system
G = zpk([0],[0 -6 -10],1)
% % rltool(G)
% figure(1)
% rlocus(G)
z = -log(0.20)/(sqrt((pi*pi)+(log(0.20)*log(0.20)))) %Z=damping ratio
% sgrid(z,0)
%----------------------------------
Gu = zpk([],[0 -6 -10],192.1)
kv1 = dcgain(Gu)
Ts = 4/1.794 %setteling time
NTs = Ts/2 %two fold reduction in the settling time
sigma = 4/(NTs)
the = 180-(atand((sqrt(1-(z*z)))/z))
wd = sigma*(tan(deg2rad(180-the)))
s = -sigma + wd*i
%---------------------------
Gu1 = (s+6)/(s*(s+10)*(s+6)); %add the required zero and find angle
Theta=((180/pi)*angle(Gu1))
pole = sigma+(wd/tan(deg2rad(180+Theta)))
lead = zpk([-6],[0 -10 -6 -pole],1977)
Gce = zpk([-6],[0 -10 -6 -pole],1977)
Tu=feedback(Gu,1);
Tc=feedback(Gce,1);
figure(3)
step(Tu)
hold on
step(Tc)
grid on
legend('un-compensated','Lead compensated')
title('Lead controller')
%---------------------now add lag--------------
Gus = zpk([0 -6],[0 -6 -10 -pole],1977)
Kv = dcgain(Gus)
Gu = zpk([0],[0 -6 -10],192.1)
kv1 = dcgain(Gu)
e = 1/(Kv)
e1 = e/4.71

Kv2 = (1-e1)/e1
pc = 0.01
zc = (Kv2/Kv)*pc

lag = zpk([-zc],[-pc],1)
laglead = lag*lead


Tc2=feedback(laglead,1);
figure(4)
step(Tu)
hold on
step(Tc)
hold on
step(Tc2)
grid on
legend('un-compensated','lead-compensated','laglead-compensated')
title('Lag-Lead controller')


