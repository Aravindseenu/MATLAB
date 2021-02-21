%% Prepare the workspace
clear 
clc
close all

%% defining the system
% %---------------------PI_compensator--------------------%
% G = zpk([],[-1 -2 -10],1)
% % rltool(G)
% figure(1)
% rlocus(G)
% z = 0.174
% sgrid(z,0)
% 
% Gu = zpk([],[-1 -2 -10],164.6)
% Kp = dcgain(Gu)
% error = 1/(1+Kp)
% 
% Gc = zpk([-0.1],[0],0.9805);
% PI = Gu*Gc
% Kp1 = dcgain(PI);
% 
% Gc = zpk([-0.1],[0],0.9805);
% Gce=Gu*Gc;
% Kp1 = dcgain(Gce);
% Tu=feedback(Gu,1);
% Tc=feedback(Gce,1);
% 
% figure(2)
% rlocus(Gce)
% sgrid(z,0)
% 
% figure(3)
% step(Tu)
% hold
% step(Tc)
% legend('un-compensated','PI-compensated')
% title('PI controller and its response')

%% ----------------PD_compensator-----------------
%% type 1 (with no zero)
%% defining the system
% G = zpk([],[0 -4 -6],1)
% %rltool(G)
% figure(1)
% rlocus(G)
% z = -log(0.16)/(sqrt((pi*pi)+(log(0.16)*log(0.16)))) %Z=damping ratio
% sgrid(z,0)
% %------------------------------------------
% Gu = zpk([],[0 -4 -6],43.4)
% Ts = 4/1.205 %setteling time
% NTs = Ts/3 %threefold reduction in the settling time
% sigma = 4/(NTs)
% the = atand((sqrt(1-(z*z)))/z)
% wd = sigma*(tan(deg2rad(the)))
% s = -sigma + wd*i
% %------------------------------------------
% Gu1 = (1)/(s*(s+4)*(s+6)); %type the question
% Theta=180-((180/pi)*angle(Gu1))
% zero = sigma-(wd/tan(deg2rad(180-Theta))) %the PD ans
% PD = zpk([-zero],[0 -4 -6],47.188) % final answer
% %-------------------------------------------
% Tu=feedback(Gu,1);
% Tc=feedback(PD,1);
% % figure(3)
% step(Tu)
% hold on
% step(Tc)
% grid on
% legend('un-compensated','PD-compensated')
% title('PD controller and its response')

%% ----------------PD_compensator-----------------
%% type 2(with zero)
% %% defining the system
% G = zpk([-6],[-2 -3 -5],1)
% 
% figure(1)
% % rlocus(G)
% %z = 0.707 % given
% % sgrid(z,0)
% %rltool(G)
% z = 0.707 
% %-----------------------------------------
% Gu = zpk([-6],[-2 -3 -5],4.4616)%
% Ts = 4/2.32 %setteling time
% NTs = Ts/2 %twofold reduction in the settling time
% sigma = 4/(NTs)
% the = atand((sqrt(1-(z*z)))/z)
% wd = sigma*(tan(deg2rad(the)))
% s = -sigma + wd*i
% %-----------------------------------------
% Gu1 = (s+6)/((s+2)*(s+3)*(s+5)); %type the question
% Theta=((180/pi)*angle(Gu1))
% zero = (wd/tan(deg2rad(the)))+sigma %the PD ans
% PD = zpk([-zero -6],[-2 -3 -5],2.2734) % final answer
% 
% Tu=feedback(Gu,1);
% Tc=feedback(PD,1);
% % figure(3)
% step(Tu)
% hold on
% step(Tc)
% grid on
% legend('un-compensated','PD-compensated')
% title('PD controller and its response')

%% --------------------PID-----------------------------

%% Type 1 settling time method
%% defining the system
% G = zpk([],[-4 -6 -10],1)
% % rltool(G)
% % figure(1)
% % rlocus(G)
%  z = -log(0.25)/(sqrt((pi*pi)+(log(0.25)*log(0.25)))) %Z=damping ratio
% % sgrid(z,0)
% %--------------------------------------
% Gu = zpk([],[0 -4 -6 -10],417)
% Ts = 4/2.71 %setteling time
% NTs = 2 %no reduction in the settling time
% sigma = 4/(NTs)
% the = atand((sqrt(1-(z*z)))/z)
% wd = sigma*(tan(deg2rad(the)))
% s = -sigma + wd*i
% %---------------------------------------
% Gu1 = (1)/(s*(s+4)*(s+6)*(s+10));
% Theta=180-((180/pi)*angle(Gu1))
% zero = sigma-(wd/tan(deg2rad(180-Theta)))
% PD = zpk([-zero],[0 -4 -6 -10],47.188)%PD ans 
% %----------now add pI to answer----------------
% PI = zpk([-0.1],[0],1)
% PID = PD*PI


%% --------------------PID-----------------------------
%% Type 2 peak time method
%% defining the system
 Gu = zpk([-8],[-3 -6 -10],121)
% rltool(G)
figure(1)
rlocus(Gu)
z = -log(0.20)/(sqrt((pi*pi)+(log(0.20)*log(0.20)))) %Z=damping ratio
sgrid(z,0)
%---------------------------------------
Tu=feedback(Gu,1);
figure(2) % to find the peak time of system
step(Tu) 
title('Un compensated system')
%---------------------------------------
Tp = 0.298  %peak time
NTp = (2*Tp)/3 %two third of peak time
wd = pi/NTp 
the =atand((sqrt(1-(z*z)))/z)
sigma = wd/(tan(deg2rad(180-the)))
s = sigma + wd*i
%---------------------------------------
Gu1 = (s+8)/((s+3)*(s+6)*(s+10))
Theta = 180-((180/pi)*angle(Gu1))
zero = (wd/tan(deg2rad(Theta)))-sigma
PD = zpk([-zero -8],[-3 -6 -10],5.34)
%----------now add pI to answer----------------
PI = zpk([-0.1],[0],4.6)
PID = PD*PI 
Gce = zpk([-zero -8],[-3 -6 -10],5.34)
Tu=feedback(Gu,1);
Tc=feedback(Gce,1);
figure(3)
step(Tu)
hold on
step(Tc)
grid on
legend('un-compensated','PD-compensated')
Gc = zpk([-0.1],[0],4.6)
title('PD controller')
 
Gce2=Gce*Gc
Tc2=feedback(Gce2,1);
figure(4)
step(Tu)
hold on
step(Tc)
hold on
step(Tc2)
grid on
legend('un-compensated','PD-compensated','PID-compensated')
title('PID controller')
 Ta = tf([1],[1 0]);
 
