clear all 
%% Loading data
test20=load('20sekundtestgrzalka.txt');
test15=load('15sekundtestgrzalka.txt');
test10=load('10sekundtestgrzalka.txt');
test5=load('5sekundtestgrzalka.txt');

test15(:,2)=test15(:,2)/1000;
test20(:,2)=test20(:,2)/1000;

test5(:,2)=test5(:,2)/1000;
test10(:,2)=test10(:,2)/1000;


%plot results
plot(test20(:,2),test20(:,1));
title('Test heater on for 20 s');
figure
plot(test15(:,2),test15(:,1));
title('Test heater on for 15 s');
figure
plot(test10(:,2),test10(:,1));
title('Test heater on for 10 s')
figure
plot(test5(:,2),test5(:,1));
title('Test heater on for 5 s')

model_x=test20(2:end,2);
model_y=test20(2:end,1);


%% static characteristics
t_steady = 175;

test20_steady = test20(:,1);
test20_steady_mean = mean(test20_steady(end-t_steady:end));

%here range has to be smaller becouse of too long measurement
test15_steady = test15(:,1);
test15_steady_mean = mean(test15_steady(2410-t_steady:2410));

test10_steady = test10(:,1);
test10_steady_mean = mean(test10_steady(end-t_steady:end));

test5_steady = test5(:,1);
test5_steady_mean = mean(test5_steady(end-t_steady:end));

steady_values=[20,test5_steady_mean,test10_steady_mean,test15_steady_mean,test20_steady_mean];
PWM_pulse=[0,5,10,15,20];
p = polyfit(PWM_pulse,steady_values,1);
t=[0:1:20];
linear_fit=t*p(1)+p(2);
figure
plot(PWM_pulse,steady_values);
hold on
plot(t,linear_fit);
legend('Static characteristic')

%Conclusions: linearity of model will be better if measurements would start from same temperature  



%% Model parameters

%fitting made by cftool
% General model:
%      f(x) = 75*k*(1-exp(-(x-To)/T))+25
% Coefficients (with 95% confidence bounds):
%        T =       67.62  (66.83, 68.42)
%        To =       9.577  (9.301, 9.854)
%        k =      0.7428  (0.7404, 0.7453)
% 
% Goodness of fit:
%   SSE: 8573
%   R-square: 0.9874
%   Adjusted R-square: 0.9873
%   RMSE: 1.853

s = tf('s');
T = 850.3;        
%static gain -k of the object is equaled factor a of linear fitting to
%static characteristics
k_static = p(1);       % [-]

T =67.62; % time constant
To =9.577; % transport delay
k =0.74;  %gain of model
Ts=1; %sampling time
y_ref = 30;
start_temp=25;
% tc dla To/2=<tc=<T0
%parameter for SIMC method
tc = 7;

PWM_PULSE=1;


% Paremeters for SIMC method
Kp = T / (k*(tc+To));
Ti = min(T, 4*(tc+To));
Td = 0;

%PADE transfer function for e^-sTo 
[PADE_NUM,PADE_DEN]=pade(To,1);
PADE_MODEL=tf(PADE_NUM,PADE_DEN);
% transfer function for model
G=(k/(1+s*T))*PADE_MODEL
G_basic=(k/(1+s*T));
[NUM_Gbasic,DEN_Gbasic]=tfdata(G_basic,'v')

%step(G,75)
%discrete transfer function
Gd = c2d(G, Ts, 'tustin');
Gd_basic=c2d(G_basic, Ts, 'tustin');

[Gd_basicNUM,Gd_basicDEN]=tfdata(Gd_basic,'v');

sim('simulation')
%figure
%step(Gd_basic,75)


