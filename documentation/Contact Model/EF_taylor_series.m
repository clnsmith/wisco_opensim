%% Nonlinear Elastic Foundation - Taylor Series Expansion
%========================================================

%close all; clc; clear;

E1 = 5000000;
v1 = 0.45;
h1 = 0.0025;

E2 = 10000000;
v2 = 0.45;
h2 = 0.0025;

k1 = (1-v1)*E1/((1+v1)*(1-2*v1));
k2 = (1-v2)*E2/((1+v2)*(1-2*v2));

d = 0:0.00001:0.0024;

P1 = -k1*log(1-d/h1)/10^6;
P2 = -k2*log(1-d/h2)/10^6;

%Taylor Series Expansion
%=======================
x1 = -d/h1;
ln4_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4;
ts4_P1 = -k1*ln4_P1/10^6;

ln5_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4+(x1).^5/5;
ts5_P1 = -k1*ln5_P1/10^6;

ln6_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4+(x1).^5/5-(x1).^6/6;
ts6_P1 = -k1*ln6_P1/10^6;

ln7_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4+(x1).^5/5-(x1).^6/6+(x1).^7/7;
ts7_P1 = -k1*ln7_P1/10^6;

ln8_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4+(x1).^5/5-(x1).^6/6+(x1).^7/7-(x1).^8/8;
ts8_P1 = -k1*ln8_P1/10^6;

ln9_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4+(x1).^5/5-(x1).^6/6+(x1).^7/7-(x1).^8/8+(x1).^9/9;
ts9_P1 = -k1*ln9_P1/10^6;

ln10_P1 = (x1)-(x1).^2/2+(x1).^3/3-(x1).^4/4+(x1).^5/5-(x1).^6/6+(x1).^7/7-(x1).^8/8+(x1).^9/9-(x1).^10/10;
ts10_P1 = -k1*ln10_P1/10^6;

% Plot Results
%=============
lw=2;

figure('name','P1 Pressure')
hold on;
title('P1 Pressure')

plot(d,P1,'k','LineWidth',lw)
plot(d,ts4_P1,'LineWidth',lw)
plot(d,ts5_P1,'LineWidth',lw)
plot(d,ts6_P1,'LineWidth',lw)
plot(d,ts7_P1,'LineWidth',lw)
plot(d,ts8_P1,'LineWidth',lw)
plot(d,ts9_P1,'LineWidth',lw)
plot(d,ts10_P1,'LineWidth',lw)
legend('ln','ts 4','ts 5','ts 6','ts 7','ts 8','ts 9','ts 10')


figure('name','P2 Pressure')
hold on;
title('P2 Pressure')

plot(d,P2,'LineWidth',lw)
%plot(d,p2P2,':','LineWidth',lw)
%plot(d,p3P2,'--','LineWidth',lw)


legend('P2 ln','P2 2nd','P2 3rd')