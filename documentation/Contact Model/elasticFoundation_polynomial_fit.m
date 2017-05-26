% Nonlinear Elastic Foundation with fitted polynomials
%%====================================================
close all; clc; clear;

E1 = 5000000;
v1 = 0.45;
h1 = 0.005;

E2 = 10000000;
v2 = 0.45;
h2 = 0.0025;

k1 = (1-v1)*E1/((1+v1)*(1-2*v1));
k2 = (1-v2)*E2/((1+v2)*(1-2*v2));

d = 0:0.000005:0.0049;
dmm = d./10*3;

P1 = -k1*log(1-d/h1)/10^6;
P2 = -k2*log(1-d/h2)/10^6;

%Quadratic
p2c1 = polyfitZero(d,P1,2);
p2c2 = polyfitZero(d,P2,2);

p2P1 = polyval(p2c1,d);
p2P2 = polyval(p2c2,d);

%Cubic
p3c1 = polyfitZero(d,P1,3);
p3c2 = polyfitZero(d,P2,3);

p3P1 = polyval(p3c1,d);
p3P2 = polyval(p3c2,d);

%Quartic
p4c1 = polyfitZero(d,P1,4);
p4c2 = polyfitZero(d,P2,4);

p4P1 = polyval(p4c1,d);
p4P2 = polyval(p4c2,d);

%Quintic
p5c1 = polyfitZero(d,P1,5);
p5c2 = polyfitZero(d,P2,5);

p5P1 = polyval(p5c1,d);
p5P2 = polyval(p5c2,d);

%6th
p6c1 = polyfitZero(dmm,P1,6);
p6c2 = polyfitZero(dmm,P2,6);

p6P1 = polyval(p6c1,dmm);
p6P2 = polyval(p6c2,dmm);

%7th
p7c1 = polyfitZero(dmm,P1,7);
p7c2 = polyfitZero(dmm,P2,7);

p7P1 = polyval(p7c1,dmm);
p7P2 = polyval(p7c2,dmm);

%Taylor Series Expansion
%=======================
x1 = 1-d/h1;

tsP1_ln = (x1-1)-(x1-1).^2/2+(x1-1).^3/3-(x1-1).^4/4+(x1-1).^5/5-(x1-1).^6/6+(x1-1).^7/7;
tsP1 = -k1*tsP1_ln/10^6;

% Plot Results
%=============
lw=2;

figure('name','P1 Pressure')
hold on;
title('P1 Pressure')

plot(d,P1,'k','LineWidth',lw)
plot(d,p2P1,'LineWidth',lw)
plot(d,p3P1,'LineWidth',lw)
plot(d,p4P1,'LineWidth',lw)
plot(d,p5P1,'LineWidth',lw)
%plot(d,p6P1,'LineWidth',lw)
%plot(d,p7P1,'LineWidth',lw)
legend('P1 ln','P1 2nd','P1 3rd','P1 4th','P1 ts')


figure('name','P2 Pressure')
hold on;
title('P2 Pressure')

plot(d,P2,'LineWidth',lw)
plot(d,p2P2,':','LineWidth',lw)
plot(d,p3P2,'--','LineWidth',lw)


legend('P2 ln','P2 2nd','P2 3rd')