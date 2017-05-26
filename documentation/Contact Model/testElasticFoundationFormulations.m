function [] = testElasticFoundationFormulations()
%E1 = 5000000;
E1 = 5;
v1 = 0.45;
h1 = 0.004;

%E2 = 10000000;
E2 = 10;
v2 = 0.45;
h2 = 0.0025;

k1 = (1-v1)*E1/((1+v1)*(1-2*v1));
k2 = (1-v2)*E2/((1+v2)*(1-2*v2));

%k1 = 10;
%k2 = 20;

K1 = (1-v1)*E1/((1+v1)*(1-2*v1)*h1);
K2 = (1-v2)*E2/((1+v2)*(1-2*v2)*h2);

KC = K1*K2/(K1+K2);

dc = 0:0.0001:0.004;

%Linear Model
%============
P_lin_lump = KC*dc;

%Nonlinear Approximation Model
%=============================
kc = k1*k2/(k1+k2);
hc=h1+h2;

P_nonlin_2side = -kc*log(1-dc./hc);

%Wolfram Solution
%================
P_nonlin_wolf = -((dc-h1-h2)*k1*k2)/(exp(h2*k1+h1*k2));

%Nonlinear Eq Solve Model
%========================
options = optimoptions('fsolve');
options.MaxFunctionEvaluations = 10000;
options.OptimalityTolerance = 1e-20;
options.StepTolerance = 1e-20;
options.FunctionTolerance = 1e-20;
options.Algorithm = 'levenberg-marquardt';
%xS = zeros(4,length(dc));
dS = zeros(2,length(dc));
Fs = zeros(2,length(dc));

for i = 1:length(dc)
    param(1) = dc(i);
    param(2) = k1;
    param(3) = h1;
    param(4) = k2;
    param(5) = h2;
    
%     x0(1) = dc(i)/2;
%     x0(2) = dc(i)/2;
%     x0(3) = P_nonlin_2side(i);
%     x0(4) = P_nonlin_2side(i);
%     
%     f=@(x) nonlin_twoside(x,param);
%     xS(:,i) = fsolve(f,x0,options);

    d0(1) = dc(i)/2;
    d0(2) = dc(i)/2;
    
    f=@(d) nonlin_twoside_depth(d,param);
    dS(:,i) = fsolve(f,d0,options);
    
    %R = fsolve(f,d0,options);
    
    Fs(1,i) = -k1*log(1-dS(1,i)/h1);
    Fs(2,i) = -k2*log(1-dS(2,i)/h2);
end

%Plot Results
%============
lw = 2;
figure()
hold on

%plot(dc,P_lin_lump,'LineWidth',lw);
%plot(dc,P_nonlin_2side,'LineWidth',lw);
%plot(dc,xS(3,:))
%plot(dc,Fs(1,:),':','LineWidth',lw);
%plot(dc,Fs(2,:),'--','LineWidth',lw);
plot(dc,P_nonlin_wolf,'LineWidth',lw);
%legend('linear','nonlin approx','nonlin solver F1','nonlin solver F2')
%legend('nonlin approx','nonlin solver F1','nonlin solver F2')

%figure()
%hold on
%plot(dc,dS(1,:),'LineWidth',2);
%plot(dc,dS(2,:),'LineWidth',2)
end

function Z = nonlin_twoside(x,param)
%x(1) = d1
%x(2) = d2
%x(3) = F1
%x(4) = F2

dc = param(1);
k1 = param(2);
h1 = param(3);
k2 = param(4);
h2 = param(5);

Z(1) = x(1)+x(2)-dc;
Z(2) = x(3)-x(4);
Z(3) = -k1*log(1-x(1)/h1)-x(3);
Z(4) = -k2*log(1-x(2)/h2)-x(4);

%F(1) = (1-d(1)/h1)^k1-(1-d(2)/h2)^k2;
%F(2) = d(1)+d(2)-dc;
end

function Z = nonlin_twoside_depth(d,param)
%d(1) = d1
%d(2) = d2
%x(3) = F1
%x(4) = F2

dc = param(1);
k1 = param(2);
h1 = param(3);
k2 = param(4);
h2 = param(5);

Z(1) = k1*log(1-d(1)/h1)-k2*log(1-d(2)/h2);
%Z(1) = (1-d(1)/h1)^k1-(1-d(2)/h2)^k2;
Z(2) = d(1)+d(2)-dc;
end