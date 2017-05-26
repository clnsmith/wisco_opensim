%% Evaluating elastic foundation formulations
%==========================================================================

E = 5000000;
v = 0.45;
h = 0.005;

K = (1-v)*E/((1+v)*(1-2*v)*h);

E1 = 5000000;
v1 = 0.45;
h1 = 0.0025;

E2 = 10000000;
v2 = 0.45;
h2 = 0.0025;

K1 = (1-v1)*E1/((1+v1)*(1-2*v1)*h1);
K2 = (1-v2)*E2/((1+v2)*(1-2*v2)*h2);

d = 0:0.0001:0.004;


%% Lumped Models
P_lin_lump = K*d./10^6;

P_nonlin_lump = -(1-v)*E/((1+v)*(1-2*v))*log(1-d./h)./10^6;

%% Two sided Models
Kc = K1*K2/(K1+K2);
P_lin_2side = Kc*d./10^6;

k1 = (1-v1)*E1/((1+v1)*(1-2*v1));
k2 = (1-v2)*E2/((1+v2)*(1-2*v2));
kc = k1*k2/(k1+k2);
P_nonlin_2side = -2*kc*log(1-d./h)./10^6;
%% Plot Results
%==============
line_width = 2;

figure('name','Elastic Foundation Formulations - Same Properties')
hold on

title('Elastic Foundation - Same Properties')
xlabel('Depth [m]')
ylabel('Pressure [MPa]')
plot(d,P_lin_lump,'LineWidth',line_width)
plot(d,P_nonlin_lump,'LineWidth',line_width)
plot(d,P_lin_2side,'--','LineWidth',line_width)
plot(d,P_nonlin_2side,'--','LineWidth',line_width)
legend('Linear Lumped','Nonlinear Lumped','Linear 2side','Nonlinear 2side')
