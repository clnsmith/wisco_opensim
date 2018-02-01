%% Understanding Ligament Parameter Relationships
%------------------------------------------------
nSample = 10000;

min_Lref = 1;
max_Lref = 5;

min_RS = -0.05;
max_RS = 0.05;

min_dL = 0;
max_dL = 1;

K = 800;

Lref = min_Lref +(max_Lref-min_Lref)*rand([nSample 1]);
dL = min_dL +(max_dL-min_dL)*rand([nSample 1]);
RS = min_RS + (max_RS-min_RS)*rand([nSample 1]);

L0 = Lref./(1+RS);
L = Lref+dL; 

strain = (L-L0)./L0;
force = K*strain;

figure('name','Lref vs Force')
plot(Lref,force,'.');
xlabel('Reference Length [cm]')
ylabel('Force [N]')


