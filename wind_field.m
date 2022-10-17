% nice latex plotting
set(0,'defaulttextinterpreter','latex');

%% PARMETERS
% ===========
alpha = 0.5;
h_ref = 15;
Vx = 10;
Vy = 10;

num = 100;
%% calculation of sigmoid
height = linspace(0,30,num);
pos_x = zeros(1,num);
pos_y = zeros(1,num);
wind_x = zeros(1,num);
wind_y = zeros(1,num);
wind_z = zeros(1,num);
for i=1:100
    z = height(i);
    wind_x(i) = sigmoid(Vx,alpha,h_ref,z);
    wind_y(i) = sigmoid(Vy,alpha,h_ref,z);
    wind_z(i) = 0;
end


%% plotting
fig = figure();
fig.Name = 'wind_field';
plot3(wind_x,wind_y,height);
hold on;
grid on;
plot3(pos_x,pos_y,height,'k');
step = 10;
quiver3(pos_x(1 : step : end), pos_y(1 : step : end), height(1 : step : end),...
            wind_x(1 : step : end), wind_y(1 : step : end), pos_x(1 : step : end), 0, 'r');
p = fill3([15,15,-5,-5]', [-5,15,15,-5]', [h_ref,h_ref,h_ref,h_ref], 1);
p.FaceAlpha = '0.2' ;
daspect([1 1 1]);
xlim([-5,15]);
ylim([-5,15]);
xlabel('$w_x$ ($\frac{m}{s}$)');
ylabel('$w_y$ ($\frac{m}{s}$)');
zlabel('$Z$ (m)');
legend('$\vec{w} (V_x,V_y,\alpha,h_{ref})$','Interpreter','latex');







%% functions
function output = sigmoid(V,a,h,z)
    output = V/(1+exp(-a*(z-h)));
end