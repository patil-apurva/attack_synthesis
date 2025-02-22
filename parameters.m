close all

danger_y1 = 0.15;
danger_y2 = 0.25;

%time parameters
h = 0.01; % time step
t0 = 0.0; %initial time
T = 4; % final time

%==================================
%tuning parameters
s2 = 0.01; %s^2
eta = 0.1 %lagrange multiplier
a = 1; %R=a.I
b = 0.1; %V=b.||x(t)||^2
e = 1;
lambda = 0.1 %cost of adversary's control input also the PDE linearization constant for the adversary's problem
%==================================

G_u = [0 0; 0 0; 1 0; 0 1];

x0 = [-0.5; 0.1; 0; 0]; %initial position
runs = 10000; %MC runs
traj_num = 100; %number of trajectories to plot
%==========================================================================
alpha = a*s2; %PDE linearization constant for the agent's problem
k1 = -e/T;
k2 = k1;
k3 = k1;

s = sqrt(s2); %Sigma=s.I

figure (2)
hold on;
set(gca, 'FontName', 'Arial', 'FontSize', 18)
% xlabel('$p_x$', 'Interpreter','latex', 'FontSize', 30); ylabel('$p_y$', 'Interpreter','latex','FontSize', 30); 
% xticks(xP:0.1:xQ)
% yticks(yP:0.1:yQ)
set(gca,'LineWidth',1)
ax = gca;
ax.LineWidth = 1;
ax.Color = 'w';
axis equal;
xP = x0(1)-0.08;
xQ = 0.05;
yP = -0.05;
yQ = 0.25;

xlim([xP, xQ]);
ylim([yP, yQ]);

set(gca,'XColor', 'none','YColor','none')

rectangle('Position',[xP yP xQ-xP yQ-yP], 'FaceColor', 'w', 'EdgeColor', 'k', 'LineWidth',1.5); %outer rectangle

I = imread('danger.jpg');
image([-0.05 0.05], [(danger_y1 + danger_y2)/2 + 0.05 (danger_y1 + danger_y2)/2 - 0.05], I);

I = imread('car.jpg');
image([x0(1)-0.07 x0(1)+0.07], [x0(2)+0.05 x0(2)-0.05], I);

I = imread('goal.jpg');
image([-0.06 0.06], [0.05 -0.05], I);




