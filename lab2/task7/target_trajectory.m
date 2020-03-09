clear all
close all
clc


%% making a simple loop for calculating the circular trajectory

dt = 0.01;
omega = 1;
r = 700000;
alpha = 0; % alpha is the local angle in the local trajectory axes


% define the rotation matrix (constant)
phi = 0;
theta = pi*0.4;
psi = 0;

Rx = [1 0 0;0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta) ];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

RR = Rx * Ry* Rz;

for i = 1:10000
    
    %calculating the local coorddinates wrt curcular trajectory plane
    x_local = [r*cos(alpha) , r*sin(alpha), 0]';
    
    %obtain the global coordinate of the point on trajectory
    x_global = RR  * x_local;
    
    %update the local angle alpha
    alpha = alpha + omega * dt;
    
    %save the global position
    X(i,:) = x_global';
    
end


figure(1)
plot3(X(:,1),X(:,2),X(:,3));
axis equal

% %% simulation of the target moving on the trajectory
%  figure(2);
% plot3(X(:,1),X(:,2),X(:,3));
% hold on
% fh = plot3(X(1,1),X(1,2),X(1,3),'ro','MarkerSize',8,'MarkerFaceColor','r');
% 
% for i = 1:10000
%    clf(fh)
%    plot3(X(:,1),X(:,2),X(:,3));
%    fh = plot3(X(i,1),X(i,2),X(i,3),'ro','MarkerSize',8,'MarkerFaceColor','r');
%    drawnow;
% end
%    
    
    