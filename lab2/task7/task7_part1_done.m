clear all
close all
clc


%% making a simple loop for calculating the circular trajectory

dt = 0.01;
omega_target = 1;
omega_orbit = 1;

r = 700;
alpha = 0; % alpha is the local angle in the local trajectory axes
beta = 0;
R = 1200;

X_target = [0 0 0];
X_orbit = [0 0 0];

% define the rotation matrix for satellite and target orbits(constant and arbitrary)
rot_target = rotation(0,1,0);
rot_orbit = rotation(0, pi*0.5,0);

%the euler angles of the satellite itself
theta = -1;
phi = -0.;
psi =3.0;

%For demonstration
figure(1)
fh = plot3(X_target(:,1),X_target(:,2),X_target(:,3));
hold on
plot3(X_orbit(:,1),X_orbit(:,2),X_orbit(:,3));
[X,Y,Z] = getTriangleVertices([0,0,0],[0,pi*0.,-pi*0.5]);
patch(X,Y,Z,'red')
axis equal

for i = 1:1000 %which is the number of steps we want
    
    %calculating the local coorddinates wrt curcular trajectory plane
    x_local_target = [r*cos(alpha) , r*sin(alpha), 0]';
    x_local_orbit = [R*cos(beta) , R*sin(beta), 0]';
    
    %obtain the global coordinate of the point on trajectory
    x_global_target = rot_target  * x_local_target;
    x_global_orbit = rot_orbit  * x_local_orbit;
    
    %update the local angle alpha
    alpha = alpha + omega_target * dt;
    beta = beta + omega_orbit * dt;
    
    %save the global position
    X_target(i,:) = x_global_target';
    X_orbit(i,:) = x_global_orbit';
    
    %calculate the vector joining the target and the satellite
    x_rel_global = (x_global_target - x_global_orbit);
    
    %tranforming this vector into local axes (using a rotation matrix with
    %negative the values of phi,theta and psi) (negative equivalent to
    %inverse of the rotation matrix)
    rot_local = rotation(-phi,-theta,-psi); 
    x_rel_local = rot_local * x_rel_global;
    
    %calculate the angles of spherical coordinates of the local verctor
    %we've obtained
    zeta = atan2(x_rel_global(3),  sqrt(x_rel_global(1)^2+x_rel_global(2)^2));  %synonymous to desired pitch
    eta = atan2(x_rel_global(2),x_rel_global(1)); %synonymous to desired yaw
    
    %based on spherical coordinates of the relative global vector, the x
    %axis (front) of the satellite is pointed by rotating by angle eta
    %around z axis and then angle -zeta around local y axis (notice that at
    %the beginning the local and global z axes are the same)
    psi = eta;
    theta = -zeta;
    
    %% This part is for animation only
    cla
    %plotting trajectories
    fh = plot3(X_target(:,1),X_target(:,2),X_target(:,3));
    hold on
    plot3(X_orbit(:,1),X_orbit(:,2),X_orbit(:,3));
    plot3(X_target(i,1),X_target(i,2),X_target(i,3),'bo','MarkerSize',8,'MarkerFaceColor','b')
% %     plot3(X_orbit(i,1),X_orbit(i,2),X_orbit(i,3),'ro','MarkerSize',8,'MarkerFaceColor','r')

    %plotting lines
    line([X_orbit(i,1),X_target(i,1)],[X_orbit(i,2),X_target(i,2)],[X_orbit(i,3),X_target(i,3)],'Color','green')
%     line([0,x_rel_global(1)],[0,x_rel_global(2)],[0,x_rel_global(3)],'Color','m')
    a = [1500,0 0]';
    A = rotation(phi,theta,psi)*a;
    line([X_orbit(i,1),X_orbit(i,1)+A(1)],[X_orbit(i,2),X_orbit(i,2)+A(2)],[X_orbit(i,3),X_orbit(i,3)+A(3)],'Color','black')
    
    %plotting rigid bodies
    [X,Y,Z] = getTriangleVertices(X_orbit(i,:),[phi,theta,psi]);
    patch(X,Y,Z,'red')
%     [X,Y,Z] = getTriangleVertices(X_orbit(i,:)*0,[phi,theta,psi]);
%     patch(X,Y,Z,'blue')
%     [X,Y,Z] = getTriangleVertices(X_orbit(i,:)*0,[phi,-0.8913,2.4438]);
%     patch(X,Y,Z,'green')


    xlabel('X_0')
    ylabel('Y_0')
    zlabel('Z_0')
%     axis([-1 1 -1 1 -1 1]*500)
%     view(0,90)
    drawnow
%     pause(.1)
end



%% For testing purposes
% figure(1)
% fh = plot3(X_target(:,1),X_target(:,2),X_target(:,3));
% hold on
% plot3(X_orbit(:,1),X_orbit(:,2),X_orbit(:,3));
% [X,Y,Z] = getTriangleVertices([0,0,0],[0,pi*0.,psi]);
% patch(X,Y,Z,'red')
% axis equal
% 
% for i = 1:size(X_target,1)    
%     fh = plot3(X_target(:,1),X_target(:,2),X_target(:,3));
%     hold on
%     plot3(X_orbit(:,1),X_orbit(:,2),X_orbit(:,3));
%     plot3(X_target(i,1),X_target(i,2),X_target(i,3),'bo','MarkerSize',8,'MarkerFaceColor','b')
% %     plot3(X_orbit(i,1),X_orbit(i,2),X_orbit(i,3),'ro','MarkerSize',8,'MarkerFaceColor','r') %to draw a moving point
%     line([X_orbit(i,1),X_target(i,1)],[X_orbit(i,2),X_target(i,2)],[X_orbit(i,3),X_target(i,3)],'Color','green')
%     [X,Y,Z] = getTriangleVertices(X_orbit(i,:),[0,0,psi]);
%     patch(X,Y,Z,'red') %to draw a moving triangle (rigid body)
%     xlabel('X_0')
%     ylabel('Y_0')
%     zlabel('Z_0')
%     drawnow
%     cla
% end


%% 
%this function follows a Z-Y-X rotation
%this function rotates from local to global (?)
function Rt = rotation(phi,theta,psi) 
% theta = -theta;
Rx = [1 0 0;0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta) ];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

% Rt = Rx * Ry* Rz; %target rotation matrix (this is wrong)
Rt = Rz * Ry * Rx;%target rotation matrix (this is the right one)
end

function [X, Y ,Z] = getTriangleVertices(pos,angle)
k = 100;
x = pos(1);
y = pos(2);
z = pos(3);
phi = angle(1);
theta = angle(2);
psi = angle(3);

% the idea of this function is that we have a triangle in local x-y plane,
%so we rotate it according to the attitude of the rigid body (calculate 
%the new correct vertices in global axes) and then translate these points
%so that the median of the triangle (it's centroid) lies on the given point

%in the local axes the coordinates of the vertices are:
%p1: (0.5k, -2/3k, 0)
%p2: (-0.5k, -2/3k, 0)
%p1: (0, 4/3k, 0)

%here we consider an isosceles triangle of base length = 1 and height = 2

%rotate the vertices first

%create the correct rotation matrix
rot_ = rotation(phi,theta,psi);

%rotate the vertices and express them in global axes
p1 = rot_*[-2/3*k,0.5*k,0]';
p2 = rot_*[-2/3*k,-0.5*k,0]';
p3 = rot_*[4/3*k,0,0]';

%translate the rotated vertices so that the median is the given point
p1 = p1 + [x,y,z]';
p2 = p2 + [x,y,z]';
p3 = p3 + [x,y,z]';

X = [p1(1),p2(1),p3(1)];
Y = [p1(2),p2(2),p3(2)];
Z = [p1(3),p2(3),p3(3)];

end
