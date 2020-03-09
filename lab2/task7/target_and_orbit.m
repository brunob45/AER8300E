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
theta = 0;
phi = 0.3;
psi =0;

%this is where we save euler angles
PHI = 0;
THETA = 0;
PSI = 0;
ERROR_PSI = 0;
ERROR_THETA = 0;

error_theta_integral = 0;
error_psi_integral = 0;
error_phi_integral = 0;

%For demonstration
figure(1)
fh = plot3(X_target(:,1),X_target(:,2),X_target(:,3));
hold on
plot3(X_orbit(:,1),X_orbit(:,2),X_orbit(:,3));
[X,Y,Z] = getTriangleVertices([0,0,0],[0,pi*0.,-pi*0.5]);
patch(X,Y,Z,'red')
axis equal

old_error = 0;

for i = 1:2000 %which is the number of steps we want
    i
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
    psi_desired = eta;
    theta_desired = -zeta;
    phi_desired = 0;
    
    %% The PID control part
    %till now, there is no update for the satellite states (phi, theta,
    %psi), only calculated desired values.
    
%     for that, we need dynamics, represented by the moments of intertias in (kg m^2)
    Ixx = 10.42;
    Iyy =  9.96;
    Izz =  9.96;
    
    %calculating the moments that will rotate the satellite (using PID 
    %controller)
    %first we calculate the error .. start by calculating spherical axes
    %angles from x_rel_local
    
    zeta_l = atan2(x_rel_local(3),  sqrt(x_rel_local(1)^2+x_rel_local(2)^2));  %synonymous to desired pitch
    eta_l = atan2(x_rel_local(2),x_rel_local(1)); %synonymous to desired yaw
    
    zeta_l * 180/pi;
    
    ky = 500.00;
    kiy = 0;
    
    kz = 235;
    kiz = 0;
    kdz = 0;
    
    kx = 800;
    kix = 100;
    
    if i > 1
%         error_psi = 0 + eta_l;
%         error_theta = 0 - zeta_l;
        
        phi*180/pi;
        
        
        error_psi = psi_desired - psi
        error_theta = theta_desired - theta;
        error_phi = 0 - phi;
        
        psi_desired
        psi
        if abs(error_psi - old_error) > 1.9*pi
            error_psi = old_error;
        else
            old_error = error_psi;
        end
        error_psi
        
        %integral errors
        error_phi_integral = error_phi_integral + error_phi * dt;
        error_theta_integral = error_theta_integral + error_theta * dt;
        error_psi_integral = error_psi_integral + error_theta * dt;
        
        %differential errors
        error_psi_differential = (error_psi - ERROR_PSI(i-1))/dt;
        
        Mx = kx * error_phi + kix * error_phi_integral;
        My = ky * error_theta + kiy*error_theta_integral;
        Mz = kz * error_psi + kiz * error_psi_integral + kdz * error_psi_differential;
%         My = 10;
%         Mz = 10;
        ERROR_PSI(i) = error_psi;
        ERROR_THETA(i) = error_theta;
    else
        Mx = 0;
        My = 0;
        Mz = 0;
    end
    
    
    
    %updating the states (the angular rates)
    d_p = Mx/Ixx;
    d_q = My/Iyy;
    d_r = Mz/Izz;
    
    %convert to rates of psi,theta,phi (taken from Nelson's book on flight
    %mechanics)
    euler_rates = [1,sin(phi)*tan(theta),cos(phi)*tan(theta);...
        0, cos(phi), -sin(phi);...
        0, sin(phi)/cos(theta),cos(phi)/cos(theta)]*[d_p,d_q,d_r]';
    
    d_phi = euler_rates(1);
    d_theta = euler_rates(2);
    d_psi = euler_rates(3);
    
    phi = phi + d_phi * dt;
    theta = theta + d_theta * dt;
    psi = psi + d_psi * dt;
    
    %saturation of angles
    if psi > 2*pi
        psi = psi - 2*pi;
    elseif psi < -2*pi
        psi = psi + 2*pi;
    end
    
    if psi > 0
        if psi > pi
            psi = psi - 2*pi;
        end
    end
    
    if psi < 0
        if psi < -pi
            psi = psi + 2*pi;
        end
    end
    
    THETA(i) = theta;
    PHI(i) = phi;
    PSI(i) = psi;
    
    
    %% This part is for animation only
    cla
    %plotting trajectories
    fh = plot3(X_target(:,1),X_target(:,2),X_target(:,3));
    hold on
    plot3(X_orbit(:,1),X_orbit(:,2),X_orbit(:,3));
    plot3(X_target(i,1),X_target(i,2),X_target(i,3),'bo','MarkerSize',8,'MarkerFaceColor','b')
%     plot3(X_orbit(i,1),X_orbit(i,2),X_orbit(i,3),'ro','MarkerSize',8,'MarkerFaceColor','r')

    %plotting lines
    line([X_orbit(i,1),X_target(i,1)],[X_orbit(i,2),X_target(i,2)],[X_orbit(i,3),X_target(i,3)],'Color','green')
%     line([0,x_rel_global(1)],[0,x_rel_global(2)],[0,x_rel_global(3)],'Color','m') %a line that representes the relative vector between sat and target in global axes
    a = [1500,0 0]';
    A = rotation(phi_desired,theta_desired,psi_desired)*a;
    line([X_orbit(i,1),X_orbit(i,1)+A(1)],[X_orbit(i,2),X_orbit(i,2)+A(2)],[X_orbit(i,3),X_orbit(i,3)+A(3)],'Color','black') %extension of the x axis of the satellite towards the target
%     B = rotation(phi,theta,psi)*a;
%     line([0,B(1)],[0,B(2)],[0,B(3)],'Color','black')
    
    %plotting rigid bodies
    [X,Y,Z] = getTriangleVertices(X_orbit(i,:),[phi_desired,theta_desired,psi_desired]);
    patch(X,Y,Z,'red')
    [X,Y,Z] = getTriangleVertices(X_orbit(i,:),[phi,theta,psi]);
    patch(X,Y,Z,'magenta')
%     [X,Y,Z] = getTriangleVertices(X_orbit(i,:)*0,[phi,theta,psi]);
%     patch(X,Y,Z,'blue') %reference object at origin that rotates the same way as the rigid body in actual configuration
%     [X,Y,Z] = getTriangleVertices(X_orbit(i,:)*0,[phi_desired,theta_desired,psi_desired]);
%     patch(X,Y,Z,'red') %reference object at origin that rotates the same way as the rigid body in desired configuration
    
    xlabel('X_0')
    ylabel('Y_0')
    zlabel('Z_0')
%     axis([-1 1 -1 1 -1 1]*500)
%     view(0,90)
    drawnow
%     pause(.1)
end

%% 
%this function follows a Z-Y-X rotation
%this function rotates from local to global (?)
function Rt = rotation(phi,theta,psi) 
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
