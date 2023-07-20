%% Newton Raphson Method%%
clc;
clear all;
close all;
syms theta1 theta2 theta3;


%% User initial input%%
numA = input("Enter number of A's "); %no. of links
for i=1:numA
    a(i) = input('Enter link length'); %link lengths
    thetas(i) = input('Enter theta intitial guess'); %initial angle guess
end
% a(1) = 30;
% a(2) = 20;
% a(3) = 10;
% thetas(1) = 2;
% thetas(2) = 5;
% thetas(3) = 3;

max_error = 1; %Acceptable error
%% Forward Kinematic eqn

Xe = input('Enter FKE_Xe : a(1).*cosd(theta1)+ a(2).*cosd(theta1+theta2)+a(3).*cosd(theta1+theta2+theta3)');
Ye = input('Enter FKE_Ye : a(1).*sind(theta1)+ a(2).*sind(theta1+theta2)+a(3).*sind(theta1+theta2+theta3)');
Ze = 0; %input('Enter FKE_Xe : a(1).*sind(theta1)+ a(2).*sind(theta1+theta2)+a(3).*sind(theta1+theta2+theta3)');

%% Jacobian Matrix %%

% Manual

% J1 = diff(Xe, theta1,a(1));
% J2 = diff(Xe, theta2,a(2));
% J3 = diff(Xe, theta3,a(3));
% J4 = diff(Ye, theta1,a(1));
% J5 = diff(Ye, theta2,a(2));
% J6 = diff(Ye, theta3,a(3));

%Inbuilt funct
Ja = jacobian([Xe ; Ye ; Ze],[theta1;theta2;theta3])
thetai1 = thetas(1);
thetai2= thetas(2);
thetai3 = thetas(3);

%% Goal position%%
xe = input('Enter goal x');
ye = input('Enter goal y');
ze = 0;%input('Enter goal z');

%% NR_calculator_loop%%
n = 1;
K = input('Enter no. of iterations');
error = 3;
theta_N = [];
%figure;
while (n<K)&&(error>max_error)
    %Ja = [J1 J2 J3 ;J4 J5 J6];
    
%   Value substituition

    Ja =  subs(Ja, {theta1,theta2,theta3},{thetai1,thetai2,thetai3});
    J = double(Ja);
    xi = subs(Xe,{theta1,theta2,theta3},{thetai1,thetai2,thetai3});
    Xi = double(xi);
    yi = subs(Ye,{theta1,theta2,theta3},{thetai1,thetai2,thetai3});
    Yi = double(yi);
    zi = subs(Ze,{theta1,theta2,theta3},{thetai1,thetai2,thetai3});
    Zi = double(zi);
    
%   Jacobian Pseudo inverse

    pseudoinverseJ = (inv(J'*J))*(J');
    
%   NR_angle calculator
    theta_N = [thetai1;thetai2;thetai3]-(pseudoinverseJ*[Xi-xe;Yi-ye;Zi-ze]);
    
    thetai1 = theta_N(1);
    thetai2 = theta_N(2);
    thetai3 = theta_N(3);
    
%     Visualization
    x = a(1).*cosd(thetai1);
    y = a(1).*sind(thetai1);
   
    x1 = x+(a(2).*cosd(thetai2));
    y1 = y+(a(2).*sind(thetai2));
    
    x2  = x1+(a(3).*cosd(thetai3));
    y2 = y1+(a(3).*sind(thetai3));
    
    
%   error calculation 
    endeff_pos = [x2;y2];
    goal_pos = [xe;ye];
    error = sqrt(immse(endeff_pos,goal_pos)); 
    o(n) = error;
    
%     plot([0,x],[0,y]); 
%     hold on;
%     plot([x,x1],[y,y1]); 
%     hold on;
%     plot([x1,x2],[y1,y2]);
%     hold on;
%     % plot([x2,x3],[y2,y3]); 
%     % hold on;
%     scatter(xe,ye);
%     axis([-105 105 -105 105]);
%     pause(.05)
%     clf;
    
    n=n+1;

   
 
end

% Final Visualization
figure;
plot([0,x],[0,y]); 
hold on;
plot([x,x1],[y,y1]); 
hold on;
plot([x1,x2],[y1,y2]);
hold on;
% plot([x2,x3],[y2,y3]); 
% hold on;
scatter(xe,ye);
axis([-105 105 -105 105]);

