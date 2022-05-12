
syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot tau1 tau2 g 'real'
syms t 'real'
visualization = true;

M1 = 1; %Kg
M2 = 1; %Kg
L1 = 1; %m
L2 = 1; %m
r1 = 0.45; %m
r2 = 0.45; %m
I1 = 0.084; %Kg.m2
I2 = 0.084; %Kg.m2
g = 9.81; %m/s2

T = 10;
theta1_initial = 200;
theta2_initial = 125;

y0 = [deg2rad(theta1_initial), deg2rad(theta2_initial) 0, 0];

[time,y] = ode45(@ode_rrbot,[0,T],y0);

theta1_desired = (pi*time.^3)/500 - (3*pi*time.^2)/100 - time/18014398509481984 + pi;
theta2_desired = (pi*time.^3)/1000 - (3*pi*time.^2)/200 - time/36028797018963968 + pi/2;

theta1_dot_desired = (3*pi*time.^2)/500 - (3*pi*time)/50 - 1/18014398509481984;
theta2_dot_desired = (3*pi*time.^2)/1000 - (3*pi*time)/100 - 1/36028797018963968;

theta1_ddot_desired = (3*pi*time)/250 - (3*pi)/50;
theta2_ddot_desired = (3*pi*time)/500 - (3*pi)/100;

% Reconstructing the Control Inputs
global U
for i = 1:size(time)
    U1(i) = double(subs(U(1),[theta1, theta2, theta1_dot, theta2_dot,t],[y(i,1),y(i,2),y(i,3),y(i,4),time(i)]));
    U2(i) = double(subs(U(2),[theta1, theta2, theta1_dot, theta2_dot,t],[y(i,1),y(i,2),y(i,3),y(i,4),time(i)]));
end


%% plots
figure
hold on
subplot(2,2,1)
plot(time,y(:,1))
hold on
plot(time, theta1_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad')
title('theta1')

subplot(2,2,2)
plot(time,y(:,2))
hold on
plot(time, theta2_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad')
title('theta2')

subplot(2,2,3)
plot(time,y(:,3))
hold on
plot(time, theta1_dot_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad/s')
title('theta1-dot')

subplot(2,2,4)
plot(time,y(:,4))
hold on
plot(time, theta2_dot_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad/s')
title('theta2-dot')
hold off

figure
hold on
subplot(2,1,1)
plot(time,U1)
xlabel('Time step')
ylabel('Nm')
title('tau1')

subplot(2,1,2)
plot(time,U2)
xlabel('Time step')
ylabel('Nm')
title('tau2')
hold off
pause(10)
close all

%% visualization
figure
if(visualization)

    theta1_desired_plot = (pi*time.^3)/500 - (3*pi*time.^2)/100 - time/18014398509481984 + pi;
    theta2_desired_plot = (pi*time.^3)/1000 - (3*pi*time.^2)/200 - time/36028797018963968 + pi/2;
   
    l1 = 1;
    l2 = 1;

    x1_desired_plot = l1*sin(theta1_desired_plot);
    y1_desired_plot = l1*cos(theta1_desired_plot);

    x2_desired_plot = l1*sin(theta1_desired_plot) + l2*sin(theta1_desired_plot+theta2_desired_plot);
    y2_desired_plot = l1*cos(theta1_desired_plot) + l2*cos(theta1_desired_plot+theta2_desired_plot);
    

    x1_pos= l1*sin(y(:,1));
    x2_pos= l1*sin(y(:,1)) + l2*sin(y(:,1)+y(:,2));
    y1_pos= l1*cos(y(:,1));
    y2_pos= l1*cos(y(:,1)) + l2*cos(y(:,1)+y(:,2));

    for i=1:size(y)
        plot(x1_desired_plot, y1_desired_plot,'Color','red','LineStyle','--')
        hold on
        plot(x2_desired_plot, y2_desired_plot,'Color','red','LineStyle','--')
        hold on
        plot([0 x1_pos(i) x2_pos(i)],[0 y1_pos(i) y2_pos(i)],'blue', 'LineWidth',4.0)
        hold on
        plot(x2_pos(1:i),y2_pos(1:i),'blue')
        hold on
        plot(x1_pos(1:i),y1_pos(1:i),'blue')
        hold on
        plot(x1_pos(i),y1_pos(i),'.','MarkerSize',24.0)
        xlim([-2.25 2.25])
        ylim([-2.25 2.25])
        pause(0.000075)
        hold off
    end
end
