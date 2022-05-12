clear; close; clc;
% ROS Setup
rosinit;
j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client,req,'Timeout',3);

i = 1;

tic;
t = 0;
while(t < 9.750)
t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
% design your state feedback controller in the following

theta1 = wrapTo2Pi(jointData.Position(1));
theta2 = wrapTo2Pi(jointData.Position(2));
theta1_dot = jointData.Velocity(1);
theta2_dot = jointData.Velocity(2);

X = [theta1;theta2;theta1_dot;theta2_dot];

U = [- (8829*sin(theta1 + theta2))/2000 - (28449*sin(theta1))/2000 - ((9*cos(theta2))/20 + 573/2000)*(t/4503599627370496 + 8*theta2 + 6*theta2_dot - (397*pi)/100 + (87*pi*t)/500 + (51*pi*t^2)/500 - (pi*t^3)/125 + 3/18014398509481984) - ((9*cos(theta2))/10 + 1573/1000)*((3*t)/1125899906842624 + 48*theta1 + 14*theta1_dot - (2397*pi)/50 + (207*pi*t)/250 + (339*pi*t^2)/250 - (12*pi*t^3)/125 + 7/9007199254740992) - (9*theta2_dot*sin(theta2)*(2*theta1_dot + theta2_dot))/20;
                             (227481*pi)/200000 - (573*theta2)/250 - (1719*theta2_dot)/1000 - (573*t)/9007199254740992000 - (8829*sin(theta1 + theta2))/2000 + (9*theta1_dot^2*sin(theta2))/20 - (49851*pi*t)/1000000 - (29223*pi*t^2)/1000000 + (573*pi*t^3)/250000 - ((9*cos(theta2))/20 + 573/2000)*((3*t)/1125899906842624 + 48*theta1 + 14*theta1_dot - (2397*pi)/50 + (207*pi*t)/250 + (339*pi*t^2)/250 - (12*pi*t^3)/125 + 7/9007199254740992) - 1719/36028797018963968000];

tau1.Data = U(1);
tau2.Data = U(2);
send(j1_effort,tau1);
send(j2_effort,tau2);

% you can sample data here to be plotted at the end
X1(i) = theta1;
X2(i) = theta2;
X3(i) = theta1_dot;
X4(i) = theta2_dot;
TAU1(i) = U(1);
TAU2(i) = U(2);

time(i) = t;
i = i+1;

end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;

Theta1_desired = (pi*time.^3)/500 - (3*pi*time.^2)/100 - time/18014398509481984 + pi;
Theta2_desired = (pi*time.^3)/1000 - (3*pi*time.^2)/200 - time/36028797018963968 + pi/2;

Theta1_dot_desired = (3*pi*time.^2)/500 - (3*pi*time)/50 - 1/18014398509481984;
Theta2_dot_desired = (3*pi*time.^2)/1000 - (3*pi*time)/100 - 1/36028797018963968;


%% plots
figure
hold on
subplot(2,2,1)
plot(time,X1)
hold on
plot(time, Theta1_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad')
title('theta1')

subplot(2,2,2)
plot(time,X2)
hold on
plot(time, Theta2_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad')
title('theta2')

subplot(2,2,3)
plot(time,X3)
hold on
plot(time, Theta1_dot_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad/s')
title('theta1-dot')

subplot(2,2,4)
plot(time,X4)
hold on
plot(time, Theta2_dot_desired,'Color','red','LineStyle','--')
xlabel('Time step')
ylabel('rad/s')
title('theta2-dot')
hold off

figure
hold on
subplot(2,1,1)
plot(time,TAU1)
xlabel('Time step')
ylabel('Nm')
title('tau1')

subplot(2,1,2)
plot(time,TAU2)
xlabel('Time step')
ylabel('Nm')
title('tau2')
hold off