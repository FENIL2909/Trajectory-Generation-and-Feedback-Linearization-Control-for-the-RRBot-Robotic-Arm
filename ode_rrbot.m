%% ode_rrbot

function dX = ode_rrbot(t,X)

M1 = 1; %Kg
M2 = 1; %Kg
L1 = 1; %m
L2 = 1; %m
r1 = 0.45; %m
r2 = 0.45; %m
I1 = 0.084; %Kg.m2
I2 = 0.084; %Kg.m2
g = 9.81; %m/s2

dX = zeros(4,1);
X = num2cell(X);
[theta1, theta2, theta1_dot, theta2_dot] = deal(X{:});

% Obtained by EigenValue placement for the virtual control input

% Overall Control Law
% Instead of evaluating the cubic polynomical trajectory here over-and-over
% again the overall control in terms of the state variables is substituted
% here
%U = [- (8829*sin(theta1 + theta2))/2000 - (28449*sin(theta1))/2000 - ((9*cos(theta2))/10 + 1573/1000)*((3*t)/4503599627370496 + 12*theta1 + 7*theta1_dot - (597*pi)/50 + (51*pi*t)/125 + (159*pi*t^2)/500 - (3*pi*t^3)/125 + 7/18014398509481984) - ((9*cos(theta2))/20 + 573/2000)*(t/18014398509481984 + 2*theta2 + 3*theta2_dot - (97*pi)/100 + (21*pi*t)/250 + (21*pi*t^2)/1000 - (pi*t^3)/500 + 3/36028797018963968) - (9*theta2_dot*sin(theta2)*(2*theta1_dot + theta2_dot))/20;
 %                             (55581*pi)/200000 - (573*theta2)/1000 - (1719*theta2_dot)/2000 - (573*t)/36028797018963968000 - (8829*sin(theta1 + theta2))/2000 + (9*theta1_dot^2*sin(theta2))/20 - (12033*pi*t)/500000 - (12033*pi*t^2)/2000000 + (573*pi*t^3)/1000000 - ((9*cos(theta2))/20 + 573/2000)*((3*t)/4503599627370496 + 12*theta1 + 7*theta1_dot - (597*pi)/50 + (51*pi*t)/125 + (159*pi*t^2)/500 - (3*pi*t^3)/125 + 7/18014398509481984) - 1719/72057594037927936000];

U = [- (8829*sin(theta1 + theta2))/2000 - (28449*sin(theta1))/2000 - ((9*cos(theta2))/20 + 573/2000)*(t/4503599627370496 + 8*theta2 + 6*theta2_dot - (397*pi)/100 + (87*pi*t)/500 + (51*pi*t^2)/500 - (pi*t^3)/125 + 3/18014398509481984) - ((9*cos(theta2))/10 + 1573/1000)*((3*t)/1125899906842624 + 48*theta1 + 14*theta1_dot - (2397*pi)/50 + (207*pi*t)/250 + (339*pi*t^2)/250 - (12*pi*t^3)/125 + 7/9007199254740992) - (9*theta2_dot*sin(theta2)*(2*theta1_dot + theta2_dot))/20;
                              (227481*pi)/200000 - (573*theta2)/250 - (1719*theta2_dot)/1000 - (573*t)/9007199254740992000 - (8829*sin(theta1 + theta2))/2000 + (9*theta1_dot^2*sin(theta2))/20 - (49851*pi*t)/1000000 - (29223*pi*t^2)/1000000 + (573*pi*t^3)/250000 - ((9*cos(theta2))/20 + 573/2000)*((3*t)/1125899906842624 + 48*theta1 + 14*theta1_dot - (2397*pi)/50 + (207*pi*t)/250 + (339*pi*t^2)/250 - (12*pi*t^3)/125 + 7/9007199254740992) - 1719/36028797018963968000];

tau1 = U(1);
tau2 = U(2);

dX(1) = theta1_dot;
dX(2) = theta2_dot;
dX(3) = (I2*tau1 - I2*tau2 + M2*r2^2*tau1 - M2*r2^2*tau2 + L1*M2^2*g*r2^2*sin(theta1) + I2*L1*M2*g*sin(theta1) + I2*M1*g*r1*sin(theta1) - L1*M2*r2*tau2*cos(theta2) + L1*M2^2*r2^3*theta1_dot^2*sin(theta2) + L1*M2^2*r2^3*theta2_dot^2*sin(theta2) + L1^2*M2^2*r2^2*theta1_dot^2*cos(theta2)*sin(theta2) - L1*M2^2*g*r2^2*sin(theta1 + theta2)*cos(theta2) + I2*L1*M2*r2*theta1_dot^2*sin(theta2) + I2*L1*M2*r2*theta2_dot^2*sin(theta2) + M1*M2*g*r1*r2^2*sin(theta1) + 2*L1*M2^2*r2^3*theta1_dot*theta2_dot*sin(theta2) + 2*I2*L1*M2*r2*theta1_dot*theta2_dot*sin(theta2))/(- L1^2*M2^2*r2^2*cos(theta2)^2 + L1^2*M2^2*r2^2 + I2*L1^2*M2 + M1*M2*r1^2*r2^2 + I1*M2*r2^2 + I2*M1*r1^2 + I1*I2);
dX(4) = -(I2*tau1 - I1*tau2 - I2*tau2 - L1^2*M2*tau2 - M1*r1^2*tau2 + M2*r2^2*tau1 - M2*r2^2*tau2 - L1^2*M2^2*g*r2*sin(theta1 + theta2) + L1*M2^2*g*r2^2*sin(theta1) - I1*M2*g*r2*sin(theta1 + theta2) + I2*L1*M2*g*sin(theta1) + I2*M1*g*r1*sin(theta1) + L1*M2*r2*tau1*cos(theta2) - 2*L1*M2*r2*tau2*cos(theta2) + L1*M2^2*r2^3*theta1_dot^2*sin(theta2) + L1^3*M2^2*r2*theta1_dot^2*sin(theta2) + L1*M2^2*r2^3*theta2_dot^2*sin(theta2) + 2*L1^2*M2^2*r2^2*theta1_dot^2*cos(theta2)*sin(theta2) + L1^2*M2^2*r2^2*theta2_dot^2*cos(theta2)*sin(theta2) - L1*M2^2*g*r2^2*sin(theta1 + theta2)*cos(theta2) + L1^2*M2^2*g*r2*cos(theta2)*sin(theta1) - M1*M2*g*r1^2*r2*sin(theta1 + theta2) + I1*L1*M2*r2*theta1_dot^2*sin(theta2) + I2*L1*M2*r2*theta1_dot^2*sin(theta2) + I2*L1*M2*r2*theta2_dot^2*sin(theta2) + M1*M2*g*r1*r2^2*sin(theta1) + 2*L1*M2^2*r2^3*theta1_dot*theta2_dot*sin(theta2) + 2*L1^2*M2^2*r2^2*theta1_dot*theta2_dot*cos(theta2)*sin(theta2) + L1*M1*M2*r1^2*r2*theta1_dot^2*sin(theta2) + 2*I2*L1*M2*r2*theta1_dot*theta2_dot*sin(theta2) + L1*M1*M2*g*r1*r2*cos(theta2)*sin(theta1))/(- L1^2*M2^2*r2^2*cos(theta2)^2 + L1^2*M2^2*r2^2 + I2*L1^2*M2 + M1*M2*r1^2*r2^2 + I1*M2*r2^2 + I2*M1*r1^2 + I1*I2);
end