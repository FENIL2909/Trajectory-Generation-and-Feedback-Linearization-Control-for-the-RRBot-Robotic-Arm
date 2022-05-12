%% System Parameters

M1 = 1; %Kg
M2 = 1; %Kg
L1 = 1; %m
L2 = 1; %m
r1 = 0.45; %m
r2 = 0.45; %m
I1 = 0.084; %Kg.m2
I2 = 0.084; %Kg.m2
g = 9.81; %m/s2

syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot tau1 tau2 g 'real'
syms M1 M2 I1 I2 L1 L2 r1 r2 'real'
syms x1_dot y1_dot x2_dot y2_dot 'real'
syms t 'real'

%% State Space Representation
X = sym('X', [4,1]);
X(1) = theta1;
X(2) = theta2;
X(3) = theta1_dot;
X(4) = theta2_dot;

%% Dynamic Equations

x1_dot = (theta1_dot)*(r1)*(cos(theta1));
y1_dot = -(theta1_dot)*(r1)*(sin(theta1));

x2_dot = (theta1_dot)*(L1)*(cos(theta1)) + (theta1_dot + theta2_dot)*(r2)*(cos(theta1 + theta2));
y2_dot = -(theta1_dot)*(L1)*(sin(theta1)) - (theta1_dot + theta2_dot)*(r2)*(sin(theta1 + theta2));

K1 = (1/2)*(I1)*(theta1_dot*theta1_dot) + (1/2)*(M1)*((x1_dot*x1_dot) + (y1_dot*y1_dot));
K2 = (1/2)*(I2)*((theta2_dot + theta1_dot)*(theta2_dot + theta1_dot)) + (1/2)*(M2)*((x2_dot*x2_dot) + (y2_dot*y2_dot));

P1 = M1*g*r1*cos(theta1);
P2 = M2*g*(L1*(cos(theta1)) + r2*(cos(theta1 + theta2)));
L = K1 + K2 - P1 - P2;

u = [tau1;tau2];
q = [theta1;theta2];
dq = [theta1_dot; theta2_dot];
ddq = [theta1_ddot; theta2_ddot];

DL_Dq = gradient(L,q);  % used gradient instead of jacobian to keep matrix size consistent
DL_Ddq = gradient(L,dq); % used gradient instead of jacobian to keep matrix size consistent
dDL_dtDdq = jacobian(DL_Ddq,[q;dq])*[dq;ddq];

EOM = simplify(dDL_dtDdq - DL_Dq -u)
%disp(EOM)

EOM_numerical = subs(EOM,[M1,M2,L1,L2,r1,r2,I1,I2,g],[1,1,1,1,0.45,0.45,0.084,0.084,9.81]);



%% Finding g(q)

g_q_symbolic =subs(EOM,[theta1_dot, theta2_dot, theta1_ddot, theta2_ddot, tau1, tau2],[0,0,0,0,0,0]);
fprintf("**************************************************************************************************\n")
fprintf("********** Symbolic Gravity Vector g_q_symbolic **********\n")

disp(simplify(g_q_symbolic))

g_q_numerical =subs(EOM_numerical,[theta1_dot, theta2_dot, theta1_ddot, theta2_ddot, tau1, tau2],[0,0,0,0,0,0]);
fprintf("********** Numerical Gravity Vector g_q_numerical **********\n")
disp(simplify(g_q_numerical))


%% Finding M(q)

M_q_temp_s = subs(EOM,[theta1_dot, theta2_dot, tau1, tau2],[0,0,0,0]) - g_q_symbolic;
fprintf("*************************************************************************************************\n")
fprintf("********** Symbolic Mass Matrix M_q_symbolic **********\n")
M_q_symbolic = jacobian(M_q_temp_s,[theta1_ddot,theta2_ddot]);
disp(simplify(M_q_symbolic))

M_q_temp_n = subs(EOM_numerical,[theta1_dot, theta2_dot, tau1, tau2],[0,0,0,0]) - g_q_numerical;
fprintf("********** Numerical Mass Matrix M_q_numerical **********\n")
M_q_numerical = jacobian(M_q_temp_n,[theta1_ddot,theta2_ddot]);
disp(simplify(M_q_numerical))

%% Finding C(q,q_dot)q_dot  (Whole coriolis term)

C_q_qd_symbolic = subs(EOM - g_q_symbolic - M_q_temp_s,[tau1, tau2],[0,0]);
fprintf("**************************************************************************************************\n")
fprintf("********** Symbolic Coriolis term C_q_qd_symbolic **********\n")
disp(simplify(C_q_qd_symbolic))

C_q_qd_numerical = subs(EOM_numerical - g_q_numerical - M_q_temp_n,[tau1, tau2],[0,0]);
fprintf("********** Numerical Coriolis term C_q_qd_numerical **********\n")
disp(simplify(C_q_qd_numerical))

%% Complete EOM in Manipulator Form
manipulator_EOM_symbolic =  M_q_symbolic*([theta1_ddot,theta2_ddot]') + C_q_qd_symbolic + g_q_symbolic;
manipulator_EOM_numerical =  M_q_numerical*([theta1_ddot,theta2_ddot]') + C_q_qd_numerical + g_q_numerical;

fprintf("**************************************************************************************************\n")
fprintf("********** Symbolic EOM in manipulator form **********\n")
disp(simplify(manipulator_EOM_symbolic))

fprintf("********** Numerical EOM in manipulator form **********\n")
disp(simplify(manipulator_EOM_numerical))

%% Virtual Control input design

A = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
B = [0 0; 0 0; 1 0; 0 1];


lambda = [-2 -4 -6 -8];
fprintf("**************************************************************************************************\n")
fprintf("********** K Gains **********")
K = place(A,B,lambda)


%% Feedback Lineareization Control

% Received from the Trajectory Generation
theta1_desired = (pi*t^3)/500 - (3*pi*t^2)/100 - t/18014398509481984 + pi;
theta2_desired = (pi*t^3)/1000 - (3*pi*t^2)/200 - t/36028797018963968 + pi/2;

theta1_dot_desired = (3*pi*t^2)/500 - (3*pi*t)/50 - 1/18014398509481984;
theta2_dot_desired = (3*pi*t^2)/1000 - (3*pi*t)/100 - 1/36028797018963968;

theta1_ddot_desired = (3*pi*t)/250 - (3*pi)/50;
theta2_ddot_desired = (3*pi*t)/500 - (3*pi)/100;

X = [theta1; theta2; theta1_dot; theta2_dot];
X_desired = [(theta1_desired); theta2_desired; theta1_dot_desired; theta2_dot_desired];
feed_foward_input = [theta1_ddot_desired; theta2_ddot_desired];

M_q = [(9*cos(theta2))/10 + 1573/1000, (9*cos(theta2))/20 + 573/2000;
       (9*cos(theta2))/20 + 573/2000,                      573/2000];

C_q_qd = [-(9*theta2_dot*sin(theta2)*(2*theta1_dot + theta2_dot))/20;
                           (9*theta1_dot^2*sin(theta2))/20];

g_q = [- (8829*sin(theta1 + theta2))/2000 - (28449*sin(theta1))/2000;
                            -(8829*sin(theta1 + theta2))/2000];

fprintf("**************************************************************************************************\n")
fprintf("********** Overall Control law obtained by feedback linearization **********\n")
global U
U = M_q*(-K*(X-X_desired) + feed_foward_input) + C_q_qd + g_q
%U_ros = M_q*(-K*(X-wrapTo2Pi(X_desired)) + feed_foward_input) + C_q_qd + g_q
