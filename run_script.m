% Parameter Setting
l_box = 0.12;
h_box = 0.06;
l_sensor = 0.09;
h_sensor = 0.05;
g = 9.81;
beta = pi/180*45;
gamma = beta - atan((h_sensor-(h_box/2))/(l_sensor-(l_box/2)));
R = sqrt((l_sensor-(l_box/2))^2 + (h_sensor-(h_box/2))^2);
u = [-g ; R*cos(gamma);R*sin(gamma)];
% rotation and translation movement
w0 = pi/180*180;
theta0 = pi/180*30;
v0 = 100;
theta_v0 = pi/180*60;
% Time Invariant Kalman Filter (find theta)
delta_t = 0.01;
A_I = [1 delta_t; 0 1];
C_I = [1 0; 0 1];
G_I = [1/2*(delta_t)^2; delta_t];
H_I = 0;
R_I = 0.01;
Q_I = 0.0000001;
% Time Invariant Kalman Filter (find position)
A_V = [eye(2) delta_t*eye(2) zeros(2); zeros(2) eye(2) zeros(2); zeros(2) zeros(2) zeros(2)];
B_V = [0 0 0; 1/2*delta_t^2 0 0; 0 0 0; delta_t 0 0; 0 0 0; 1 0 0];
G_V = [1/2*(delta_t^2)*eye(2); delta_t*eye(2); eye(2)];
H_V = 0;
R_V = 0.01;
Q_V = [0.0000001; 0.0000001];

% run simulink
stoptime = 20; % set stop time of simulink at 20 s
open_system('simulation_system','loadonly'); % open simulation_system.slx
out = sim('simulation_system', stoptime); % out = result from simulink

% set variable to plot graph
t = out.tout;
step = stoptime*2;
pos_cm_world_x = out.pos_cm_world.Data(1:step:end,1);
pos_cm_world_y = out.pos_cm_world.Data(1:step:end,2);
pos_estimate_x = out.pos_estimate.Data(1:step:end,1);
pos_estimate_y = out.pos_estimate.Data(1:step:end,2);
pos_error_x = out.pos_error.Data(:,1);
pos_error_y = out.pos_error.Data(:,2);
theta_box_world = out.theta_box_wolrd.Data(1:step:end);
theta_estiamte = out.theta_estimate.Data(1:step:end);
theta_error = out.theta_error.Data;
acc_x = out.acc.Data(:,1);
acc_y = out.acc.Data(:,2);
acc_imu_estimate_x = out.acc_imu_estimate.Data(:,1);
acc_imu_estimate_y = out.acc_imu_estimate.Data(:,2);
w_gyro = out.w_gyro.Data;
theta_dot_estimate = out.theta_dot_estimate.Data;

% plot graph
% plot ata from gyrocope (measurement) and kalmanfilter (estimation) 
figure
subplot(3,1,1);
plot(t, w_gyro);
title('Gyroscope');
xlabel('time(s)');
ylabel('angular velocity(rad/s)');
ylim([min(w_gyro) max(w_gyro)]);
subplot(3,1,2);
plot(t, theta_dot_estimate, 'r');
title('Estimated Angular Velocity (KF)');
xlabel('time(s)');
ylabel('angular velocity(rad/s)');
ylim([min(w_gyro) max(w_gyro)]);
subplot(3,1,3);
plot(t, w_gyro, t, theta_dot_estimate, 'r');
title('Compare');
xlabel('time(s)');
ylabel('angular velocity(rad/s)');
ylim([min(w_gyro) max(w_gyro)]);
legend('angular vel measurement', 'angular vel estimate')

% plot data from acceleration (measurement) and kalmanfilter (estimation) 
figure
subplot(3,1,1);
plot(t, acc_x);
title('Accelerometer xaxis on IMU');
xlabel('time(s)');
ylabel('acceleration(m/s^2)');
ylim([min(acc_x) max(acc_x(3:end))]);

subplot(3,1,2);
plot(t, acc_imu_estimate_x,'r');
title('Estimated Acceleration xaxis on IMU');
xlabel('time(s)');
ylabel('acceleration(m/s^2)');
ylim([min(acc_x) max(acc_x(3:end))]);

subplot(3,1,3);
plot(t,acc_x, t, acc_imu_estimate_x);
title('Compare xaxis');
xlabel('time(s)');
ylabel('acceleration(m/s^2)');
ylim([min(acc_x) max(acc_x(3:end))]);
legend('acc_x measurement', 'acc_x estimate');

figure
subplot(3,1,1);
plot(t, acc_y);
title('Accelerometer yaxis on IMU');
xlabel('time(s)');
ylabel('acceleration(m/s^2)');
ylim([min(acc_y) max(acc_y)]);

subplot(3,1,2);
plot(t, acc_imu_estimate_y,'r');
title('Estimated Acceleration yaxis on IMU');
xlabel('time(s)');
ylabel('acceleration(m/s^2)');
ylim([min(acc_y) max(acc_y)]);

subplot(3,1,3);
plot(t,acc_y, t,acc_imu_estimate_y);
title('Compare yaxis');
xlabel('time(s)');
ylabel('acceleration(m/s^2)');
ylim([min(acc_y) max(acc_y)]);
legend('acc_y measurement', 'acc_y estimate');

% plot true position and estimated postion
figure
plot(pos_cm_world_x, pos_cm_world_y, 'b+', pos_estimate_x, pos_estimate_y, 'r');
title('Postion of cm (relate world frame)');
xlabel('poistion x(m)');
ylabel('poistion y(m)');
legend('true position', 'estimated postion(KF)');
ylim([0 max(pos_cm_world_y)*1.1]);

% plot true theta and estimated theta
figure
plot(t(1:step:end), theta_box_world, 'b+', t(1:step:end), theta_estiamte, 'r');
title('Theta (relate world frame)');
xlabel('time(s)');
ylabel('theta(rad)');
legend('true theta', 'estimated theta(KF)')
