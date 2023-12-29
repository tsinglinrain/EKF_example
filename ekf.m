%%
clc;
clear;
%%
uav1 = load('./data/uav1.txt');
uav2 = load('./data/uav2.txt');
uav3 = load('./data/uav3.txt');
target = load('./data/target.txt');
measurement = load('./data/measurement.txt');
measurement = measurement';
%% 
T = 0.1;
alpha = 1/60;
I3 = eye(3);
I9 = eye(9);
O3= zeros(3);
O31 = [0; 0; 0];
N = 401;

sigma = 0.5 * pi/180;
R = sigma^2 * eye(6);

a_max = 15;
%%
X0 = zeros(9, N);
X0(:, 1) = [653.42, 567.81, 590.61, 89.93, 61.87, 84.71, 3.64, 5.43, 4.81];
P0(:, 1) = [500, 500, 500, 50, 50, 50, 5, 5, 5];
P0 = P0*P0';
%%
Phi = [I3   T*I3   ((-1 + alpha*T + exp(-alpha*T))/alpha^2)*I3;  ...
       O3    I3    ((1 - exp(-alpha*T))/alpha)*I3;  ...
       O3    O3    exp(-alpha*T)*I3]; 
 
U = [(T^2/2 - (alpha*T - 1 + exp(-alpha*T))/alpha^2)*I3; ...
     (T - (1 - exp(-alpha*T))/alpha)*I3; ...
     (1 - exp(-alpha*T))*I3];
%% 迭代
for i = 2 : N 
    Xn = Phi*X0(:, i-1) + U*X0(7: 9, i-1);  % 状态一步预测
    Q_matrix = Q_func(alpha, T, X0(7:9), a_max);
    Pn = Phi*P0*Phi' + Q_matrix;   % 一步预测误差协方差矩阵

    pos1 = Xn(1: 3, 1) - uav1(i, 2: 4)'; 
    pos2 = Xn(1: 3, 1) - uav2(i, 2: 4)';
    pos3 = Xn(1: 3, 1) - uav3(i, 2: 4)';

    H1 = jacobi_func(pos1(1, 1), pos1(2, 1), pos1(3, 1));
    H2 = jacobi_func(pos2(1, 1), pos2(2, 1), pos2(3, 1));
    H3 = jacobi_func(pos3(1, 1), pos3(2, 1), pos3(3, 1));
    H_true = [H1; H2; H3];

    H = zeros(6, 9);    
    H(1: 6, 1: 3) = H_true;
    S = H*Pn*H' + R;
    K = Pn*H'*inv(S);

    h1 = h_func(pos1(1, 1), pos1(2, 1), pos1(3, 1));
    h2 = h_func(pos2(1, 1), pos2(2, 1), pos2(3, 1));
    h3 = h_func(pos3(1, 1), pos3(2, 1), pos3(3, 1));
    h_total = [h1; h2; h3];

    difference = measurement(1: 6, i) - h_total;
    X0(:, i) = Xn + K*difference;
    P0 = (I9 - K*H)*Pn*(I9 - K*H)' + K*R*K';
end
%% 
X0 = X0';
%%
target(:, [3, 5]) = target(:, [5, 3]);
target(:, [4, 8]) = target(:, [8, 4]);
target(:, [7, 9]) = target(:, [9, 7]);
error = X0 - target(:, 2: 10);
t = target(:, 1);

%%
figure(1)
set(gcf, 'color', 'w');
plot3(X0(:, 1), X0(:, 2), X0(:, 3), 'b', 'linewidth', 1)
hold on
plot3(target(:, 2), target(:, 3), target(:, 4), 'r', 'linewidth', 1)
title('估计轨迹与实际轨迹');
xlabel('X轴(m)');
ylabel('Y轴(m)');
zlabel('Z轴(m)');
legend('目标估计轨迹', '目标真实轨迹');
%% 
figure(2)
set(gcf,'color','w');
subplot(311)
plot(t, X0(:, 1), 'b', 'LineWidth', 1)
hold on
plot(t, target(:, 2), 'r', 'linewidth', 1)
xlabel('time(sec)');
ylabel('X轴(m)');
legend('目标估计轨迹','目标真实轨迹');
title('三个方向的真实位置与估计位置');
subplot(312)
plot(t,X0(:,2),'b','LineWidth',1)
hold on
plot(t, target(:, 3),'r','linewidth',1)
xlabel('time(sec)');
ylabel('Y轴 (m)');
legend('目标估计轨迹','目标真实轨迹');
subplot(313)
plot(t,X0(:,3),'b','LineWidth',1)
hold on
plot(t, target(:, 4),'r','linewidth',1)
xlabel('time(sec)');
ylabel('Z轴(m)');
legend('目标估计轨迹','目标真实轨迹');
%% 
figure(3)
set(gcf, 'color', 'w');
subplot(311)
plot(t, error(:, 1), 'b')
xlabel('time(sec)');
ylabel('X轴误差(m)');
grid on;
title('三个方向的位置误差');
subplot(312)
plot(t, error(:,2), 'b')
xlabel('time(sec)');
ylabel('Y轴位置误差(m)');
grid on;
subplot(313)
plot(t, error(:,3), 'b')
xlabel('time(sec)');
ylabel('Z轴位置误差(m)');
grid on;
%%
figure(4)
set(gcf, 'color', 'w');
subplot(311)
plot(t, error(:, 4),'b')
xlabel('time(sec)');
ylabel('X轴速度误差(m/s)');
title('三轴误差');
grid on;
subplot(312)
plot(t, error(:, 5), 'b')
xlabel('time(sec)');
ylabel('Y轴速度误差(m/s)');
grid on;
subplot(313)
plot(t, error(:, 6), 'b')
xlabel('time(sec)');
ylabel('Z轴速度误差 (m/s)');
grid on;
%%
figure(5)
set(gcf, 'color', 'w');
subplot(311)
plot(t, error(:, 7),'b')
xlabel('time(sec)');
ylabel('X轴加速度误差(m/s)');
title('三轴误差');
grid on;
subplot(312)
plot(t, error(:, 8), 'b')
xlabel('time(sec)');
ylabel('Y轴加速度误差(m/s)');
grid on;
subplot(313)
plot(t, error(:, 9), 'b')
xlabel('time(sec)');
ylabel('Z轴加速度误差(m/s)');
grid on;
%%
function jacobi_h = jacobi_func(x_rs, y_rs, z_rs)
    jacobi_h = [- (x_rs * y_rs) / ((x_rs^2 + y_rs^2 + z_rs^2) * sqrt(x_rs^2 + z_rs^2)), ...
                sqrt(x_rs^2 + z_rs^2) / (x_rs^2 + y_rs^2 + z_rs^2), ...
                - (z_rs * y_rs) / ((x_rs^2 + y_rs^2 + z_rs^2) * sqrt(x_rs^2 + z_rs^2)); ...
                z_rs / (x_rs^2 + z_rs^2), ... 
                0, ...
                - x_rs / (x_rs^2 + z_rs^2)];
end
%%
function h = h_func(x_rs, y_rs, z_rs)
    h = [asin(y_rs / sqrt(x_rs^2 + y_rs^2 + z_rs^2)); ... 
         atan2(-z_rs, x_rs)];
end
%% 
function Q_matrix = Q_func(alpha, T, ap0, a_max)
    q11 = (2*alpha^3*T^3-6*alpha^2*T^2+6*alpha*T+3-12*alpha*T*exp(-alpha*T)-3*exp(-2*alpha*T))/(6*alpha^5);
    q12 = (alpha^2*T^2-2*alpha*T+1-2*(1-alpha*T)*exp(-alpha*T)+exp(-2*alpha*T))/(2*alpha^4);
    q21 = q12;
    q22 = (2*alpha*T-3+4*exp(-alpha*T)-exp(-2*alpha*T))/(2*alpha^3);
    q13 = (1-2*alpha*T*exp(-alpha*T)-exp(-2*alpha*T))/(2*alpha^3);
    q31 = q13;
    q23 = (1-2*exp(-alpha*T)+exp(-2*alpha*T))/(2*alpha^3);
    q32 = q23;
    q33 = (1-exp(-2*alpha*T))/(2*alpha);
    ap0x = ap0(1);
    ap0y = ap0(2);
    ap0z = ap0(3);
    if ap0x >= 0 && ap0x < a_max
        sigma_2x = ((4-pi)/pi)*(a_max-ap0x)^2;
    elseif ap0x < 0 && ap0x > -a_max
        sigma_2x = ((4-pi)/pi)*(a_max+ap0x)^2;
    end
    if ap0y >= 0 && ap0y < a_max
        sigma_2y = ((4-pi)/pi)*(a_max-ap0y)^2;
    elseif ap0y < 0 && ap0y > -a_max
        sigma_2y = ((4-pi)/pi)*(a_max+ap0y)^2;
    end
    if ap0z >= 0 && ap0z < a_max
        sigma_2z = ((4-pi)/pi)*(a_max-ap0z)^2;
    elseif ap0z < 0 && ap0z > -a_max
        sigma_2z = ((4-pi)/pi)*(a_max+ap0z)^2;
    end
    sig_2 = diag([sigma_2x, sigma_2y, sigma_2z]);   %过程噪声标准差sigma
    Q_matrix = 2 * alpha * [q11*sig_2, q12*sig_2, q13*sig_2;...
                          q21*sig_2, q22*sig_2, q23*sig_2;...
                          q31*sig_2, q32*sig_2, q33*sig_2];
end