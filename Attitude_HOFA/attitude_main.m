clear;
% Parameters
fs = 200; % Control frequency in Hz
dt = 1/fs;
T = 15; % Total simulation time in seconds
t = 0:1/fs:T; % Time vector

wrapToPi = @(angle) mod(angle + pi, 2*pi) - pi;

dwd1 = 0*t-0.4;
wd1 = cumtrapz(t, dwd1);

dwd2 = 0*t+0.2;
wd2 = cumtrapz(t, dwd2);

dwd3 = 0*t+0.3;
wd3 = cumtrapz(t, dwd3);

dwd = [dwd1', dwd2', dwd3'];
wd = [wd1', wd2', wd3'];

% 初始姿态（单位旋转矩阵）
R = eye(3);

% 初始化姿态矩阵序列
R_sequence = zeros(3, 3, length(t));
R_sequence(:, :, 1) = R;


% 计算每个时刻的姿态
for i = 2:length(t)
    % 当前的角速度
    omega = [wd1(i); wd2(i); wd3(i)];
    
    % 角速度矩阵（反对称矩阵）
    omega_hat = [ 0         -omega(3)  omega(2);
                  omega(3)  0          -omega(1);
                 -omega(2)  omega(1)   0        ];
    
    % 旋转矩阵的更新（使用小角度近似）
    R = R * expm(omega_hat * dt);
    
    % 保存当前的旋转矩阵
    R_sequence(:, :, i) = R;
end


Rd=R_sequence;
eul=rotm2eul(Rd);
Rd3=eul(:,1);
Rd2=eul(:,2);
Rd1=eul(:,3);

Rd1 = wrapToPi(Rd1);
Rd2 = wrapToPi(Rd2);
Rd3 = wrapToPi(Rd3);


J=diag([0.029 0.029 0.055]);

A=[zeros(3,3),eye(3);zeros(3,6)];
B=[zeros(3,3);eye(3)];


% 指定的新极点位置
desired_poles = [-2, -2, -1, -3, -1, -3];

% 设计状态反馈控制律
K = place(A, B, desired_poles);

% % 验证新的极点
% new_eigenvalues = eig(A - B * K);
% disp('New system poles after state feedback:');
% disp(new_eigenvalues);



% Q=diag([ 10 10 3 3 3 1]);
% R=diag([1 1 1])/10;
% % rank(ctrb(A,B));
% sys=ss(A,B,eye(6),zeros(6,3));
% K=lqr(sys,Q,R);
% % sys1=ss(A-B*K,B,eye(6),zeros(6,3));
% % step(sys1)

sim("atti.slx")


subplot(3,3,1)
plot(t,Rd1,tout,outa(:,1))
legend('desired','actual');
xlabel('t'); ylabel('phi/Rad');

subplot(3,3,2)
plot(t,Rd2,tout,outa(:,2))
legend('desired','actual');
xlabel('t'); ylabel('theta/Rad');

subplot(3,3,3)
plot(t,Rd3,tout,outa(:,3))
legend('desired','actual');
xlabel('t'); ylabel('psi/Rad');

subplot(3,3,4)
plot(t,wd1,tout,outw(:,1))
legend('desired','actual');
xlabel('t'); ylabel('w1/Rad/s');

subplot(3,3,5)
plot(t,wd2,tout,outw(:,2))
legend('desired','actual');
xlabel('t'); ylabel('w2/Rad/s');

subplot(3,3,6)
plot(t,wd3,tout,outw(:,3))
legend('desired','actual');
xlabel('t'); ylabel('w3/Rad/s');

subplot(3,3,7)
plot(tout,outM(:,1));
xlabel('t'); ylabel('M1/Nm');

subplot(3,3,8)
plot(tout,outM(:,2));
xlabel('t'); ylabel('M2/Nm');

subplot(3,3,9)
plot(tout,outM(:,3));
xlabel('t'); ylabel('M3/Nm');

