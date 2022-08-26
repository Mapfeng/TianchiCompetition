clc;
clear;

%{
1.程序编写日期：2022.08.22
2.程序说明：接收Local Planner规划出来的期望轨迹，通过控制进行跟踪
3.程序输入：
参考信息：参考轨迹点[X_ref,Y_ref]，以及参考点的速度V_ref，横摆角phi_ref，前轮转角delta_f_ref
ego车信息：后轴中心位置[X,Y]，车辆朝向phi，后轴中心纵向速度V，当前前轮转角delta_f，轴距L
4.程序输出：前轮转角delta_f，纵向加速度accel
%}

% 状态量[X,Y,phi,V]，控制量[delta_f,accel]

%% 参数设定
Nx = 4; % 状态量维度
Nu = 2; % 输入量维度
Np = 60; % 预测时域
Nc = 30; % 控制时域
st = 0.02; % 仿真步长
T = 0.02; % 采样周期
Row = 10; % 松弛因子
%*******************************************************************
Q = 1*kron(eye(Np),diag([1 1 1 1])); % 分别代表X,Y，phi和V的权重[Np×Nx,Np×Nx]
R = 1*kron(eye(Nc),diag([1 5])); % 分别代表delta_f和a的权重[Nc×Nu,Nc×Nu]
%*******************************************************************
% 用于观测轨迹和参数变化
X_final = zeros(2501,1);
Y_final = zeros(2501,1);
V_final = zeros(2501,1);
a_final = zeros(2501,1);

%% ego车初始状态
X = 0; % m
Y = 0; % m
phi = 0; % rad
V = 20; % m/s
delta_f = 0; % rad
beta = atan(1/2*tan(delta_f)); % rad
accel = 0; % m/s^2
L = 2.6; % m
U = zeros(2,1); % 控制量[2×1]
index = 1;
X_final(index) = X; 
Y_final(index) = Y;
V_final(index) = V;
a_final(index) = accel;

for time = st:st:50
    % 更新车辆状态
    beta = atan(tan(delta_f)/2);
    V = V + accel * st;
    phi = phi + 2 * V * sin(beta) / L * st;
    X = X + V * cos(phi + beta) * st;
    Y = Y + V * sin(phi + beta) * st;
    index = index + 1;
    X_final(index) = X;
    Y_final(index) = Y;
    V_final(index) = V;
    a_final(index) = accel;
    
    % 设定参考信息
%     % 直线轨迹
%     V_ref = 20;
%     X_ref = V_ref * time;
%     Y_ref = 50;
%     phi_ref = 0;
%     delta_f_ref = 0;
%     accel_ref = 0;

    % 半径为100m的圆形轨迹, 圆心为(0, 100), 速度为20m/s
    V_ref = 20; 
    X_ref = 100 * sin(0.2 * time); 
    Y_ref = 100 - 100 * cos(0.2 * time);
    phi_ref = 0.2 * time; 
    delta_f_ref = 0.104; 
    accel_ref = 0;
    while(phi_ref<-2*pi||phi_ref>2*pi)
        if (phi_ref < -2*pi)
            phi_ref = phi_ref + 2*pi;
        end
        if (phi_ref > 2*pi)
            phi_ref = phi_ref - 2*pi;
        end
    end


    % 数据初始化，得到增广状态量kesi[6×1]
    [kesi] = DataInitialization(Nx, Nu, X, Y, phi, V, delta_f, accel, X_ref, Y_ref, phi_ref, V_ref, delta_f_ref, accel_ref, U);

    % 构造预测方程的参数矩阵PHI[Np×Nx,Nx+Nu]，THETA[Np×Nx,Nc×Nu]
    [A,B,C,PHI,THETA] = MatrixConstruction(Nx, Nu, Np, Nc, V_ref, phi_ref, delta_f_ref, L, T);

    % 构造约束条件的参数矩阵H[Nc×Nu+1,Nc×Nu+1],G[Nc×Nu+1,1],A_cons[2×Nc×Nu,Nc×Nu+1],b_cons[2×Nc×Nu,1],lb[Nc×Nu+1,1],ub[Nc×Nu+1,1]
    [H,G,A_cons,b_cons,lb,ub]=ConstraintConstruction(Nx, Nu, Np, Nc, PHI, THETA, Q, R, Row, kesi, U);

    % 求解过程
%     options = optimset('Algorithm','interior-point-convex');
%     options = optimset('Algorithm','active-set'); 
    warning off all   
%     [AA,fval,exitflag] = quadprog(H,G,A_cons,b_cons,[],[],lb,ub,[],options);
    [AA,fval,exitflag] = quadprog(H, G, A_cons, b_cons, [], [], lb, ub, []);

    % 计算输出
    u_piao(1) = AA(1); % delta(delta_f - delta_f_ref)
    u_piao(2) = AA(2); % delta(accel - accel_ref)
    U(1) = kesi(5) + u_piao(1); % delta_f - delta_f_ref
    U(2) = kesi(6) + u_piao(2); % accel - accel_ref
    delta_f = U(1) + delta_f_ref; % k+1时刻的delta_f
    accel = U(2) + accel_ref; % k+1时刻的accel
end

%% 作图
figure
plot(X_final,Y_final)
figure
plot(V_final)
figure
plot(a_final)


%% 数据初始化子函数
function [kesi] = DataInitialization(Nx, Nu, X, Y, phi, V, X_ref, Y_ref, phi_ref, V_ref, U)
    kesi = zeros(Nx+Nu,1); 
    kesi(1) = X - X_ref;
    kesi(2) = Y - Y_ref;
    heading_offset = phi - phi_ref;
    while(heading_offset < -pi)
        heading_offset = heading_offset + 2*pi;
    end
    while(heading_offset > pi)
        heading_offset = heading_offset - 2*pi;
    end
    kesi(3) = heading_offset;
    kesi(4) = V - V_ref;
    kesi(5) = U(1);
    kesi(6) = U(2);
end


%% 构造预测方程的参数矩阵子函数
function [A,B,C,PHI,THETA] = MatrixConstruction(Nx, Nu, Np, Nc, V_ref, phi_ref, delta_f_ref, L, T)
    beta_ref = atan(tan(delta_f_ref)/2);
    a = [
        1   0   -V_ref*sin(phi_ref+beta_ref)*T   cos(phi_ref+beta_ref)*T
        0   1    V_ref*cos(phi_ref+beta_ref)*T   sin(phi_ref+beta_ref)*T
        0   0                                1       2*sin(beta_ref)*T/L
        0   0                                0                         1
    ];
    b = [
        -V_ref*sin(phi_ref+beta_ref)*(2/(3*(cos(delta_f_ref))^2+1))*T    0
         V_ref*cos(phi_ref+beta_ref)*(2/(3*(cos(delta_f_ref))^2+1))*T    0
             2*V_ref*cos(beta_ref)/L*(2/(3*(cos(delta_f_ref))^2+1))*T    0
                                                                    0    1
    ];
    A_cell = cell(2,2);
    B_cell = cell(2,1);
    A_cell{1,1} = a;
    A_cell{1,2} = b;
    A_cell{2,1} = zeros(Nu,Nx);
    A_cell{2,2} = eye(Nu);
    B_cell{1,1} = b;
    B_cell{2,1} = eye(Nu);
    A = cell2mat(A_cell);
    B = cell2mat(B_cell);
    C = [
        1 0 0 0 0 0
        0 1 0 0 0 0
        0 0 1 0 0 0
        0 0 0 1 0 0
    ];
    PHI_cell = cell(Np,1);
    THETA_cell = cell(Np,Nc);
    for i = 1:Np
        PHI_cell{i,1} = C*A^i;
        for j = 1:Nc
            if i >= j
                THETA_cell{i,j} = C*A^(i-j)*B;
            else
                THETA_cell{i,j} = zeros(Nx,Nu);
            end
        end
    end
    PHI = cell2mat(PHI_cell);
    THETA = cell2mat(THETA_cell);
end


%% 构造约束条件的参数矩阵子函数
function [H,G,A_cons,b_cons,lb,ub]=ConstraintConstruction(Nx, Nu, Np, Nc, PHI, THETA, Q, R, Row, kesi, U)
    % 优化目标：加速度变化量 => 0，前轮转角变化量 => 0，跟踪误差 => 0
    H_cell = cell(2,2);
    H_cell{1,1} = THETA'*Q*THETA+R;
    H_cell{1,2} = zeros(Nu*Nc,1);
    H_cell{2,1} = zeros(1,Nu*Nc);
    H_cell{2,2} = Row;
    Ep = PHI*kesi;
    G_cell = cell(1,2);
    G_cell{1,1} = 2*Ep'*Q*THETA;
    G_cell{1,2} = 0;
    H = cell2mat(H_cell);
    G = cell2mat(G_cell);
    H = (H + H')/2;
    G = G';
    % 不等式约束处理
    A_t = zeros(Nc,Nc);
    for i = 1:Nc
        for j = 1:Nc
            if i >= j
                A_t(i,j) = 1;
            else
                A_t(i,j) = 0;
            end
        end
    end
    A_I = kron(A_t,eye(Nu));
    Ut = kron(ones(Nc,1),U); % 上一时刻的控制量
    umin = [-0.889;-2]; % u = [delta_f - delta_f_ref;accel - accel_ref]
    umax = [0.681;2];
    delta_umin = [-0.01;-0.02];
    delta_umax = [0.01;0.02];
    Umin = kron(ones(Nc,1),umin);
    Umax = kron(ones(Nc,1),umax);
    % 加入松弛因子后的约束系数
    A_cons_cell = {A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell = {Umax-Ut;Ut-Umin};
    A_cons = cell2mat(A_cons_cell);
    b_cons = cell2mat(b_cons_cell);
    % 状态量约束
    delta_Umin = kron(ones(Nc,1),delta_umin);
    delta_Umax = kron(ones(Nc,1),delta_umax);
    lb = [delta_Umin;0];
    ub = [delta_Umax;10];
end


