clc;
clear;

%{
1.�����д���ڣ�2022.08.22
2.����˵��������Local Planner�滮�����������켣��ͨ�����ƽ��и���
3.�������룺
�ο���Ϣ���ο��켣��[X_ref,Y_ref]���Լ��ο�����ٶ�V_ref����ڽ�phi_ref��ǰ��ת��delta_f_ref
ego����Ϣ����������λ��[X,Y]����������phi���������������ٶ�V����ǰǰ��ת��delta_f�����L
4.���������ǰ��ת��delta_f��������ٶ�accel
%}

% ״̬��[X,Y,phi,V]��������[delta_f,accel]

%% �����趨
Nx = 4; % ״̬��ά��
Nu = 2; % ������ά��
Np = 60; % Ԥ��ʱ��
Nc = 30; % ����ʱ��
st = 0.02; % ���沽��
T = 0.02; % ��������
Row = 10; % �ɳ�����
%*******************************************************************
Q = 1*kron(eye(Np),diag([1 1 1 1])); % �ֱ����X,Y��phi��V��Ȩ��[Np��Nx,Np��Nx]
R = 1*kron(eye(Nc),diag([1 5])); % �ֱ����delta_f��a��Ȩ��[Nc��Nu,Nc��Nu]
%*******************************************************************
% ���ڹ۲�켣�Ͳ����仯
X_final = zeros(2501,1);
Y_final = zeros(2501,1);
V_final = zeros(2501,1);
a_final = zeros(2501,1);

%% ego����ʼ״̬
X = 0; % m
Y = 0; % m
phi = 0; % rad
V = 20; % m/s
delta_f = 0; % rad
beta = atan(1/2*tan(delta_f)); % rad
accel = 0; % m/s^2
L = 2.6; % m
U = zeros(2,1); % ������[2��1]
index = 1;
X_final(index) = X; 
Y_final(index) = Y;
V_final(index) = V;
a_final(index) = accel;

for time = st:st:50
    % ���³���״̬
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
    
    % �趨�ο���Ϣ
%     % ֱ�߹켣
%     V_ref = 20;
%     X_ref = V_ref * time;
%     Y_ref = 50;
%     phi_ref = 0;
%     delta_f_ref = 0;
%     accel_ref = 0;

    % �뾶Ϊ100m��Բ�ι켣, Բ��Ϊ(0, 100), �ٶ�Ϊ20m/s
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


    % ���ݳ�ʼ�����õ�����״̬��kesi[6��1]
    [kesi] = DataInitialization(Nx, Nu, X, Y, phi, V, delta_f, accel, X_ref, Y_ref, phi_ref, V_ref, delta_f_ref, accel_ref, U);

    % ����Ԥ�ⷽ�̵Ĳ�������PHI[Np��Nx,Nx+Nu]��THETA[Np��Nx,Nc��Nu]
    [A,B,C,PHI,THETA] = MatrixConstruction(Nx, Nu, Np, Nc, V_ref, phi_ref, delta_f_ref, L, T);

    % ����Լ�������Ĳ�������H[Nc��Nu+1,Nc��Nu+1],G[Nc��Nu+1,1],A_cons[2��Nc��Nu,Nc��Nu+1],b_cons[2��Nc��Nu,1],lb[Nc��Nu+1,1],ub[Nc��Nu+1,1]
    [H,G,A_cons,b_cons,lb,ub]=ConstraintConstruction(Nx, Nu, Np, Nc, PHI, THETA, Q, R, Row, kesi, U);

    % ������
%     options = optimset('Algorithm','interior-point-convex');
%     options = optimset('Algorithm','active-set'); 
    warning off all   
%     [AA,fval,exitflag] = quadprog(H,G,A_cons,b_cons,[],[],lb,ub,[],options);
    [AA,fval,exitflag] = quadprog(H, G, A_cons, b_cons, [], [], lb, ub, []);

    % �������
    u_piao(1) = AA(1); % delta(delta_f - delta_f_ref)
    u_piao(2) = AA(2); % delta(accel - accel_ref)
    U(1) = kesi(5) + u_piao(1); % delta_f - delta_f_ref
    U(2) = kesi(6) + u_piao(2); % accel - accel_ref
    delta_f = U(1) + delta_f_ref; % k+1ʱ�̵�delta_f
    accel = U(2) + accel_ref; % k+1ʱ�̵�accel
end

%% ��ͼ
figure
plot(X_final,Y_final)
figure
plot(V_final)
figure
plot(a_final)


%% ���ݳ�ʼ���Ӻ���
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


%% ����Ԥ�ⷽ�̵Ĳ��������Ӻ���
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


%% ����Լ�������Ĳ��������Ӻ���
function [H,G,A_cons,b_cons,lb,ub]=ConstraintConstruction(Nx, Nu, Np, Nc, PHI, THETA, Q, R, Row, kesi, U)
    % �Ż�Ŀ�꣺���ٶȱ仯�� => 0��ǰ��ת�Ǳ仯�� => 0��������� => 0
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
    % ����ʽԼ������
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
    Ut = kron(ones(Nc,1),U); % ��һʱ�̵Ŀ�����
    umin = [-0.889;-2]; % u = [delta_f - delta_f_ref;accel - accel_ref]
    umax = [0.681;2];
    delta_umin = [-0.01;-0.02];
    delta_umax = [0.01;0.02];
    Umin = kron(ones(Nc,1),umin);
    Umax = kron(ones(Nc,1),umax);
    % �����ɳ����Ӻ��Լ��ϵ��
    A_cons_cell = {A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1)};
    b_cons_cell = {Umax-Ut;Ut-Umin};
    A_cons = cell2mat(A_cons_cell);
    b_cons = cell2mat(b_cons_cell);
    % ״̬��Լ��
    delta_Umin = kron(ones(Nc,1),delta_umin);
    delta_Umax = kron(ones(Nc,1),delta_umax);
    lb = [delta_Umin;0];
    ub = [delta_Umax;10];
end


