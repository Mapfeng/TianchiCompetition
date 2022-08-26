
%{
1.�����д���ڣ�2022.08.24
2.����˵����
3.�������룺������ǰʱ�̵�����״̬��Ԥ��ʱ��Np�Ͳ�������T
4.���������������Ԥ��ʱ��Np��ÿһʱ�̵�״̬
%}

% �����������������[Np,1]�ľ���

function [x_pre, y_pre, v_pre, a_pre, theta_pre, omega_pre] = VehicleStatePre(x_now, y_now, v_now, a_now, theta_now, omega_now, Np,T)
    % �����վ��󣬱������
    x_pre = zeros(Np,1);
    y_pre = zeros(Np,1);
    v_pre = zeros(Np,1);
    a_pre = zeros(Np,1);
    theta_pre = zeros(Np,1);
    omega_pre = zeros(Np,1);
    % ������ʼ��
    x = x_now;
    y = y_now;
    v = v_now;
    a = a_now;
    theta = theta_now;
    omega = omega_now;
    for i = 1 : Np
        % ͨ��ģ��Ԥ����һʱ��״̬
        [X_next] = GetNextStateByCTRA(x, y, v, a, theta, omega, T);
        % ���±���
        x = X_next(1);
        y = X_next(2);
        v = X_next(3);
        a = X_next(4);
        theta = X_next(5);
        omega = X_next(6);
        % ����iʱ�̵�Ԥ��״̬����Ϊ�������
        x_pre(i,1) = x;
        y_pre(i,1) = y;
        v_pre(i,1) = v;
        a_pre(i,1) = a;
        theta_pre(i,1) = theta;
        omega_pre(i,1) = omega;
    end


























