
%{
1.�����д���ڣ�2022.08.26
2.����˵�������øĽ������г�ģ��Ԥ�⳵��Ԥ��ʱ���ڵ�״̬��
3.�������룺������״̬��vehicle_info��Ԥ��ʱ��Np��ǰ��ת������delta[Np,1]��������ٶ�����acc[Np,1]����������Ts
4.���������Ԥ��ʱ���ڵ�λ�ã�����ǣ������ٶȣ����Ĳ�ƫ�ǣ�������ٶȣ�����Ӽ��ٶȣ�����Ӽ��ٶȣ���Ϊ[Np,1]ά����
%}

function [x_pre, y_pre, theta_pre, v_pre, beta_pre, y_acc_pre, x_jerk_pre, y_jerk_pre] = BicycleModelStatePre(vehicle_info, Np, delta, acc, Ts)

    x_now = vehicle_info(1); % ��ǰ����λ��
    y_now = vehicle_info(2); % ��ǰ����λ��
    theta_now = vehicle_info(3); % ��ǰ�����
    v_now = vehicle_info(4); % ��ǰ����
    x_a_now = vehicle_info(5); % ��ǰ������ٶ�
    y_acc_now = vehicle_info(6); % ��ǰ������ٶ�
    delta_f_now = vehicle_info(7); % ��ǰǰ��ת��
    delta_old = vehicle_info(8); % ��һʱ��ǰ��ת������
    acc_old = vehicle_info(9); % ��һʱ��������ٶ�����
    L = vehicle_info(12); % �������

    % ��������ʼ��
    x_pre = zeros(Np,1); % ����λ��
    y_pre = zeros(Np,1); % ����λ��
    theta_pre = zeros(Np,1); % �����
    v_pre = zeros(Np,1); % �����ٶ�
    beta_pre = zeros(Np,1); % ���Ĳ�ƫ��
    y_acc_pre = zeros(Np,1); % ������ٶ�
    x_jerk_pre = zeros(Np,1); % ����Ӽ��ٶ�
    y_jerk_pre = zeros(Np,1); % ����Ӽ��ٶ�

    for i = 1 : Np 
        % �������г�ģ�ͼ�����һʱ��״̬
        [X_next] = GetNextStateByBicycleModel(x_now, y_now, theta_now, v_now, delta(i), acc(i), Ts, L);
        % �������
        x_pre(i,1) = X_next(1); 
        y_pre(i,1) = X_next(2);
        theta_pre(i,1) = X_next(3);
        v_pre(i,1) = X_next(4); 
        beta_pre(i,1) = X_next(5);
        y_acc_pre(i,1) = v_pre(i,1) * sin(beta_pre(i,1)) * 2 / L * v_pre(i,1);
        x_jerk_pre(i,1) = acc(i) - acc_old; % ��ǰ������ٶ�x_a_now�Ƿ�����һʱ��������ٶ�����acc_old��ȣ������õ�ʽ�����������Ϊacc(i) - x_a_now
        y_jerk_pre(i,1) = y_acc_pre(i,1) - y_acc_now;
        % ���±���
        x_now = X_next(1); 
        y_now = X_next(2);
        theta_now = X_next(3);
        v_now  = X_next(4);
        acc_old = acc(i);
        y_acc_now = y_acc_pre(i,1);
    end