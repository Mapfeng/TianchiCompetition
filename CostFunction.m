function cost = CostFunction(x, vehicle_info, lane_info, Np, Ts)
delta = x(1 : Np); % ǰ��ת�� 
acc = x(Np + 1 : 2 * Np); % ���ٶ� Ĭ��

% x���Ա�������Npάdelta��Npάv���

%{
    ��������б�������[Np,1]ά������y_acc��ʾ������ٶȣ�x_jerk��ʾ����jerk��������acc(i) - acc(i-1)�õ���
    ����x_jerk_pre(1) = x_acc_pre(1) - x_acc_pre(0)����һʱ�̵ļ��ٶȣ���������Ƽ��ɣ�����y_acc�ļ�����Ҫ˼����
    y_jerk_pre ͬ x_jerk_pre;
%}

%TODO����BicycleModelStatePre��ʵ�����溯����
[x_pre, y_pre, theta_pre, v_pre, beta_pre, y_acc_pre, x_jerk_pre, y_jerk_pre] = BicycleModelStatePre(vehicle_info, Np, delta, acc, Ts);

