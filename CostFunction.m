function cost  = CostFunction(x, vehicle_info, lane_info, Np, Ts)
x_acc =x(1:Np); % ���ٶ� Ĭ��
delta = x(Np + 1 : 2 * Np); % ǰ��ת��

% x ���Ա����� �� Npάacc ��Npάdelta���

% ��������б������� Np * 1 ά���� y_acc ��ʾ���٣� x_delta_acc_pre ��ʾ jerk�� ������ acc(i) -
% acc(i - 1) �õ��� ���� x_jerk_pre(1) = x_acc��1�� - acc_(0)����һʱ�̵ļ��ٶȣ���������Ƽ��ɣ�
% ����y_acc�ļ�����Ҫ˼���� y_jerk_pre ͬx_jerk_pre;

%TODO����BicycleModelStatePre��ʵ�����溯����
[x_pre, y_pre, phi_pre, v_pre, beta_pre, y_acc_pre, x_jerk_pre, y_jerk_pre] = BicycleModelStatePre(vehicle_info, Np,acc, delta, Ts);

