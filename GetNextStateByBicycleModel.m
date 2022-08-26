
%{
1.�����д���ڣ�2022.08.25
2.����˵����ʹ�������ṩ�ĸĽ����г�ģ�͸��³���״̬
3.�������룺X_now[5��1]������������ǰλ��[x_now, y_now]�������theta_now�������ٶ�v_now����������������������Ĳ�ƫ��beta��ǰ��ת��delta_f_now��������ٶ�a_now����������Ts���������L
4.���������X_next[5��1]��������һʱ��λ��[x_next, y_next]�������theta_next�������ٶ�v_next�����Ĳ�ƫ��beta
%}

% ״̬��[x,y,theta,v]'

function [X_next] = GetNextStateByBicycleModel(x_now, y_now, theta_now, v_now, delta_f_now, a_now, Ts, L)

    % ��ǰʱ�̵�״̬
    X_now = [x_now; y_now; theta_now; v_now; 0];

    % �������Ĳ�ƫ��
    beta = atan(0.5 * tan(delta_f_now));
    % ��������λ������
    delta_x = v_now * cos(theta_now + beta) * Ts;
    % �������λ������
    delta_y = v_now * sin(theta_now + beta) * Ts;
    % ���㺽�������
    delta_theta = v_now * sin(beta) / (L / 2) * Ts;
    % �����ٶ�����
    delta_v = a_now * Ts;

    % ��һʱ�̵�״̬
    X_next = X_now + [delta_x; delta_y; delta_theta; delta_v; beta]; % ������һʱ��״̬



























