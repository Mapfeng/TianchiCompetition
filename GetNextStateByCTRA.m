
%{
1.�����д���ڣ�2022.08.24
2.����˵�������ڸ���״̬�ĳ����˶�ѧģ�ͣ�����CTRA(constant turn rate and acceleration)��SCTRA(simplified constant turn rate and acceleration)
3.�������룺X_now[6��1]������������ǰλ��[x_now, y_now]���ٶ�v_now�����ٶ�a_now�������theta_now��������ٶ�omega_now����������T
4.���������X_next[6��1]��������һʱ��λ��[x_next, y_next]���ٶ�v_next�����ٶ�a_next�������theta_next��������ٶ�omega_next
%}

% ״̬��[x,y,v,a,theta,omega]'

function [X_next] = GetNextStateByCTRA(x_now, y_now, v_now, a_now, theta_now, omega_now, T)

    X_now = [x_now, y_now, v_now, a_now, theta_now, omega_now]'; % ������ǰ״̬

    if (abs(omega_now) < 0.01)
        % �ü򻯹�ʽ
        delta_x = (v_now * T + 0.5 * a_now * T^2) * cos(theta_now);
        delta_y = (v_now * T + 0.5 * a_now * T^2) * sin(theta_now);
    else
        % �ø��ӹ�ʽ
        delta_x = ((v_now + a_now * T) * sin(theta_now + omega_now * T) - v_now * sin(theta_now)) / omega_now + a_now * cos(theta_now + omega_now * T - cos(theta_now)) / omega_now^2;
        delta_y = - ((v_now + a_now * T) * cos(theta_now + omega_now * T) - v_now * cos(theta_now)) / omega_now + a_now * cos(theta_now + omega_now * T - sin(theta_now)) / omega_now^2;
    end

    X_next = X_now + [delta_x; delta_y; a_now * T; 0; omega_now * T; 0]; % ������һʱ��״̬



























