
%{
1.程序编写日期：2022.08.24
2.程序说明：用于更新状态的车辆运动学模型，包括CTRA(constant turn rate and acceleration)和SCTRA(simplified constant turn rate and acceleration)
3.程序输入：X_now[6×1]，包括车辆当前位置[x_now, y_now]，速度v_now，加速度a_now，航向角theta_now，航向角速度omega_now；采样周期T
4.程序输出：X_next[6×1]，包括下一时刻位置[x_next, y_next]，速度v_next，加速度a_next，航向角theta_next，航向角速度omega_next
%}

% 状态量[x,y,v,a,theta,omega]'

function [X_next] = GetNextStateByCTRA(x_now, y_now, v_now, a_now, theta_now, omega_now, T)

    X_now = [x_now, y_now, v_now, a_now, theta_now, omega_now]'; % 车辆当前状态

    if (abs(omega_now) < 0.01)
        % 用简化公式
        delta_x = (v_now * T + 0.5 * a_now * T^2) * cos(theta_now);
        delta_y = (v_now * T + 0.5 * a_now * T^2) * sin(theta_now);
    else
        % 用复杂公式
        delta_x = ((v_now + a_now * T) * sin(theta_now + omega_now * T) - v_now * sin(theta_now)) / omega_now + a_now * cos(theta_now + omega_now * T - cos(theta_now)) / omega_now^2;
        delta_y = - ((v_now + a_now * T) * cos(theta_now + omega_now * T) - v_now * cos(theta_now)) / omega_now + a_now * cos(theta_now + omega_now * T - sin(theta_now)) / omega_now^2;
    end

    X_next = X_now + [delta_x; delta_y; a_now * T; 0; omega_now * T; 0]; % 车辆下一时刻状态



























