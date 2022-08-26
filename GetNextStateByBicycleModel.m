
%{
1.程序编写日期：2022.08.25
2.程序说明：使用赛方提供的改进自行车模型更新车辆状态
3.程序输入：X_now[5×1]，包括车辆当前位置[x_now, y_now]，航向角theta_now，纵向速度v_now，第五行留空用来输出质心侧偏角beta；前轮转角delta_f_now；纵向加速度a_now；采样周期Ts；车轴轴距L
4.程序输出：X_next[5×1]，包括下一时刻位置[x_next, y_next]，航向角theta_next，纵向速度v_next；质心侧偏角beta
%}

% 状态量[x,y,theta,v]'

function [X_next] = GetNextStateByBicycleModel(x_now, y_now, theta_now, v_now, delta_f_now, a_now, Ts, L)

    % 当前时刻的状态
    X_now = [x_now; y_now; theta_now; v_now; 0];

    % 计算质心侧偏角
    beta = atan(0.5 * tan(delta_f_now));
    % 计算纵向位移增量
    delta_x = v_now * cos(theta_now + beta) * Ts;
    % 计算横向位移增量
    delta_y = v_now * sin(theta_now + beta) * Ts;
    % 计算航向角增量
    delta_theta = v_now * sin(beta) / (L / 2) * Ts;
    % 计算速度增量
    delta_v = a_now * Ts;

    % 下一时刻的状态
    X_next = X_now + [delta_x; delta_y; delta_theta; delta_v; beta]; % 车辆下一时刻状态



























