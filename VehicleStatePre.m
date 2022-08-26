
%{
1.程序编写日期：2022.08.24
2.程序说明：
3.程序输入：车辆当前时刻的六个状态，预测时域Np和采样周期T
4.程序输出：车辆在预测时域Np内每一时刻的状态
%}

% 函数的六个输出都是[Np,1]的矩阵

function [x_pre, y_pre, v_pre, a_pre, theta_pre, omega_pre] = VehicleStatePre(x_now, y_now, v_now, a_now, theta_now, omega_now, Np,T)
    % 创建空矩阵，保存输出
    x_pre = zeros(Np,1);
    y_pre = zeros(Np,1);
    v_pre = zeros(Np,1);
    a_pre = zeros(Np,1);
    theta_pre = zeros(Np,1);
    omega_pre = zeros(Np,1);
    % 变量初始化
    x = x_now;
    y = y_now;
    v = v_now;
    a = a_now;
    theta = theta_now;
    omega = omega_now;
    for i = 1 : Np
        % 通过模型预测下一时刻状态
        [X_next] = GetNextStateByCTRA(x, y, v, a, theta, omega, T);
        % 更新变量
        x = X_next(1);
        y = X_next(2);
        v = X_next(3);
        a = X_next(4);
        theta = X_next(5);
        omega = X_next(6);
        % 保存i时刻的预测状态，作为函数输出
        x_pre(i,1) = x;
        y_pre(i,1) = y;
        v_pre(i,1) = v;
        a_pre(i,1) = a;
        theta_pre(i,1) = theta;
        omega_pre(i,1) = omega;
    end


























