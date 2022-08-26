
%{
1.程序编写日期：2022.08.26
2.程序说明：利用改进的自行车模型预测车辆预测时域内的状态量
3.程序输入：车辆各状态量vehicle_info；预测时域Np；前轮转角序列delta[Np,1]；纵向加速度序列acc[Np,1]；采样周期Ts
4.程序输出：预测时域内的位置，航向角，纵向速度，质心侧偏角，横向加速度，纵向加加速度，横向加加速度，均为[Np,1]维向量
%}

function [x_pre, y_pre, theta_pre, v_pre, beta_pre, y_acc_pre, x_jerk_pre, y_jerk_pre] = BicycleModelStatePre(vehicle_info, Np, delta, acc, Ts)

    x_now = vehicle_info(1); % 当前纵向位置
    y_now = vehicle_info(2); % 当前横向位置
    theta_now = vehicle_info(3); % 当前航向角
    v_now = vehicle_info(4); % 当前车速
    x_a_now = vehicle_info(5); % 当前纵向加速度
    y_acc_now = vehicle_info(6); % 当前横向加速度
    delta_f_now = vehicle_info(7); % 当前前轮转角
    delta_old = vehicle_info(8); % 上一时刻前轮转角命令
    acc_old = vehicle_info(9); % 上一时刻纵向加速度命令
    L = vehicle_info(12); % 车轴轴距

    % 输出矩阵初始化
    x_pre = zeros(Np,1); % 纵向位置
    y_pre = zeros(Np,1); % 横向位置
    theta_pre = zeros(Np,1); % 航向角
    v_pre = zeros(Np,1); % 纵向速度
    beta_pre = zeros(Np,1); % 质心侧偏角
    y_acc_pre = zeros(Np,1); % 横向加速度
    x_jerk_pre = zeros(Np,1); % 纵向加加速度
    y_jerk_pre = zeros(Np,1); % 横向加加速度

    for i = 1 : Np 
        % 根据自行车模型计算下一时刻状态
        [X_next] = GetNextStateByBicycleModel(x_now, y_now, theta_now, v_now, delta(i), acc(i), Ts, L);
        % 保存输出
        x_pre(i,1) = X_next(1); 
        y_pre(i,1) = X_next(2);
        theta_pre(i,1) = X_next(3);
        v_pre(i,1) = X_next(4); 
        beta_pre(i,1) = X_next(5);
        y_acc_pre(i,1) = v_pre(i,1) * sin(beta_pre(i,1)) * 2 / L * v_pre(i,1);
        x_jerk_pre(i,1) = acc(i) - acc_old; % 当前纵向加速度x_a_now是否与上一时刻纵向加速度命令acc_old相等？相等则该等式成立，否则改为acc(i) - x_a_now
        y_jerk_pre(i,1) = y_acc_pre(i,1) - y_acc_now;
        % 更新变量
        x_now = X_next(1); 
        y_now = X_next(2);
        theta_now = X_next(3);
        v_now  = X_next(4);
        acc_old = acc(i);
        y_acc_now = y_acc_pre(i,1);
    end