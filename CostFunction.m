function cost = CostFunction(x, vehicle_info, lane_info, Np, Ts)
delta = x(1 : Np); % 前轮转角 
acc = x(Np + 1 : 2 * Np); % 加速度 默认

% x是自变量，由Np维delta与Np维v组成

%{
    下面的所有变量都是[Np,1]维向量，y_acc表示横向加速度，x_jerk表示纵向jerk，可以用acc(i) - acc(i-1)得到，
    比如x_jerk_pre(1) = x_acc_pre(1) - x_acc_pre(0)（上一时刻的加速度），后面递推即可，其中y_acc的计算需要思考，
    y_jerk_pre 同 x_jerk_pre;
%}

%TODO：在BicycleModelStatePre下实现下面函数：
[x_pre, y_pre, theta_pre, v_pre, beta_pre, y_acc_pre, x_jerk_pre, y_jerk_pre] = BicycleModelStatePre(vehicle_info, Np, delta, acc, Ts);

