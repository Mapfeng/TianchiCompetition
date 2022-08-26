function cost  = CostFunction(x, vehicle_info, lane_info, Np, Ts)
x_acc =x(1:Np); % 加速度 默认
delta = x(Np + 1 : 2 * Np); % 前轮转角

% x 是自变量， 由 Np维acc 与Np维delta组成

% 下面的所有变量都是 Np * 1 维向量 y_acc 表示加速， x_delta_acc_pre 表示 jerk， 可以用 acc(i) -
% acc(i - 1) 得到， 比如 x_jerk_pre(1) = x_acc（1） - acc_(0)（上一时刻的加速度），后面递推即可，
% 其中y_acc的计算需要思考， y_jerk_pre 同x_jerk_pre;

%TODO：在BicycleModelStatePre下实现下面函数：
[x_pre, y_pre, phi_pre, v_pre, beta_pre, y_acc_pre, x_jerk_pre, y_jerk_pre] = BicycleModelStatePre(vehicle_info, Np,acc, delta, Ts);

