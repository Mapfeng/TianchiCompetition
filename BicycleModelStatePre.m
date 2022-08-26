function [x_pre, y_pre, phi_pre, v_pre, beta_pre, y_acc_pre, x_delta_acc_pre, y_delta_acc_pre] = BicycleModelStatePre(vehicle_info, Np, Ts)

for 1: 1 : Np 
    
    [x_pre, y_pre, phi_pre, v_pre, beta_pre, y_acc_pre, x_delta_acc_pre, y_delta_acc_pre] = GetNextStateByBicycleModel()
    
end