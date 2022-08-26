一个文件定义基础的数据，可以根据实际需求添加新的metadata

```C++

struct Point {
  double x;
  double y;
    ...
}

struct Vel {
  double vel_x;
  double vel_y;
  double rot_z;
    ...
}

struct Box {
  // box 四个顶点，朝向，中心点坐标。
  vector<Point> (4);
  double heading;
  Point center_point;
    ...
}
struct LaneBoundType {
  emun BoundType {
    Crossable;
    UnCrossabe;
  }
  BoundType bound_type;
    ...
}

// 8月22号
struct LaneInfo {
  string lane_id;
  vector<Point> center_line;
  emun LaneChangeType {
    CHANGE_TO_LEFT;
    CHANGE_TO_RIGHT;
    GO_STRAIGHT;
  }
  LaneChangeType lane_change_type;
  vector<BoundType> left_bound_type;
  vector<BoundType> right_bound_type;
    ...
}

// 全局路径，目前打算用多段lane构成
struct Route {
  // 第一个元素应当为ego当前所在lane， 最后一个元素未包含终点的lane
  //通过pair第一个元素判断是否会发生变道
  Vector<pair<string/*lane id*/, LaneSegment>> route;
  //如果是匝道，merge point表示 匝道的中心线的最后一个元素
  //如果是非匝道，但是有匝道merge进route， 则表示匝道中心线最后一个元素在route上的映射
  Point merge_point;
    ...
}

struct VehicleState {
  Point coord;
  double heading;
  Vel vel;
  double acceleraction;
    ...
}
// 可以根据控制器以及规划器的需求调整，下面是一个参考
struct VehicleInfo {
  VehicleState* state;
  LaneInfo* lane_info;
  double lateral_offset = 0.0;
  vector<double> speed_limit(2,1);
  vector<double> rot_speed_limit(2,1);
  vector<double> acceleraction_limit(2,1);
    ...
  //定义可以由route中的merge_point推出
  double dist_to_merge = inf;
    
}

struct OptimalPath {
  vector<VehicleState> vehicle_state;
  double cost;
    ...
}

struct Cfg {
  double lane_change_time = 4.0s
}



```

Preprocessor文件 预处理数据用于其他模块

```C++
class Preprocessor {
  preprocessor(/*赛方给的可以获取的所有数据*/) {}
   
  void GetVehicle_info() {
    ...
  }
   ...
      
  // 需要什么数据就添加什么数据，并设计相应的Get函数   
  VehicleInfo vehicle_info_;
}
```

controller文件

```C++
class Controller {
  Controller(const OptimalPath& optimizal_path, const VechicleInfo& vehicle_info) {}
  
  vector<double> GetControlSignal() {
      ...
      // 沛锋主要实现这个函数，利用optimaizal_path_,vehicle_info_,设计一个轨迹跟踪控制器注意各种约束，优化目标暂定：加速度变化量->0,前轮转角的变化趋于0，以及跟踪误差趋于0。如果上述两个变量中的数据不够可以自己添加，只要能从赛方提供的信息中推到出来即可。使用矩阵的地方注意维度。
     // 可以先在matlab按照上述方法定义基本的数据结构，然后根据这些基本的数据结构设计仿真环境。
     //需要的信息：1、参考轨迹点坐标[X_ref,Y_ref]，以及每个参考点的速度V_ref，横摆角phi_ref，前轮转角delta_f_ref；2、ego车：后轴中心位置[X,Y]，车辆朝向即横摆角phi，后轴中心纵向速度V，前轮转角delta_f，轴距L
  }
    
  OptimaizalPath* optimaizal_path_;
  VehicleInfo* vehicle_info_;
}
```



router文件，不重要，可以最后在考略

```C++
class Router {
}
```

LocalPlanner

```c++
class LocalPlanner {

  
    
  Vehicle_Info vehicle_info_;
  
}
```



```
[x_next, y_next, v_next, a_next, theta_next, omega_next] = GetNextStateByCTRA(x_now, y_now, v_now, a_now, theta_now, omega_now, T) {
  if (abs(omega_now) < 0.01) {
  // 用下面的简化公式
  return 
  }
  // 用上面复杂公式
}
```



