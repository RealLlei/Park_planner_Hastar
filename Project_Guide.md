# HAStar_ParkingPlanner 项目指导文档

## 项目概述
HAStar_ParkingPlanner 是一个基于混合 A* 算法的路径规划项目，主要用于停车场景的路径规划。项目结合了离散搜索和连续曲线生成，能够在复杂环境中生成平滑且可行的路径。

### 项目结构
以下是项目的主要目录和文件：
- **src/**: 包含主要的源代码文件。
  - `hybrid_a_star.cc` 和 `hybrid_a_star.h`: 实现混合 A* 算法的核心逻辑。
  - `grid_search.cc` 和 `grid_search.h`: 实现网格搜索的启发式计算。
  - `math_utils.cc` 和 `math_utils.h`: 提供数学工具函数。
  - `node3d.cc` 和 `node3d.h`: 定义搜索节点的属性和操作。
  - `reeds_shepp_path.cc` 和 `reeds_shepp_path.h`: 实现 Reed-Shepp 曲线生成。
- **config/**: 包含配置文件，例如车辆参数和停车场景。
- **result_plot/**: 包含可视化相关的代码。
- **main.cc**: 程序的入口点。

---

## 核心功能实现

### 1. 混合 A* 算法
混合 A* 算法结合了离散搜索和连续曲线生成，能够在复杂环境中生成平滑路径。其主要实现位于 `hybrid_a_star.cc` 中。

#### **主要流程**
1. **初始化**：
   - 加载起点和终点节点。
   - 初始化启发式地图（`grid_a_star_heuristic_generator_`）。
   - 清空 `open_set_` 和 `close_set_`。

2. **搜索循环**：
   - 从优先队列 `open_pq_` 中取出代价最低的节点。
   - 检查是否可以通过解析曲线（`AnalyticExpansion`）直接连接到终点。
   - 如果不能，生成当前节点的所有可能的下一步节点（`Next_node_generator`）。
   - 对每个生成的节点：
     - 检查是否越界或与障碍物碰撞（`ValidityCheck`）。
     - 如果有效，计算代价（`CalculateNodeCost`）并加入 `open_set_` 和 `open_pq_`。

3. **结果处理**：
   - 如果找到终点，调用 `GetResult` 函数生成最终路径。
   - 如果搜索失败，返回错误信息。

#### **代码示例**
以下是搜索循环的核心代码：
```cpp
while (!open_pq_.empty()) {
  const std::string current_id = open_pq_.top().first;
  open_pq_.pop();
  std::shared_ptr<Node3d> current_node = open_set_[current_id];

  if (AnalyticExpansion(current_node)) {
    break;
  }

  close_set_.emplace(current_node->GetIndex(), current_node);

  for (size_t i = 0; i < next_node_num_; ++i) {
    std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
    if (next_node == nullptr || close_set_.find(next_node->GetIndex()) != close_set_.end() || !ValidityCheck(next_node)) {
      continue;
    }

    if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
      CalculateNodeCost(current_node, next_node);
      open_set_.emplace(next_node->GetIndex(), next_node);
      open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
    }
  }
}
```

---

### 2. 代价估计
代价估计是混合 A* 算法的核心部分，主要由 `CalculateNodeCost` 函数实现。

#### **代价的组成**
1. **轨迹代价（Trajectory Cost）**：
   - 评估从当前节点到下一节点的移动代价。
   - 包括前进/后退代价、换挡代价、转向代价和转向变化代价。

2. **启发式代价（Heuristic Cost）**：
   - 评估从当前节点到目标节点的估计代价。
   - 使用网格搜索（`GridSearch`）生成的启发式地图。

#### **代码示例**
以下是代价计算的核心代码：
```cpp
void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}
```

---

### 3. 曲线生成与轨迹平滑
曲线生成是混合 A* 算法的重要组成部分，用于生成平滑的路径。项目中使用了 Reed-Shepp 曲线和其他数学工具。

#### **Reed-Shepp 曲线**
- Reed-Shepp 曲线是一种考虑方向变化的最短路径曲线，适用于非完整性约束的车辆。
- 实现在 `reeds_shepp_path.cc` 中。

#### **轨迹平滑**
- 使用 `math_utils` 中的工具函数对路径进行平滑处理。
- 例如，`math::NormalizeAngle` 用于归一化角度，`math::GetTrajFromCurvePathsConnect` 用于生成平滑轨迹。

#### **代码示例**
以下是生成下一步节点的代码：
```cpp
std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    std::shared_ptr<Node3d> current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;

  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering = -max_steer_angle_ + ...;
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering = -max_steer_angle_ + ...;
    traveled_distance = -step_size_;
  }

  std::vector<double> intermediate_x, intermediate_y, intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = math::NormalizeAngle(last_phi + ...);
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }

  if (intermediate_x.back() > XYbounds_[1] || ...) {
    return nullptr;
  }

  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 warm_start_config_.grid_a_star_xy_resolution));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}
```

---

### 4. 停车场景读取

#### 函数：`ReadParkingScenario`

**功能**：
- 读取停车场景配置文件，解析起点、终点、停车场边界和障碍物信息。
- 将解析结果存储到`ParkingScenario`对象中。

**输入**：
- `parking_scenario`：一个`ParkingScenario`对象的引用，用于存储解析结果。

**输出**：
- 返回`true`表示读取成功，`false`表示读取失败。

**逻辑**：
1. **打开配置文件**：
   - 文件路径为`config/parking_scenario.txt`。
   - 如果文件无法打开，打印错误信息并返回`false`。

2. **逐行读取文件内容**：
   - 每行数据通过`:`分隔，提取出键值对。
   - 根据行号调用不同的解析函数：
     - **第0行**：调用`GetParkingScenarioPose`解析起点位置。
     - **第1行**：调用`GetParkingScenarioPose`解析终点位置。
     - **第2行**：调用`GetParkingScenarioBoundary`解析停车场边界。
     - **第3行**：调用`GetParkingScenarioObstacle`解析障碍物信息。

3. **打印解析结果**：
   - 打印停车场边界和障碍物的详细信息。

**依赖函数**：
- `GetParkingScenarioPose`：解析起点和终点的位置信息。
- `GetParkingScenarioBoundary`：解析停车场边界，返回`std::vector<double>`。
- `GetParkingScenarioObstacle`：解析障碍物信息，返回`std::vector<std::vector<Vec2d>>`。

---

### 5. 混合 A* 参数读取

#### 函数：`ReadHybridAStarParam`

**功能**：
- 读取混合 A* 算法的参数配置文件，并将参数存储到`WarmStartConfig`对象中。

**输入**：
- `warm_start_config`：一个`WarmStartConfig`对象的引用，用于存储解析结果。

**输出**：
- 返回`true`表示读取成功，`false`表示读取失败。

**逻辑**：
1. **打开配置文件**：
   - 文件路径为`config/hya_param.conf`。
   - 如果文件无法打开，打印错误信息并返回`false`。

2. **逐行读取文件内容**：
   - 每行数据通过`:`分隔，提取出键值对。
   - 根据行号将值赋给`WarmStartConfig`对象的不同成员变量：
     - **第0行**：`xy_grid_resolution`。
     - **第1行**：`phi_grid_resolution`。
     - **第2行**：`next_node_num`。
     - **第3行**：`step_size`。
     - **第4行**：`traj_forward_penalty`。
     - **第5行**：`traj_back_penalty`。
     - **第6行**：`traj_gear_switch_penalty`。
     - **第7行**：`traj_steer_penalty`。
     - **第8行**：`traj_steer_change_penalty`。
     - **第9行**：`grid_a_star_xy_resolution`。
     - **第10行**：`node_radius`。
     - **第11行**：`delta_t`。

3. **关闭文件**：
   - 读取完成后关闭文件流。

---

### 6. 混合 A* 路径规划

#### 函数：`Plan`

**功能**：
- 实现混合 A* 算法的路径规划。
- 从起点到终点生成一条可行的路径，同时避开障碍物。

**输入**：
- `start_pose`：起点的位置信息（`Pos3d`）。
- `end_pose`：终点的位置信息（`Pos3d`）。
- `XYbounds`：停车场的边界（`std::vector<double>`）。
- `obstacles_vertices_vec`：障碍物的顶点信息（`std::vector<std::vector<Vec2d>>`）。
- `result`：存储路径规划结果的对象（`HybridAStartResult*`）。

**输出**：
- 返回`true`表示路径规划成功，`false`表示失败。

**逻辑**：
1. **清空容器**：
   - 清空`open_set_`、`close_set_`和优先队列`open_pq_`。
   - 重置`final_node_`为`nullptr`。

2. **处理障碍物**：
   - 将障碍物的顶点转换为线段表示，存储到`obstacles_linesegments_vec_`中。

3. **加载边界和节点**：
   - 将`XYbounds`存储到`XYbounds_`。
   - 初始化起点和终点节点（`start_node_`和`end_node_`）。

4. **检查节点有效性**：
   - 调用`ValidityCheck`函数，确保起点和终点不与障碍物发生碰撞。

5. **生成代价地图**：
   - 使用`grid_a_star_heuristic_generator_`生成从终点到其他位置的代价地图。

6. **初始化开放集和优先队列**：
   - 将起点加入`open_set_`和`open_pq_`。

7. **主循环**：
   - 从优先队列中取出代价最低的节点。
   - 检查是否可以通过解析曲线直接连接到终点（调用`AnalyticExpansion`）。
   - 如果不能，生成当前节点的所有可能的下一步节点，并检查其有效性。
   - 将有效节点加入`open_set_`和`open_pq_`。

---

### 7. 可视化功能

#### 函数：`PlotObstacles`

**功能**：
- 可视化障碍物，通过绘制其顶点连接的线段来表示。

**输入**：
- `obstacles_vertices_vec`：一个包含障碍物顶点的二维向量（`std::vector<std::vector<Vec2d>>`）。

**输出**：
- 在图中绘制蓝色的障碍物边界。

**逻辑**：
1. 遍历每个障碍物的顶点集合。
2. 将顶点连接成线段，并闭合形成多边形。
3. 使用 `matplotlibcpp` 绘制蓝色线段。

---

#### 函数：`PlotVehicleTraj`

**功能**：
- 动态可视化车辆的轨迹。

**输入**：
- `vehicle_param`：车辆参数（`VehicleParam`）。
- `traj_x`：轨迹的 x 坐标集合（`std::vector<double>`）。
- `traj_y`：轨迹的 y 坐标集合（`std::vector<double>`）。
- `traj_phi`：轨迹的方向角集合（`std::vector<double>`）。

**输出**：
- 动态绘制红色的轨迹线，并在每个时间点显示车辆的当前位置。

**逻辑**：
1. 清除当前图像。
2. 调用 `PlotTrajectory` 绘制轨迹线。
3. 调用 `PlotVehicle` 绘制车辆当前位置。
4. 暂停一段时间以实现动态效果。

---

#### 函数：`PlotParkingBoundary`

**功能**：
- 可视化停车场边界。

**输入**：
- `parking_boundary`：一个包含停车场边界点的二维向量（`std::vector<std::vector<Vec2d>>`）。

**输出**：
- 在图中绘制绿色的停车场边界。

**逻辑**：
1. 遍历每个边界点集合。
2. 将点连接成线段，并闭合形成多边形。
3. 使用 `matplotlibcpp` 绘制绿色线段。
