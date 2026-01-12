// 防止头文件重复包含
#ifndef HYBRID_A_STAR
#define HYBRID_A_STAR

#include <algorithm>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grid_search.h"
#include "reeds_shepp_path.h"

// Hybrid A*算法的结果数据结构
struct HybridAStartResult {
  std::vector<double> x;             // x坐标序列
  std::vector<double> y;             // y坐标序列
  std::vector<double> phi;           // 航向角序列
  std::vector<double> v;             // 速度序列
  std::vector<double> a;             // 加速度序列
  std::vector<double> steer;         // 前轮转角序列
  std::vector<double> accumulated_s; // 累计路径长度序列
};

class HybridAStar {
 public:
  // 构造函数：接收热启动配置和车辆参数
  explicit HybridAStar(const WarmStartConfig& warm_start_config,
                       const VehicleParam& vehicle_param);
  virtual ~HybridAStar() = default;//虚析构函数，声明在基类中，以确保派生类对象被正确销毁。本例中没有派生类
  
  // 主规划函数：从起点到终点进行路径规划
  bool Plan(const Pos3d& start_pose, const Pos3d& end_pose,
            const std::vector<double>& XYbounds,  // 地图边界 [x_min, x_max, y_min, y_max]
            const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec, // 障碍物顶点列表
            HybridAStartResult* result); // 输出结果

  // 轨迹分割：将规划出的轨迹分割成更小的段
  bool TrajectoryPartition(const HybridAStartResult& result,
                           std::vector<HybridAStartResult>* partitioned_result);

 private:
  // 分析性扩展：尝试使用Reeds-Shepp曲线直接连接当前节点到终点
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);
  
  // 检查路径是否有效（无碰撞）
  bool IsPathVaild(const std::vector<Pos3d>& curve_path);
  
  // 节点有效性检查：碰撞检测和有效性验证
  bool ValidityCheck(std::shared_ptr<Node3d> node);
  
  // 检查Reeds-Shepp路径的碰撞和有效性
  bool RSPCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end);
  
  // 将整个Reeds-Shepp路径加载为节点并添加到关闭集合
  std::shared_ptr<Node3d> LoadRSPinCS(
      const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
      std::shared_ptr<Node3d> current_node);
  
  // 生成最终节点
  std::shared_ptr<Node3d> GenerateFinalNode(
      const std::vector<Pos3d>& curve_path,
      std::shared_ptr<Node3d> current_node);
  
  // 下一个节点生成器：根据索引生成下一个可能的节点
  std::shared_ptr<Node3d> Next_node_generator(
      std::shared_ptr<Node3d> current_node, size_t next_node_index);
  
  // 计算节点成本：g(n) + h(n)
  void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node);
  
  // 轨迹段成本计算
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);
  
  // 无障碍物启发函数（欧几里得距离）
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  
  // 获取最终规划结果
  bool GetResult(HybridAStartResult* result);
  
  // 获取时间剖面（速度、加速度等）
  bool GetTemporalProfile(HybridAStartResult* result);
  
  // 生成速度和加速度剖面
  bool GenerateSpeedAcceleration(HybridAStartResult* result);
  // bool GenerateSCurveSpeedAcceleration(HybridAStartResult* result); // 可选：S曲线速度规划

 private:
  // 配置参数
  WarmStartConfig warm_start_config_;  // 热启动配置
  VehicleParam vehicle_param_;         // 车辆参数
  
  // 算法参数
  size_t next_node_num_ = 0;           // 下一个节点生成数量
  double max_steer_angle_ = 0.0;       // 最大前轮转角（弧度）
  double step_size_ = 0.0;             // 步长（模拟步长）
  double xy_grid_resolution_ = 0.0;    // XY网格分辨率
  double delta_t_ = 0.0;               // 时间步长
  
  // 轨迹成本权重参数
  double traj_forward_penalty_ = 0.0;       // 前进惩罚
  double traj_back_penalty_ = 0.0;          // 倒车惩罚
  double traj_gear_switch_penalty_ = 0.0;   // 换挡惩罚
  double traj_steer_penalty_ = 0.0;         // 转向惩罚
  double traj_steer_change_penalty_ = 0.0;  // 转向变化惩罚
  
  // 车辆运动学参数
  double max_kappa_ = 0.0;    // 最大曲率
  double min_radius_ = 0.0;   // 最小转弯半径
  
  // 启发函数权重参数（用于Reeds-Shepp启发式）
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  
  // 地图边界
  std::vector<double> XYbounds_;
  
  // 节点
  std::shared_ptr<Node3d> start_node_;   // 起点节点
  std::shared_ptr<Node3d> end_node_;     // 终点节点
  std::shared_ptr<Node3d> final_node_;   // 最终找到的节点
  
  // 障碍物表示（线段形式，便于碰撞检测）
  std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec_;

  // 优先队列比较器（用于open_set_，按成本从小到大排序）
  struct cmp {
    bool operator()(const std::pair<std::string, double>& left,
                    const std::pair<std::string, double>& right) const {
      return left.second >= right.second;
    }
  };
  
  // A*算法数据结构
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;  // 优先队列（存储节点ID和f(n)值）
  
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;   // 开放集合
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;  // 关闭集合
  
  // 工具类
  // std::unique_ptr<ReedShepp> reed_shepp_generator_;  // Reeds-Shepp曲线生成器（注释掉了）
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;//声明了一个智能指针变量 网格A*启发函数生成器
};

#endif