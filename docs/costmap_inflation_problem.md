# 问题：Nav2 InflationLayer 膨胀层青色核心区无法扩大

## 目标
在 RViz2 中，障碍物膨胀层的青色(cost ~253)核心半径远远小于红色外围区(cost ~200-252)。
希望**扩大青色核心区，缩小红色外围区**，使规划的路径远离障碍物。

## 配置
文件: `/home/hello/lidar-slam/config/nav2_params_exploration.yaml`

两个 costmap (local_costmap 和 global_costmap) 的 InflationLayer 参数完全相同：
```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 0.001   # 已从 10.0 → 0.05 → 0.001，视觉效果无变化
  inflation_radius: 1.5
```

ros__parameters 级别**没有**冗余的 cost_scaling_factor/inflation_radius（已移除，与 Nav2 默认结构一致）。

## 核心矛盾（需要解释）

根据 Nav2 InflationLayer 的 `computeCost()` 公式（[inflation_layer.hpp:149-163](https://github.com/ros-planning/navigation2/blob/jazzy/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp#L149)）：

```
cost = 253 × exp(-cost_scaling_factor × (distance × resolution - inscribed_radius))
```

当 `cost_scaling_factor = 0.001` 时：
- **1.5m 膨胀半径末端**: cost = 253 × exp(-0.001 × 1.2) = 253 × 0.9988 = **252.7**
- **整个 1.5m 半径内**: cost 范围 **252.7 ~ 253**，仅差 0.3 个单位
- **预测视觉效果**: 整个膨胀区颜色应几乎均匀一致

但实际在 RViz2 中观察到：青色核心区很小，红色外围区很大（半径比约 1:4~1:5），
说明 **cost 在空间上发生了显著衰减**，与公式预测矛盾。

而且从 cost_scaling_factor=10.0 降到 0.001，视觉效果**几乎没有变化**——这强烈暗示
InflationLayer 在计算 cost 时**可能没有真正使用这个参数值**。

## 已确认的事实

1. **参数存在且值正确**: `ros2 param get /local_costmap/local_costmap inflation_layer.cost_scaling_factor` 返回 `0.001`
2. **参数路径正确**: InflationLayer 通过 `name_ + "." + "cost_scaling_factor"` = `"inflation_layer.cost_scaling_factor"` 读取参数（源码: [inflation_layer.cpp:95](https://github.com/ros-planning/navigation2/blob/jazzy/nav2_costmap_2d/plugins/inflation_layer.cpp#L95)）
3. **YAML 结构正确**: local_costmap 和 global_costmap 均为顶层 key（与 Nav2 默认 params 结构一致）
4. **RewrittenYaml 处理正确**: navigation_launch.py 使用 `RewrittenYaml(source_file, root_key="", convert_types=True)`
5. **inflation_radius=1.5 生效**: 膨胀区总范围确实变大了（红色外围从默认 0.55m 扩展到 1.5m）

## 已排除的可能

- cost_scaling_factor 参数值未加载到节点 → **已排除**（ros2 param get 确认）
- YAML 参数嵌套层级错误 → **已排除**（与 Nav2 默认 params 结构完全一致）
- RewrittenYaml 破坏参数 → **已排除**（root_key="" 时保持原结构）
- cost_scaling_factor 值方向搞反 → 已用极端值 0.001 测试，仍无效果

## 待调查的方向

1. **InflationLayer 的 `computeCaches()` 是否使用旧缓存**：
   - `matchSize()` 会调用 `computeCaches()` 重新计算代价缓存
   - 但 `matchSize()` 只在 costmap 尺寸变化时被调用
   - 如果尺寸不变，缓存的 cost 值不会更新，即使用了新的 cost_scaling_factor

2. **cost_scaling_factor 是否在 InflationLayer 初始化后被动态覆盖**：
   - InflationLayer 注册了 `dynamicParametersCallback`
   - 是否有其他节点/进程在运行时修改了这个参数？

3. **RViz2 的颜色映射是否与预期不同**：
   - 用户确认的配色: 青色=核心(近障碍物)，红色=外围(远离障碍物)
   - 这可能与标准 costmap 调色板相反或不同

4. **是否有其他层或过滤器在产生"红色"代价**：
   - local_costmap 有 obstacle_layer + inflation_layer
   - global_costmap 有 static_layer + obstacle_layer + inflation_layer
   - 是否存在层间 cost 合并导致的非预期效果？

5. **InflationLayer 的 `computeCost()` 使用的 `distance` 参数单位是否正确**：
   - 参数 `distance` 是 cells 数
   - 通过 `distance * resolution_` 转换为米
   - `resolution_` 是否正确设置（应来自 costmap 的 resolution）？

## 关键文件
- /home/hello/lidar-slam/config/nav2_params_exploration.yaml
- /opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/inflation_layer.hpp
- ROS 2 Jazzy Nav2 (https://github.com/ros-planning/navigation2/tree/jazzy)
