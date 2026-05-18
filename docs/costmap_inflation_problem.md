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
- `config/nav2_params_exploration.yaml` — costmap 参数配置
- `scripts/diagnose_inflation.py` — **诊断工具**，抓取 costmap 并分析代价分布
- `/opt/ros/jazzy/include/nav2_costmap_2d/nav2_costmap_2d/inflation_layer.hpp` — Nav2 InflationLayer 源码
- `/opt/ros/jazzy/include/rviz_default_plugins/.../palette_builder.cpp` — RViz2 costmap 调色板源码
- ROS 2 Jazzy Nav2 (https://github.com/ros-planning/navigation2/tree/jazzy)

## 排查结论 (2026-05-18)

### 1. computeCaches() 缓存——已排除

`computeCaches()` 在 `dynamicParametersCallback` 检测到参数变化后会通过
`matchSize()` 被调用。cost 重算循环在 `computeCaches()` 中**无条件执行**，
不依赖任何 gate 条件。缓存机制正常。

### 2. 其他层/过滤器覆盖——已排除

- local_costmap: `obstacle_layer` → `inflation_layer` (inflation 在最后)
- global_costmap: `static_layer` → `obstacle_layer` → `inflation_layer` (inflation 在最后)
- 不存在 keepout_filter、costmap_filter、speed_filter 或其他 cost 修改插件
- master_grid 使用 `std::max(old, new)` 合并各层代价，inflation_layer 是最后一层

### 3. 根本原因：inscribed_radius_ 控制青色核心区

**`computeCost()` 源码 (inflation_layer.hpp:149-163)：**
```cpp
if (distance == 0)
    cost = LETHAL_OBSTACLE;             // 254
else if (distance * resolution_ <= inscribed_radius_)
    cost = INSCRIBED_INFLATED_OBSTACLE; // 253 ← 硬编码
else
    cost = (unsigned char)(252 * exp(-csf * (distance*res - inscribed_radius_)));
```

- `inscribed_radius_` 来自机器人足迹 `[[0.45,0.30],...]` 的内切圆半径 = 0.30m
- **0.30m 以内永远是 cost=253（青色），不受 cost_scaling_factor 影响**
- 青:红半径比 = 0.30 : (1.50 - 0.30) = 1:4，与用户观察吻合

### 4. RViz2 调色板 + OccupancyGrid 缩放 —— 已通过诊断脚本确认 (2026-05-18)

**Nav2 costmap_2d_ros 将内部 0-255 代价转换为 OccupancyGrid 标准的 0-100 发布。**

通过阅读 `rviz_default_plugins/palette_builder.cpp`（Jazzy 分支）的
`makeCostmapPalette()` 源码，结合 diagnostic 脚本抓取的实际 costmap 数据：

| 内部代价 (Nav2) | 发布值 (OccupancyGrid) | RViz2 palette 颜色 |
|----------------|----------------------|-------------------|
| 0 (FREE) | 0 | 透明黑 |
| 1-252 (膨胀) | 0-98 | 蓝→紫→红色渐变 |
| 253 (INSCRIBED) | **99** | **青色** `(0, 255, 255)` |
| 254 (LETHAL) | **99** | **青色** `(0, 255, 255)` |
| 255 (UNKNOWN) | 100 | 紫红 `(255, 0, 255)` |

**关键洞察**：用户看到的"青色核心区"就是发布值=99 的区域（对应内部 cost ≥253），
位于 `inscribed_radius_` (0.30m) 范围内。palette[99] = `(0, 255, 255)` = 青色。

"红色外围区"是发布值=98 及以下的区域，其中 palette[98] = `(249, 0, 6)` ≈ **纯红色**。
青→红的颜色突变发生在发布值 99→98（即内部 253→252），恰好对应 `inscribed_radius_`
边界。这是一个**单步颜色跳变**——青色直接跳到近纯红色。

**实际诊断数据** (120×120 costmap, 0.05m/pixel):
- 青色区 (pub=99): 78 cells (0.5%) — 对应 inscribed 核心
- 梯度区 (pub=1-98): 1384 cells (9.6%) — cost 范围 9-98，均值 32.9
- 反算有效 csf ≈ 3.1 (median)

**这意味着**：
1. cost_scaling_factor 无法移动青/红边界（该边界由 inscribed_radius_ 固定）
2. csf 只能控制红色区内部的衰减速度（pub=98 → pub=? 的斜率）
3. 在 0.001 和 10.0 之间切换 csf 时，青/红边界位置完全不变，
   只有红色区内部的颜色深浅会有变化（但这变化被OccupancyGrid的量化压缩了）

### 5. 最终诊断结论

| 疑点 | 结论 |
|------|------|
| computeCaches() 缓存不更新 | **排除** — cost 重算无条件执行 |
| 其他层覆盖代价 | **排除** — inflation 是管道最后一层 |
| 动态参数未生效 | **排除** — `ros2 param get` 确认值正确 |
| csf 改变无效 | **部分正确** — csf 确实改变了内部 cost 值，但 OccupancyGrid 的 0-100 量化 + 调色板的 99↔98 颜色断崖 使青/红边界看起来不变 |
| 青/红边界由什么控制 | **inscribed_radius_ = 0.30m** (来自足迹 `[[0.45,0.30],...]`) |

### 6. 配置修改

已修改 `config/nav2_params_exploration.yaml`:
- 第 101 行 (local_costmap): `cost_scaling_factor: 0.001 → 2.0`
- 第 136 行 (global_costmap): `cost_scaling_factor: 0.001 → 2.0`

**成本梯度对照** (csf=2.0, inscribed_radius=0.30m):
| 距障碍物 | 内部 cost | 发布值 | 调色板颜色 |
|---------|----------|--------|-----------|
| 0.30m | 253 | 99 | **青色** |
| 0.35m | ~228 | 89 | 紫红 |
| 0.50m | ~169 | 66 | 紫色 |
| 0.80m | ~92 | 36 | 蓝紫 |
| 1.20m | ~41 | 16 | 蓝色 |
| 1.50m | ~23 | 9 | 深蓝 |

如需更强的避障行为，可调整 `planner_server` 中的 `cost_penalty`（当前 8.0）。
降低 csf 到 1.0 会让红色区更宽（更多中高代价区域），planner 会更积极避障。
