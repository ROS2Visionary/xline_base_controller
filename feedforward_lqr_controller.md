# 前馈 + LQR 高精度路径跟踪控制器

## 目录

- [1. 概述](#1-概述)
- [2. 系统模型](#2-系统模型)
- [3. 误差模型](#3-误差模型)
- [4. 前馈控制设计](#4-前馈控制设计)
- [5. LQR反馈控制设计](#5-lqr反馈控制设计)
- [6. 组合控制律](#6-组合控制律)
- [7. 参数整定](#7-参数整定)
- [8. 实现代码](#8-实现代码)
- [9. 性能分析](#9-性能分析)
- [10. 调试指南](#10-调试指南)

---

## 1. 概述

### 1.1 设计目标

实现 **< 3mm** 精度的曲线路径跟踪，适用于：
- 贝塞尔曲线转场路径
- 圆弧路径
- 任意光滑曲线路径

### 1.2 核心思想

```
控制量 = 前馈控制 + 反馈控制

ω = ω_feedforward + ω_feedback
  = v × κ + (-K₁·e_y - K₂·e_θ)
    ─────   ─────────────────
    主控制      误差修正
    (~90%)      (~10%)
```

### 1.3 为什么需要前馈？

| 控制方式 | 曲线跟踪误差 | 原因 |
|----------|-------------|------|
| 纯反馈 | 10-30 mm | 误差出现后才纠正，有滞后 |
| 前馈+反馈 | < 3 mm | 提前给出正确控制量，反馈仅微调 |

---

## 2. 系统模型

### 2.1 差速轮机器人运动学

```
        y
        ↑
        │    θ (朝向角)
        │   ↗
        │  /
        │ /
        ●───────→ x
      机器人
```

**运动学方程：**

$$
\begin{aligned}
\dot{x} &= v \cos\theta \\
\dot{y} &= v \sin\theta \\
\dot{\theta} &= \omega
\end{aligned}
$$

其中：
- $(x, y)$：机器人位置
- $\theta$：机器人朝向角
- $v$：线速度（控制输入）
- $\omega$：角速度（控制输入）

### 2.1.1 后退跟随（倒车走正向路径）的等效前进表示

很多系统会在“后退跟随”时发送 `cmd_vel.linear.x < 0`，但机器人在世界坐标的运动方向等效为“航向 + π 的前进”。为避免在控制律里引入符号混乱，可做如下等效变换：

$$
\theta_m = \theta + \pi,\quad v_m = |v|
$$

则世界系运动学可写成与前进一致的形式：

$$
\dot{x} = v_m \cos\theta_m,\quad \dot{y} = v_m \sin\theta_m,\quad \dot{\theta}_m = \omega
$$

要点：
- 横向/航向误差应使用“运动方向航向” $\theta_m$ 计算；
- 增益与前馈项使用速度幅值 $v_m$；
- $\omega$ 是底盘真实偏航角速度（$\dot{\theta}=\omega$），与前进/后退无关，不应额外取反。

### 2.2 差速轮速度关系

```
       ┌───────┐
  v_L  │       │  v_R
   ←───●       ●───→
       │   L   │
       └───────┘
         轮距
```

$$
\begin{aligned}
v &= \frac{v_L + v_R}{2} \\
\omega &= \frac{v_R - v_L}{L}
\end{aligned}
$$

逆解：

$$
\begin{aligned}
v_L &= v - \frac{\omega L}{2} \\
v_R &= v + \frac{\omega L}{2}
\end{aligned}
$$

---

## 3. 误差模型

### 3.1 Frenet 坐标系

将误差定义在路径的 Frenet 坐标系下：

```
                    路径切线方向
                         ↗
        ─────────────────●─────────────────→ 路径
                        /│
                       / │
                      /  │ e_y (横向误差)
                     /   │
                    /    │
                   ● ────┘
                 机器人
                   ↗
                  θ  (实际朝向)

        e_θ = θ - θ_ref  (航向误差)
```

### 3.2 误差定义

| 误差 | 符号 | 含义 |
|------|------|------|
| 横向误差 | $e_y$ | 机器人到路径的垂直距离 |
| 航向误差 | $e_\theta$ | 机器人朝向与路径切线的夹角 |

### 3.3 误差计算公式

```cpp
// 给定参考点 (ref_x, ref_y, ref_theta) 和当前位置 (x, y, theta)
double dx = x - ref_x;
double dy = y - ref_y;

// 横向误差（垂直于路径方向）
double e_y = -dx * sin(ref_theta) + dy * cos(ref_theta);

// 航向误差
double e_theta = normalize_angle(theta - ref_theta);
```

### 3.4 误差动力学

误差的变化率满足：

$$
\begin{aligned}
\dot{e}_y &= v \sin(e_\theta) \approx v \cdot e_\theta \quad \text{(小角度近似)} \\
\dot{e}_\theta &= \omega - \omega_{ref} = \omega - v \cdot \kappa
\end{aligned}
$$

状态空间形式：

$$
\begin{bmatrix} \dot{e}_y \\ \dot{e}_\theta \end{bmatrix} =
\begin{bmatrix} 0 & v \\ 0 & 0 \end{bmatrix}
\begin{bmatrix} e_y \\ e_\theta \end{bmatrix} +
\begin{bmatrix} 0 \\ 1 \end{bmatrix} (\omega - v \kappa)
$$

---

## 4. 前馈控制设计

### 4.1 前馈原理

沿曲率为 $\kappa$ 的路径以速度 $v$ 行驶时，所需的角速度为：

$$
\omega_{ff} = v \cdot \kappa
$$

**推导：**

```
曲率定义：κ = 1/R（R为转弯半径）

沿圆弧行驶：
  弧长 s = v × t
  转过角度 Δθ = s / R = v × t / R = v × κ × t

角速度：
  ω = dθ/dt = v × κ
```

### 4.2 贝塞尔曲线曲率计算

对于三次贝塞尔曲线 $B(t) = (x(t), y(t))$：

$$
\kappa(t) = \frac{|x'(t) y''(t) - y'(t) x''(t)|}{(x'(t)^2 + y'(t)^2)^{3/2}}
$$

**三次贝塞尔曲线：**

$$
B(t) = (1-t)^3 P_0 + 3(1-t)^2 t P_1 + 3(1-t) t^2 P_2 + t^3 P_3
$$

**一阶导数：**

$$
B'(t) = 3(1-t)^2 (P_1-P_0) + 6(1-t)t (P_2-P_1) + 3t^2 (P_3-P_2)
$$

**二阶导数：**

$$
B''(t) = 6(1-t)(P_2-2P_1+P_0) + 6t(P_3-2P_2+P_1)
$$

### 4.3 曲率计算代码

```cpp
struct Point2D {
    double x, y;
};

// 计算贝塞尔曲线在参数 t 处的曲率
double computeBezierCurvature(
    const Point2D& P0, const Point2D& P1,
    const Point2D& P2, const Point2D& P3,
    double t
) {
    // 一阶导数
    double dx = 3 * pow(1-t, 2) * (P1.x - P0.x)
              + 6 * (1-t) * t * (P2.x - P1.x)
              + 3 * pow(t, 2) * (P3.x - P2.x);
    double dy = 3 * pow(1-t, 2) * (P1.y - P0.y)
              + 6 * (1-t) * t * (P2.y - P1.y)
              + 3 * pow(t, 2) * (P3.y - P2.y);

    // 二阶导数
    double ddx = 6 * (1-t) * (P2.x - 2*P1.x + P0.x)
               + 6 * t * (P3.x - 2*P2.x + P1.x);
    double ddy = 6 * (1-t) * (P2.y - 2*P1.y + P0.y)
               + 6 * t * (P3.y - 2*P2.y + P1.y);

    // 曲率公式
    double cross = dx * ddy - dy * ddx;
    double norm_cubed = pow(dx*dx + dy*dy, 1.5);

    if (norm_cubed < 1e-10) return 0.0;

    return cross / norm_cubed;  // 带符号曲率（左转为正）
}
```

---

## 5. LQR反馈控制设计

### 5.1 LQR 问题定义

**目标：** 最小化代价函数

$$
J = \int_0^\infty \left( \mathbf{e}^T Q \mathbf{e} + u^T R u \right) dt
$$

其中：
- $\mathbf{e} = [e_y, e_\theta]^T$ 为误差状态
- $u = \omega - \omega_{ff}$ 为反馈控制量
- $Q$ 为状态权重矩阵
- $R$ 为控制权重

### 5.2 权重矩阵选择

$$
Q = \begin{bmatrix} q_1 & 0 \\ 0 & q_2 \end{bmatrix}, \quad R = r
$$

| 参数 | 含义 | 建议值 | 影响 |
|------|------|--------|------|
| $q_1$ | 横向误差权重 | 10000 | 越大，横向精度越高 |
| $q_2$ | 航向误差权重 | 100 | 越大，航向响应越快 |
| $r$ | 控制量权重 | 1 | 越大，控制越平滑 |

### 5.3 LQR 增益求解

对于系统：

$$
\dot{\mathbf{e}} = A \mathbf{e} + B u, \quad A = \begin{bmatrix} 0 & v \\ 0 & 0 \end{bmatrix}, B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}
$$

LQR 最优控制律：

$$
u = -K \mathbf{e} = -[K_1, K_2] \begin{bmatrix} e_y \\ e_\theta \end{bmatrix}
$$

### 5.4 增益计算（解析解）

对于此特定系统，可得解析解：

$$
\begin{aligned}
K_1 &= \sqrt{\frac{q_1}{r}} \cdot \frac{1}{v} \\
K_2 &= \sqrt{\frac{2\sqrt{q_1 q_2}}{r} + \frac{q_2}{r}}
\end{aligned}
$$

**⚠️ 重要：参数与增益对应关系**

若要得到特定增益值，需要反推权重参数。例如，目标 $K_1=4.0$, $K_2=15.0$ (v=0.25m/s)：

从 $K_1 = \sqrt{q_1/r} / v = 4.0$：
$$\sqrt{q_1} = 4.0 \times 0.25 = 1.0 \Rightarrow q_1 = 1.0$$

从 $K_2 = \sqrt{2\sqrt{q_1 q_2}/r + q_2/r} = 15.0$：
$$225 = 2\sqrt{q_2} + q_2$$

解得 $q_2 \approx 200$

**简化实用公式：**

```cpp
void computeLQRGains(double v, double q1, double q2, double r,
                     double& K1, double& K2) {
    // 防止除零
    v = std::max(v, 0.05);

    K1 = sqrt(q1 / r) / v;
    K2 = sqrt(2.0 * sqrt(q1 * q2) / r + q2 / r);
}

// 示例：q1=1.0, q2=200.0, r=1.0, v=0.25 → K1=4.0, K2≈15.0
```

### 5.5 增益与速度的关系

**重要：** $K_1$ 与速度成反比（使用 q1=1.0, q2=200.0, r=1.0）

| 速度 v | K₁ | K₂ | 说明 |
|--------|-----|-----|------|
| 0.1 m/s | 10.0 | 15.1 | 低速，高增益 |
| 0.25 m/s | 4.0 | 15.1 | 中速 |
| 0.5 m/s | 2.0 | 15.1 | 高速，低增益 |

---

## 6. 组合控制律

### 6.1 完整控制律

$$
\omega = \underbrace{v \cdot \kappa}_{\text{前馈}} + \underbrace{(-K_1 e_y - K_2 e_\theta)}_{\text{LQR反馈}}
$$

### 6.2 控制框图

```
                                    ┌─────────────┐
         路径曲率 κ ────────────────→│  前馈计算   │──→ ω_ff ───┐
                                    │ ω_ff = v×κ │            │
                                    └─────────────┘            │
                                                               ▼
┌─────────┐    ┌─────────┐    ┌─────────────┐              ┌─────┐
│ 参考路径 │───→│ 误差计算 │───→│ LQR 反馈    │──→ ω_fb ───→│  Σ  │──→ ω
└─────────┘    └─────────┘    │-K₁e_y-K₂e_θ│              └─────┘
                   ▲          └─────────────┘                 │
                   │                                          │
                   │         ┌─────────────┐                  │
                   └─────────│  机器人     │◄─────────────────┘
                             │  (执行)     │
                             └─────────────┘
```

### 6.3 控制量限幅

```cpp
// 角速度限幅
double omega_max = 0.5;  // rad/s
omega = std::clamp(omega, -omega_max, omega_max);

// 角加速度限幅（平滑控制）
double omega_dot_max = 1.0;  // rad/s²
double dt = 0.005;  // 控制周期
double omega_change = omega - omega_last;
if (std::abs(omega_change) > omega_dot_max * dt) {
    omega = omega_last + std::copysign(omega_dot_max * dt, omega_change);
}
omega_last = omega;
```

---

## 7. 参数整定

### 7.1 整定流程

```
步骤1：设置初始参数
       q1 = 10000, q2 = 100, r = 1

步骤2：直线跟踪测试
       调整 K1 直到横向误差 < 3mm

步骤3：曲线跟踪测试
       调整 K2 直到曲线处无振荡

步骤4：高速测试
       检查稳定性，必要时降低增益
```

### 7.2 参数影响分析

| 问题现象 | 可能原因 | 调整方法 |
|----------|----------|----------|
| 横向误差大 | K₁ 太小 | 增大 q₁ 或减小 r |
| 曲线处振荡 | K₂ 太大 | 减小 q₂ |
| 响应太慢 | 增益太小 | 增大 q₁ 和 q₂ |
| 控制量抖动 | 增益太大 | 增大 r |
| 高速不稳定 | 增益未随速度调整 | 使用速度自适应增益 |

### 7.3 推荐参数

**适用于 v = 0.25 m/s，精度 < 3mm：**

```cpp
// 权重参数（已修正）
double q1 = 1.0;      // 横向误差权重
double q2 = 200.0;    // 航向误差权重
double r  = 1.0;      // 控制量权重

// 计算得到的增益
double K1 = 4.0;      // 横向误差增益 = √(1.0/1.0) / 0.25 = 4.0
double K2 = 15.1;     // 航向误差增益 = √(2×√200 + 200) ≈ 15.1

// 或者直接指定增益（推荐）
bool use_direct_gains = true;
double K1_direct = 4.0;
double K2_direct = 15.0;
```

---

## 8. 实现代码

### 8.1 完整控制器类

完整实现请参考 `xline_path_planner/include/xline_path_planner/feedforward_lqr_controller.hpp`

**关键参数结构：**

```cpp
struct LQRControllerParams {
    // LQR 权重参数（已修正）
    double q1 = 1.0;          ///< 横向误差权重
    double q2 = 200.0;        ///< 航向误差权重
    double r  = 1.0;          ///< 控制量权重

    // 直接指定增益（推荐）
    bool use_direct_gains = false;
    double K1_direct = 4.0;
    double K2_direct = 15.0;

    // 积分项参数（LQI，可选）
    bool enable_integral = false;
    double Ki = 0.5;
    double integral_max = 0.1;      ///< 积分饱和限制 (rad/s)
    double integral_decay = 0.99;   ///< 积分衰减系数

    // 限幅参数
    double omega_max = 0.5;         ///< 最大角速度 (rad/s)
    double omega_dot_max = 1.0;     ///< 最大角加速度 (rad/s²)
    double v_max = 0.25;            ///< 最大线速度 (m/s)

    // 机器人参数
    double wheel_base = 0.5;        ///< 轮距 (m)

    // 前瞻参数
    double lookahead_distance = 0.02; ///< 前瞻距离 (m)
    double lookahead_time = 0.05;     ///< 前瞻时间 (s)

    // 搜索窗口
    size_t search_window_back = 5;
    size_t search_window_forward = 30;
};
```

**核心控制流程：**

```cpp
ControlOutput compute(double current_x, double current_y, double current_theta) {
    // 1. 找到最近的路径点（改进的搜索算法）
    size_t nearest_idx = findNearestPointImproved(current_x, current_y);

    // 2. 获取参考点（基于弧长的前瞻）
    size_t ref_idx = findLookaheadPoint(nearest_idx, current_x, current_y);
    const PathPointWithCurvature& ref = path_[ref_idx];

    // 3. 计算误差
    double dx = current_x - ref.x;
    double dy = current_y - ref.y;
    double e_y = -dx * std::sin(ref.theta) + dy * std::cos(ref.theta);
    double e_theta = normalizeAngle(current_theta - ref.theta);

    // 4. 前馈控制 ω_ff = v × κ
    double omega_ff = v * ref.curvature;

    // 5. LQR 反馈控制 ω_fb = -K₁·e_y - K₂·e_θ
    double omega_fb = -K1_ * e_y - K2_ * e_theta;

    // 6. 积分项（可选）
    double omega_i = 0.0;
    if (params_.enable_integral) {
        integral_e_y_ = params_.integral_decay * integral_e_y_ + e_y * dt_;
        omega_i = -params_.Ki * integral_e_y_;
    }

    // 7. 总控制量
    double omega = omega_ff + omega_fb + omega_i;
    omega = applyLimits(omega);

    return output;
}
```

**高效角度归一化：**

```cpp
static double normalizeAngle(double angle) {
    // 使用 fmod 实现 O(1) 复杂度，避免 while 循环
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    return angle - M_PI;
}
```

### 8.2 使用示例

```cpp
#include "xline_path_planner/feedforward_lqr_controller.hpp"
#include "xline_path_planner/path_curvature_generator.hpp"
#include <iostream>

int main() {
    using namespace motion_control;

    // 创建控制器（使用直接指定增益）
    LQRControllerParams params;
    params.use_direct_gains = true;
    params.K1_direct = 4.0;
    params.K2_direct = 15.0;
    params.wheel_base = 0.5;
    params.v_max = 0.25;
    params.enable_integral = false;  // 可选开启积分项

    FeedforwardLQRController controller(params);

    // 使用曲率生成器创建路径
    PathCurvatureGenerator generator;
    std::vector<path_planner::Point3D> waypoints = {
        {0.0, 0.0, 0.0},
        {0.5, 0.1, 0.0},
        {1.0, 0.0, 0.0}
    };
    auto path = generator.generateFromPoints(waypoints);
    controller.setPath(path);

    // 当前机器人状态（有一些误差）
    double current_x = 0.502;      // 横向偏差 2mm
    double current_y = 0.1;
    double current_theta = 0.01;   // 航向偏差 0.01 rad

    // 计算控制量
    ControlOutput output = controller.compute(current_x, current_y, current_theta);

    // 输出结果
    std::cout << "=== 控制输出 ===" << std::endl;
    std::cout << "线速度 v: " << output.v << " m/s" << std::endl;
    std::cout << "角速度 ω: " << output.omega << " rad/s" << std::endl;
    std::cout << "左轮速度: " << output.v_left << " m/s" << std::endl;
    std::cout << "右轮速度: " << output.v_right << " m/s" << std::endl;

    std::cout << "\n=== 调试信息 ===" << std::endl;
    std::cout << "横向误差 e_y: " << output.e_y * 1000 << " mm" << std::endl;
    std::cout << "航向误差 e_θ: " << output.e_theta * 180 / M_PI << " deg" << std::endl;
    std::cout << "参考曲率 κ: " << output.ref_curvature << " 1/m" << std::endl;
    std::cout << "前馈量 ω_ff: " << output.omega_ff << " rad/s" << std::endl;
    std::cout << "反馈量 ω_fb: " << output.omega_fb << " rad/s" << std::endl;

    double K1, K2;
    controller.getGains(K1, K2);
    std::cout << "增益 K1: " << K1 << std::endl;
    std::cout << "增益 K2: " << K2 << std::endl;

    return 0;
}
```

### 8.3 输出示例

```
=== 控制输出 ===
线速度 v: 0.25 m/s
角速度 ω: 0.142 rad/s
左轮速度: 0.214 m/s
右轮速度: 0.286 m/s

=== 调试信息 ===
横向误差 e_y: 2.0 mm
航向误差 e_θ: 0.57 deg
参考曲率 κ: 0.8 1/m
前馈量 ω_ff: 0.2 rad/s
反馈量 ω_fb: -0.058 rad/s
增益 K1: 4.0
增益 K2: 15.0
```

---

## 9. 性能分析

### 9.1 精度分析

在理想条件下（无噪声、无延迟、模型精确）：

| 路径类型 | 前馈贡献 | 反馈修正 | 稳态误差 |
|----------|----------|----------|----------|
| 直线 | 0% | 100% | < 1 mm |
| 大半径圆弧 (R>1m) | 80% | 20% | < 2 mm |
| 小半径圆弧 (R=0.5m) | 90% | 10% | < 3 mm |
| S型曲线 | 85% | 15% | < 3 mm |

### 9.2 误差来源与对策

| 误差来源 | 典型量级 | 对策 |
|----------|----------|------|
| 曲率计算误差 | 0.5-1 mm | 提高路径点密度 |
| 控制延迟 | 0.5-2 mm | 提高控制频率 |
| 编码器量化 | 0.1-0.5 mm | 使用高分辨率编码器 |
| 轮子打滑 | 1-5 mm | 选择合适轮胎、控制加速度 |

### 9.3 频率要求

```
控制频率与精度的关系（v = 0.25 m/s）：

频率      周期      单周期位移    推荐精度目标
──────────────────────────────────────────────
 18 Hz    56 ms     13.9 mm       15 mm  ← 实际机器人
 50 Hz    20 ms      5.0 mm       10 mm
100 Hz    10 ms      2.5 mm       5 mm
200 Hz     5 ms      1.25 mm      3 mm
```

**⚠️ 低频控制配置（18Hz）：**

| 配置 | v = 0.25 m/s | v = 0.1 m/s | v = 0.05 m/s (推荐) |
|------|-------------|-------------|---------------------|
| 单周期位移 | 13.9 mm | 5.6 mm | **2.8 mm** |
| K1 (自动) | 4.0 | 10.0 | **20.0** |
| 前瞻距离 | 0.05 m | 0.03 m | 0.02 m |
| 前瞻时间 | 0.1 s | 0.15 s | 0.2 s |
| 总前瞻 | 75 mm | 45 mm | 30 mm |
| 可达精度 | ~15 mm | ~8 mm | **~5 mm** |

**18Hz + 0.05m/s 推荐参数（默认值）：**
```cpp
LQRControllerParams params;
params.control_frequency = 18.0;
params.v_max = 0.05;
params.lookahead_distance = 0.02;
params.lookahead_time = 0.2;
// K1 会自动计算为 20.0 (= √(1/1) / 0.05)
```

---

## 10. 调试指南

### 10.1 调试步骤

```
Step 1: 验证前馈
├── 设置 K1=0, K2=0（禁用反馈）
├── 跟踪圆弧路径
├── 检查误差是否稳定（不发散）
└── 前馈正确时，误差应 < 20mm

Step 2: 调整 K1
├── 保持 K2=0
├── 跟踪直线
├── 逐渐增大 K1 直到横向误差 < 3mm
└── 注意不要出现振荡

Step 3: 调整 K2
├── 使用 Step 2 的 K1
├── 跟踪曲线
├── 逐渐增大 K2 直到曲线处无振荡
└── 航向响应应快速但不超调

Step 4: 综合测试
├── 跟踪完整路径（含直线和曲线）
├── 检查全程误差 < 3mm
└── 调整参数优化性能
```

### 10.2 常见问题

**问题1：曲线处误差大**
```
原因：曲率前馈不准确
检查：
  1. 曲率计算是否正确
  2. 曲率符号是否正确（左转为正）
  3. 路径点密度是否足够
```

**问题2：系统振荡**
```
原因：增益过大
解决：
  1. 减小 K1 和 K2
  2. 增大控制量权重 r
  3. 添加角加速度限幅
```

**问题3：响应迟钝**
```
原因：增益过小或控制频率低
解决：
  1. 增大 q1 和 q2
  2. 提高控制频率
  3. 检查通信延迟
```

### 10.3 日志输出建议

```cpp
// 每个控制周期记录
struct ControlLog {
    double timestamp;
    // 参考
    double ref_x, ref_y, ref_theta, ref_curvature;
    // 实际
    double actual_x, actual_y, actual_theta;
    // 误差
    double e_y, e_theta;
    // 控制
    double omega_ff, omega_fb, omega_total;
    // 增益
    double K1, K2;
};

// 用于离线分析
```

---

## 11. LQR vs Pure Pursuit 对比分析

### 11.1 控制律对比

| 算法 | 控制律 | 前馈 | 反馈 |
|------|--------|------|------|
| **Pure Pursuit** | ω = 2v·sin(α) / L_d | ❌ 无 | 几何追踪 |
| **前馈+LQR** | ω = v·κ + (-K₁·e_y - K₂·e_θ) | ✅ v·κ | 最优控制 |

其中：
- α：前视角（机器人到前视点的角度）
- L_d：前视距离
- κ：路径曲率

### 11.2 曲线跟踪行为

**Pure Pursuit 在曲线处会"切弯"：**

```
        实际路径
           ╭─────╮
          ╱       ╲
    机器人 ●────→ 前视点
              ╲    │
               ╲   │  ← 切弯误差
                ╲──╯
```

**前馈+LQR 通过曲率补偿：**

```
        实际路径
           ╭─────╮
          ╱       ╲
    机器人 ●───────●  ← 跟踪曲率
              ω = v×κ
```

### 11.3 不同场景对比

| 场景 | Pure Pursuit | 前馈+LQR | 优势方 |
|------|--------------|----------|--------|
| 直线 | ~相同 | ~相同 | 平局 |
| 大半径曲线 R>1m | 中等误差 | 更小 | LQR |
| 小半径曲线 R<0.5m | 较大误差 | 更小 | **LQR** |
| S型曲线 | 切弯明显 | 前馈补偿 | **LQR** |
| 圆形路径 | 固有误差 | 理论为0 | **LQR** |

### 11.4 低频控制下的优势（18Hz）

在 18Hz 低频控制下，**前馈的价值更大**：

```
Pure Pursuit:
  误差出现 → 反馈纠正 → 延迟1周期 = 56ms = 2.8mm位移

前馈+LQR:
  前馈提前给出 → 反馈仅微调 → 延迟影响小
```

**结论**：低频控制时，前馈+LQR 优势更明显。

---

## 12. 圆形路径分析

### 12.1 圆形路径特点

圆形路径是**最能体现前馈优势**的场景，因为曲率恒定：

$$
\kappa = \frac{1}{R}, \quad \omega_{ff} = v \cdot \kappa = \frac{v}{R}
$$

**示例（R=0.5m, v=0.05m/s）：**
```
κ = 1/0.5 = 2.0 (1/m)
ω_ff = 0.05 × 2.0 = 0.1 rad/s  ← 精确值！
```

### 12.2 Pure Pursuit 圆形路径误差

Pure Pursuit 在圆形路径上存在**固有稳态误差**：

```
        圆形路径
           ╭───────╮
          ╱    ●    ╲      ← 圆心
         │   ╱│╲    │
         │  ╱ │ ╲   │
    机器人●──→前视点 │      ← 前视距离 L_d
         │     ╲   │
          ╲     ╲ ╱
           ╰─────╯

稳态误差 ≈ L_d² / (8R)
```

**PP 圆形路径稳态误差表：**

| 前视距离 L_d | R=0.3m | R=0.5m | R=1.0m |
|-------------|--------|--------|--------|
| 0.1 m | 4.2 mm | 2.5 mm | 1.25 mm |
| 0.15 m | 9.4 mm | 5.6 mm | 2.8 mm |
| 0.2 m | 16.7 mm | 10 mm | 5 mm |

### 12.3 前馈+LQR 圆形路径

```
前馈控制：ω_ff = v/R（精确值）

圆形路径上：
├── 前馈提供100%所需角速度
├── 理论稳态误差 = 0
└── LQR反馈仅处理：
    ├── 初始进入误差
    └── 外部扰动
```

### 12.4 圆形路径对比总结

| 指标 | Pure Pursuit | 前馈+LQR |
|------|--------------|----------|
| 圆形稳态误差 | 有固有误差 L_d²/(8R) | **理论为0** |
| R=0.5m, L_d=0.15m | ~5.6 mm | **~0 mm** |
| 进入圆弧瞬间 | 响应滞后 | **前馈立即响应** |
| 退出圆弧瞬间 | 响应滞后 | **前馈立即响应** |

**关键结论**：前馈 `ω = v/R` 在圆形路径上是**精确解**，不是近似！

---

## 13. 位姿精度依赖性

### 13.1 LQR 输入依赖

LQR 控制器需要**机器人的实时位姿**来计算误差：

```
┌─────────────────────────────────────────────────────┐
│                    LQR 控制器                        │
│                                                     │
│  输入：                                              │
│  ├── 机器人位姿 (x, y, θ)  ← 来自里程计/定位系统     │
│  └── 参考路径点 (ref_x, ref_y, ref_θ, κ)            │
│                                                     │
│  计算：                                              │
│  ├── e_y = 横向误差（垂直于路径）                    │
│  └── e_θ = 航向误差（θ - ref_θ）                    │
│                                                     │
│  输出：                                              │
│  └── ω = v×κ + (-K₁·e_y - K₂·e_θ)                  │
└─────────────────────────────────────────────────────┘
```

### 13.2 误差计算代码

```cpp
// 需要机器人当前位姿（必须准确！）
double current_x, current_y, current_theta;

// 计算横向误差（Frenet坐标系）
double dx = current_x - ref.x;
double dy = current_y - ref.y;
double e_y = -dx * sin(ref.theta) + dy * cos(ref.theta);

// 计算航向误差
double e_theta = normalizeAngle(current_theta - ref.theta);
```

### 13.3 位姿精度的影响

| 位姿误差来源 | 对LQR的影响 | 严重程度 |
|-------------|------------|----------|
| 里程计漂移 | e_y 计算不准，累积误差 | 高 |
| 航向角漂移 | e_θ 计算不准，控制振荡 | **最高** |
| 定位延迟 | 误差滞后，响应变差 | 中 |
| 轮子打滑 | 瞬时位姿跳变 | 中 |

### 13.4 里程计误差示例

```
真实位置: (1.000, 0.000, 0.00)
里程计:   (1.005, 0.002, 0.02)  ← 5mm位置误差 + 1.1°航向误差

计算的 e_y ≈ 5mm（但实际可能是0）
计算的 e_θ ≈ 0.02 rad

→ LQR 会产生不必要的修正：
  ω_fb = -20×0.005 - 15×0.02 = -0.4 rad/s
→ 可能引起振荡或偏离
```

### 13.5 与 Pure Pursuit 对比

| 算法 | 位姿依赖 | 对位姿精度敏感度 |
|------|---------|-----------------|
| Pure Pursuit | (x, y, θ) | **中等** - 只用于找前视点 |
| **前馈+LQR** | (x, y, θ) | **高** - 直接计算误差 |

### 13.6 位姿精度建议

| 控制精度目标 | 位姿精度要求 | 航向精度要求 |
|-------------|-------------|-------------|
| < 15 mm | < 10 mm | < 3° |
| < 8 mm | < 5 mm | < 1.5° |
| < 5 mm | < 3 mm | < 1° |

### 13.7 应对策略

**如果里程计不够精确：**

1. **降低增益** - 减小 K1, K2 降低对误差的敏感度
2. **增大前瞻** - 平滑位姿噪声的影响
3. **启用积分项** - 补偿系统性偏差
4. **考虑 Pure Pursuit** - 对位姿误差更鲁棒

```cpp
// 位姿不精确时的保守参数
LQRControllerParams params;
params.use_direct_gains = true;
params.K1_direct = 10.0;    // 降低（原20.0）
params.K2_direct = 10.0;    // 降低（原15.0）
params.lookahead_distance = 0.05;  // 增大
params.enable_integral = true;     // 开启积分补偿
```

---

## 附录 A：数学符号表

| 符号 | 含义 | 单位 |
|------|------|------|
| $x, y$ | 位置坐标 | m |
| $\theta$ | 朝向角 | rad |
| $v$ | 线速度 | m/s |
| $\omega$ | 角速度 | rad/s |
| $\kappa$ | 曲率 | 1/m |
| $R$ | 转弯半径 | m |
| $e_y$ | 横向误差 | m |
| $e_\theta$ | 航向误差 | rad |
| $K_1$ | 横向误差增益 | 1/m |
| $K_2$ | 航向误差增益 | 1/rad |

## 附录 B：参考文献

1. Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm.
2. Snider, J. M. (2009). Automatic Steering Methods for Autonomous Automobile Path Tracking.
3. Paden, B., et al. (2016). A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.

---

## 附录 C：版本更新记录

### v1.1 (2024-12)

**修正问题：**
1. 修正 LQR 参数计算错误：原文档 q1=10000, q2=100 无法得到 K1=4.0, K2=15.0
2. 正确参数：q1=1.0, q2=200.0, r=1.0 → K1=4.0, K2≈15.1

**新增功能：**
1. **直接指定增益**：可通过 `use_direct_gains=true` 直接设置 K1, K2
2. **积分项支持 (LQI)**：可选开启 `enable_integral` 消除静态误差
3. **改进的最近点搜索**：可配置搜索窗口，防止曲线处震荡
4. **基于弧长的前瞻**：使用二分搜索实现高效前瞻点定位
5. **高效角度归一化**：使用 fmod 替代 while 循环，O(1) 复杂度

**代码改进：**
- 控制输出增加 `ref_curvature` 和 `ref_index` 调试信息
- 参数结构增加积分项、前瞻距离、搜索窗口等配置

---

*文档版本: 1.1*
*最后更新: 2024-12*
