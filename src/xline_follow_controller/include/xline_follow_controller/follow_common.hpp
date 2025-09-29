#pragma once
#include <cmath>
#include <deque>
#include <algorithm>
#include <vector>
#include <chrono>
#include <angles/angles.h>
#include <tf2/utils.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Dense>

namespace xline
{
namespace follow_controller
{

class HampelFilter
{
private:
  std::deque<double> window;  // 滑动窗口容器
  int window_size;            // 窗口长度
  double threshold;           // 阈值系数k
  bool use_relative_mad;      // 是否使用相对MAD

public:
  // 构造函数
  HampelFilter(int size = 5, double k = 3.0, bool relative_mad = false)
    : window_size(size), threshold(k), use_relative_mad(relative_mad)
  {
  }

  // 核心滤波函数
  double filter(double new_value)
  {
    // 更新滑动窗口
    window.push_back(new_value);
    if (static_cast<int>(window.size()) > window_size)
    {
      window.pop_front();
    }

    // 窗口未填满时直接返回原始值
    if (static_cast<int>(window.size()) < window_size)
    {
      return new_value;
    }

    // 计算中位数
    double median = compute_median(window);

    // 计算MAD
    std::vector<double> abs_deviations;
    for (auto v : window)
    {
      abs_deviations.push_back(std::abs(v - median));
    }
    double mad = compute_median(abs_deviations);

    // 计算动态阈值（1.4826为高斯分布换算系数）
    double sigma = 1.4826 * mad;
    if (use_relative_mad && median != 0)
    {
      sigma /= std::abs(median);
    }
    double dynamic_threshold = threshold * sigma;

    // 异常值检测与替换
    bool is_outlier = std::abs(new_value - median) > dynamic_threshold;
    double result = is_outlier ? median : new_value;

    if (is_outlier)
    {
      // printf("检测到异常值,替换为中位数");
    }

    return result;
  }

  // 清空滤波器状态
  void reset(int size = 5, double k = 3.0)
  {
    window.clear();
    window_size = size;
    threshold = k;
  }

private:
  // 高效中位数计算
  template <typename T>
  double compute_median(const T& container)
  {
    std::vector<double> temp(container.begin(), container.end());
    size_t n = temp.size() / 2;
    std::nth_element(temp.begin(), temp.begin() + n, temp.end());

    if (temp.size() % 2 == 1)
    {
      return temp[n];
    }
    else
    {
      double a = *std::max_element(temp.begin(), temp.begin() + n);
      std::nth_element(temp.begin(), temp.begin() + n, temp.end());
      return (a + temp[n]) / 2.0;
    }
  }
};

class SavitzkyGolayFilter
{
public:
  // 构造函数，初始化窗口大小和多项式阶数
  SavitzkyGolayFilter(int window_size = 5, int polynomial_order = 2)
    : window_size_(window_size), polynomial_order_(polynomial_order)
  {
    // 检查窗口大小是否为奇数且大于1
    assert(window_size % 2 == 1 && window_size > 1);
    // 检查多项式阶数是否合法
    assert(polynomial_order_ <= window_size_ - 1 && polynomial_order_ >= 0);

    // 计算卷积系数
    computeCoefficients();
  }

  // 重置滤波器的窗口大小和多项式阶数
  void reset(int new_window_size, int new_polynomial_order)
  {
    // 更新参数
    window_size_ = new_window_size;
    polynomial_order_ = new_polynomial_order;

    // 再次计算卷积系数
    computeCoefficients();

    // 清空滑动窗口的数据
    window_data_.clear();
  }

  // 滤波函数（返回当前窗口中心点的平滑值）
  double filter(double new_value)
  {
    // 更新滑动窗口
    if (static_cast<int>(window_data_.size()) >= window_size_)
    {
      window_data_.pop_front();
    }
    window_data_.push_back(new_value);

    // 如果窗口数据还不足，直接返回原始值
    if (static_cast<int>(window_data_.size()) < window_size_)
    {
      return new_value;
    }

    // 计算加权和
    double smoothed_value = 0.0;
    for (int i = 0; i < window_size_; ++i)
    {
      smoothed_value += coefficients_[i] * window_data_[i];
    }

    return smoothed_value;
  }

private:
  int window_size_;                   // 滑动窗口大小
  int polynomial_order_;              // 多项式阶数
  std::deque<double> window_data_;    // 存储滑动窗口数据
  std::vector<double> coefficients_;  // Savitzky-Golay 滤波的卷积系数

  // 计算Savitzky-Golay滤波的卷积系数
  void computeCoefficients()
  {
    int n = (window_size_ - 1) / 2;  // 半窗口大小
    int m = window_size_;
    int k = polynomial_order_ + 1;  // 多项式阶数 + 1

    // 构造设计矩阵A，矩阵A的每一行是 (x^0, x^1, ..., x^polynomial_order)
    std::vector<std::vector<double>> A(m, std::vector<double>(k, 0.0));
    for (int i = 0; i < m; ++i)
    {
      double x = i - n;  // x从 -n 到 n
      for (int j = 0; j < k; ++j)
      {
        A[i][j] = std::pow(x, j);
      }
    }

    // 使用Eigen库来计算矩阵的逆和乘法
    Eigen::MatrixXd mat_A(m, k);
    for (int i = 0; i < m; ++i)
    {
      for (int j = 0; j < k; ++j)
      {
        mat_A(i, j) = A[i][j];
      }
    }

    Eigen::MatrixXd mat_A_transpose = mat_A.transpose();
    Eigen::MatrixXd mat_ATA = mat_A_transpose * mat_A;

    // 计算ATA的逆矩阵
    Eigen::MatrixXd mat_ATA_inv = mat_ATA.inverse();

    // 计算ATA_inv * AT → 系数矩阵
    Eigen::MatrixXd mat_coefficients = mat_ATA_inv * mat_A_transpose;

    // 将第一行系数存储为卷积系数
    coefficients_.resize(m);
    for (int i = 0; i < m; ++i)
    {
      coefficients_[i] = mat_coefficients(0, i);
    }
  }
};

// PID 控制器类定义
class PIDController
{
public:
  // 构造函数，初始化 PID 参数
  PIDController(double kp, double ki, double kd, double max_output, double min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output)
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
    integral_max_ = 0.0;
    integral_min_ = 0.0;
    p_output_ = 0.0;
    i_output_ = 0.0;
    d_output_ = 0.0;
  }

  // 计算控制输出
  double compute(double error, double dt)
  {
    if (dt <= 0.0 || std::isnan(dt) || std::isinf(dt))
    {
      return 0.0;
    }

    integral_ += error * dt;
    integral_ = clamp(integral_, integral_min_, integral_max_);

    // 计算微分项
    double derivative = (error - prev_error_) / dt;

    p_output_ = kp_ * error;
    i_output_ = ki_ * integral_;
    d_output_ = kd_ * derivative;
    double output = p_output_ + i_output_ + d_output_;

    prev_error_ = error;

    double clamped_output = clamp(output, min_output_, max_output_);

    return clamped_output;
  }

  void reset()
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
  }

  // 设置积分项限制
  void setIntegralLimits(double min, double max)
  {
    integral_min_ = min;
    integral_max_ = max;
  }

  void setOutputLimits(double min, double max)
  {
    min_output_ = min;
    max_output_ = max;
  }

  // 设置 PID 参数
  void setGains(double kp, double ki, double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  double kp_;  // 比例系数
  double ki_;  // 积分系数
  double kd_;  // 微分系数
  
  // 获取各项输出
  double getP() const { return p_output_; }
  double getI() const { return i_output_; }
  double getD() const { return d_output_; }

private:
  double prev_error_;    // 上一次误差
  double integral_;      // 积分项累加
  double max_output_;    // 控制输出的最大值
  double min_output_;    // 控制输出的最小值
  double integral_max_;  // 积分项最大值
  double integral_min_;  // 积分项最小值
  double p_output_;      // P项输出
  double i_output_;      // I项输出
  double d_output_;      // D项输出

  // 辅助函数：限制数值范围
  double clamp(double value, double min, double max)
  {
    if (value > max)
      return max;
    if (value < min)
      return min;
    return value;
  }
};

/**
 * @brief 改进的Stanley控制器，用于高精度直线路径跟随
 *
 * 该控制器结合了Stanley方法、自适应参数调整、传感器融合和平滑滤波
 * 专为RTK定位与IMU传感器结合的系统设计，优化航向纠偏性能
 */
class ImprovedHeadingController
{
public:
  /**
   * @brief 构造函数
   *
   * @param k_stanley Stanley增益系数
   * @param stanley_v0 低速稳定因子
   * @param heading_weight 航向误差权重
   * @param cross_track_weight 横向误差权重
   */
  ImprovedHeadingController(double k_stanley = 0.5, double stanley_v0 = 0.1, double heading_weight = 0.7,
                            double cross_track_weight = 0.3)
    : k_stanley_(k_stanley)
    , stanley_v0_(stanley_v0)
    , heading_weight_(heading_weight)
    , cross_track_weight_(cross_track_weight)
    , k_min_(0.3)
    , k_max_(0.8)
    , speed_factor_(0.5)
    , damping_(0.8)
    , natural_freq_(2.0)
    , saturation_threshold_(0.2)
    , imu_heading_weight_(0.7)
    , rtk_heading_weight_(0.3)
    , max_steering_rate_(0.3)
    ,  // rad/s
    smoothing_factor_(0.7)
    , prev_heading_(0.0)
    , prev_angular_output_(0.0)
    , prev_cross_track_error_(0.0)
    , prev_heading_error_(0.0)
  {
    // 初始化滤波数组
    for (int i = 0; i < filter_window_size_; i++)
    {
      heading_error_buffer_.push_back(0.0);
      cross_track_error_buffer_.push_back(0.0);
    }

    // 初始化速度适配参数表
    initializeSpeedAdaptiveParams();

    // 记录创建时间
    last_update_time_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief 重置控制器
   */
  void reset()
  {
    prev_heading_ = 0.0;
    prev_angular_output_ = 0.0;
    prev_cross_track_error_ = 0.0;
    prev_heading_error_ = 0.0;

    // 清空滤波缓冲区
    std::fill(heading_error_buffer_.begin(), heading_error_buffer_.end(), 0.0);
    std::fill(cross_track_error_buffer_.begin(), cross_track_error_buffer_.end(), 0.0);

    // 重置时间
    last_update_time_ = std::chrono::steady_clock::now();
  }

  /**
   * @brief 设置控制参数
   *
   * @param k_stanley Stanley增益系数
   * @param heading_weight 航向误差权重
   * @param cross_track_weight 横向误差权重
   */
  void setParameters(double k_stanley, double heading_weight, double cross_track_weight)
  {
    k_stanley_ = k_stanley;
    heading_weight_ = heading_weight;
    cross_track_weight_ = cross_track_weight;
  }

  /**
   * @brief 设置高级控制参数
   *
   * @param max_steering_rate 最大转向变化率(rad/s)
   * @param smoothing_factor 输出平滑因子(0-1)
   * @param imu_weight IMU数据权重(0-1)
   */
  void setAdvancedParameters(double max_steering_rate, double smoothing_factor, double imu_weight)
  {
    max_steering_rate_ = max_steering_rate;
    smoothing_factor_ = smoothing_factor;
    imu_heading_weight_ = imu_weight;
    rtk_heading_weight_ = 1.0 - imu_weight;
  }

  /**
   * @brief 设置速度自适应参数
   *
   * @param k_min 最小Stanley增益
   * @param k_max 最大Stanley增益
   * @param speed_factor 速度影响因子
   */
  void setAdaptiveParameters(double k_min, double k_max, double speed_factor)
  {
    k_min_ = k_min;
    k_max_ = k_max;
    speed_factor_ = speed_factor;
  }

  /**
   * @brief 主控制函数 - 计算航向控制输出
   *
   * @param cross_track_error 横向误差(m)
   * @param heading_error 航向误差(rad)
   * @param speed 当前速度(m/s)
   * @param imu_angular_rate IMU测量的角速度(rad/s)
   * @return double 角速度控制输出(rad/s)
   */
  double computeSteeringControl(double cross_track_error, double heading_error, double speed, double imu_angular_rate)
  {
    // 计算时间增量
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_update_time_).count();
    last_update_time_ = current_time;

    // 防止dt异常值
    if (dt <= 0.0 || dt > 0.5)
    {
      dt = 0.02;  // 默认50Hz
    }

    // 滤波处理横向误差和航向误差
    double filtered_cte = filterCrossTrackError(cross_track_error);
    double filtered_heading_error = filterHeadingError(heading_error);

    // 融合IMU数据进行航向估计
    double fused_heading_error = fuseHeadingWithIMU(filtered_heading_error, imu_angular_rate, dt);

    // 获取速度自适应参数
    auto adaptive_params = getSpeedAdaptiveParams(speed);
    double adaptive_k_stanley = adaptive_params.k_stanley;
    double adaptive_heading_weight = adaptive_params.heading_weight;
    double adaptive_cross_track_weight = adaptive_params.cross_track_weight;
    double adaptive_smoothing = adaptive_params.smoothing_factor;

    // 根据误差大小调整权重
    adjustWeightsByError(filtered_cte, adaptive_heading_weight, adaptive_cross_track_weight);

    // 应用非线性饱和函数处理大误差
    double saturated_cte = saturatedResponse(filtered_cte, saturation_threshold_);

    // 计算Stanley横向误差项
    double cross_track_term = computeCrossTrackTerm(saturated_cte, speed, adaptive_k_stanley);

    // 计算总的转向角
    double raw_steering_angle =
        adaptive_heading_weight * fused_heading_error + adaptive_cross_track_weight * cross_track_term;

    // 应用二阶响应特性，提供更平滑的响应
    raw_steering_angle = applySecondOrderDynamics(raw_steering_angle);

    // 限制转向角变化率
    double rate_limited_steering =
        limitSteeringRateOfChange(raw_steering_angle, prev_angular_output_, max_steering_rate_, dt);

    // 平滑输出
    double final_output = smoothOutput(rate_limited_steering, prev_angular_output_, adaptive_smoothing);

    // 更新状态
    prev_angular_output_ = final_output;
    prev_cross_track_error_ = filtered_cte;
    prev_heading_error_ = filtered_heading_error;

    return final_output;
  }

  /**
   * @brief 获取控制器诊断信息
   *
   * @return 键值对形式的诊断信息
   */
  std::map<std::string, double> getDiagnostics() const
  {
    std::map<std::string, double> diagnostics;
    diagnostics["stanley_gain"] = k_stanley_;
    diagnostics["heading_weight"] = heading_weight_;
    diagnostics["cross_track_weight"] = cross_track_weight_;
    diagnostics["prev_cross_track_error"] = prev_cross_track_error_;
    diagnostics["prev_heading_error"] = prev_heading_error_;
    diagnostics["prev_angular_output"] = prev_angular_output_;
    return diagnostics;
  }

private:
  // 基本控制参数
  double k_stanley_;           // Stanley增益系数
  double stanley_v0_;          // 低速稳定因子
  double heading_weight_;      // 航向误差权重
  double cross_track_weight_;  // 横向误差权重

  // 自适应参数
  double k_min_;         // 最小增益
  double k_max_;         // 最大增益
  double speed_factor_;  // 速度影响因子

  // 动态响应参数
  double damping_;               // 二阶系统阻尼比
  double natural_freq_;          // 二阶系统自然频率
  double saturation_threshold_;  // 饱和阈值

  // 传感器融合参数
  double imu_heading_weight_;  // IMU航向权重
  double rtk_heading_weight_;  // RTK航向权重

  // 平滑控制参数
  double max_steering_rate_;  // 最大转向变化率(rad/s)
  double smoothing_factor_;   // 输出平滑因子

  // 状态变量
  double prev_heading_;            // 上一时刻航向
  double prev_angular_output_;     // 上一次角速度输出
  double prev_cross_track_error_;  // 上一次横向误差
  double prev_heading_error_;      // 上一次航向误差

  // 时间记录
  std::chrono::steady_clock::time_point last_update_time_;

  // 滤波相关
  static const int filter_window_size_ = 5;
  std::deque<double> heading_error_buffer_;
  std::deque<double> cross_track_error_buffer_;

  // 速度自适应参数表
  struct SpeedAdaptiveParams
  {
    double k_stanley;
    double heading_weight;
    double cross_track_weight;
    double smoothing_factor;
  };
  std::vector<std::pair<double, SpeedAdaptiveParams>> speed_adaptive_params_;

  /**
   * @brief 初始化速度自适应参数表
   */
  void initializeSpeedAdaptiveParams()
  {
    // 低速参数(0.1-0.3 m/s)
    speed_adaptive_params_.push_back({ 0.3, { 0.68, 0.65, 0.35, 0.60 } });

    // 中低速参数(0.3-0.5 m/s)
    speed_adaptive_params_.push_back({ 0.5, { 0.60, 0.70, 0.30, 0.65 } });

    // 中速参数(0.5-0.7 m/s)
    speed_adaptive_params_.push_back({ 0.7, { 0.50, 0.75, 0.25, 0.70 } });

    // 中高速参数(0.7-0.9 m/s)
    speed_adaptive_params_.push_back({ 0.9, { 0.40, 0.80, 0.20, 0.75 } });

    // 高速参数(>0.9 m/s)
    speed_adaptive_params_.push_back({ 999.0, { 0.32, 0.85, 0.15, 0.80 } });
  }

  /**
   * @brief 获取指定速度对应的自适应参数
   *
   * @param speed 当前速度
   * @return 自适应参数集
   */
  SpeedAdaptiveParams getSpeedAdaptiveParams(double speed)
  {
    for (size_t i = 0; i < speed_adaptive_params_.size(); ++i)
    {
      if (speed < speed_adaptive_params_[i].first)
      {
        // 找到第一个大于当前速度的区间
        if (i == 0)
        {
          return speed_adaptive_params_[0].second;
        }
        else
        {
          // 在两个区间之间进行线性插值
          double lower_speed = (i == 1) ? 0.1 : speed_adaptive_params_[i - 2].first;
          double upper_speed = speed_adaptive_params_[i - 1].first;
          double ratio = (speed - lower_speed) / (upper_speed - lower_speed);

          // 线性插值计算参数
          SpeedAdaptiveParams result;
          auto& lower_params = speed_adaptive_params_[i - 1].second;
          auto& upper_params = speed_adaptive_params_[i].second;

          result.k_stanley = lower_params.k_stanley + ratio * (upper_params.k_stanley - lower_params.k_stanley);
          result.heading_weight =
              lower_params.heading_weight + ratio * (upper_params.heading_weight - lower_params.heading_weight);
          result.cross_track_weight = lower_params.cross_track_weight +
                                      ratio * (upper_params.cross_track_weight - lower_params.cross_track_weight);
          result.smoothing_factor =
              lower_params.smoothing_factor + ratio * (upper_params.smoothing_factor - lower_params.smoothing_factor);

          return result;
        }
      }
    }

    // 如果速度超过所有区间，返回最高速区间参数
    return speed_adaptive_params_.back().second;
  }

  /**
   * @brief 基于横向误差大小调整权重
   *
   * @param cross_track_error 横向误差
   * @param heading_weight 航向权重(引用)
   * @param cte_weight 横向误差权重(引用)
   */
  void adjustWeightsByError(double cross_track_error, double& heading_weight, double& cte_weight)
  {
    // 误差很大时，更关注横向误差纠正
    // 误差小时，更关注航向保持
    double error_threshold = 0.05;  // 5cm阈值

    if (std::abs(cross_track_error) > error_threshold)
    {
      // 误差较大，增加横向误差权重
      double factor = std::min(std::abs(cross_track_error) / 0.1, 1.0);
      heading_weight = heading_weight * (1.0 - factor * 0.3);  // 降低20%-30%
      cte_weight = 1.0 - heading_weight;
    }
  }

  /**
   * @brief 使用饱和函数处理大误差
   *
   * @param error 原始误差
   * @param threshold 饱和阈值
   * @return 处理后的误差
   */
  double saturatedResponse(double error, double threshold)
  {
    if (std::abs(error) <= threshold)
    {
      return error;
    }
    else
    {
      double sign = (error > 0) ? 1.0 : -1.0;
      return sign * (threshold + std::tanh((std::abs(error) - threshold) * 0.5));
    }
  }

  /**
   * @brief 计算Stanley横向误差项
   *
   * @param cross_track_error 横向误差
   * @param speed 当前速度
   * @param gain Stanley增益
   * @return 横向误差控制项
   */
  double computeCrossTrackTerm(double cross_track_error, double speed, double gain)
  {
    // 防止低速时分母接近零
    double adjusted_speed = speed + stanley_v0_;

    // 计算横向误差控制项
    double cross_track_term = std::atan2(gain * cross_track_error, adjusted_speed);

    return cross_track_term;
  }

  /**
   * @brief 将IMU角速度数据融合到航向估计中
   *
   * @param rtk_heading_error RTK提供的航向误差
   * @param imu_angular_rate IMU测量的角速度
   * @param dt 时间增量
   * @return 融合后的航向误差
   */
  double fuseHeadingWithIMU(double rtk_heading_error, double imu_angular_rate, double dt)
  {
    // 使用IMU角速度积分估计航向变化
    double imu_heading_change = imu_angular_rate * dt;
    double imu_heading_error = prev_heading_error_ + imu_heading_change;

    // 融合IMU和RTK数据
    double fused_heading_error = imu_heading_weight_ * imu_heading_error + rtk_heading_weight_ * rtk_heading_error;

    // 规范化到[-π, π]
    fused_heading_error = angles::normalize_angle(fused_heading_error);

    return fused_heading_error;
  }

  /**
   * @brief 应用二阶动态响应特性
   *
   * @param error 误差值
   * @return 处理后的响应
   */
  double applySecondOrderDynamics(double error)
  {
    // 二阶系统响应公式
    double response =
        natural_freq_ * natural_freq_ * error /
        (1.0 + 2.0 * damping_ * natural_freq_ * std::abs(error) + natural_freq_ * natural_freq_ * error * error);

    return response;
  }

  /**
   * @brief 限制转向角变化率
   *
   * @param new_steering 新计算的转向角
   * @param prev_steering 上一次的转向角
   * @param max_rate 最大变化率
   * @param dt 时间增量
   * @return 限制后的转向角
   */
  double limitSteeringRateOfChange(double new_steering, double prev_steering, double max_rate, double dt)
  {
    double max_change = max_rate * dt;
    double change = new_steering - prev_steering;

    if (change > max_change)
    {
      return prev_steering + max_change;
    }
    else if (change < -max_change)
    {
      return prev_steering - max_change;
    }
    else
    {
      return new_steering;
    }
  }

  /**
   * @brief 平滑控制输出
   *
   * @param new_value 新计算的输出值
   * @param prev_value 上一次的输出值
   * @param alpha 平滑因子(0-1)
   * @return 平滑后的输出值
   */
  double smoothOutput(double new_value, double prev_value, double alpha)
  {
    return alpha * new_value + (1.0 - alpha) * prev_value;
  }

  /**
   * @brief 中值滤波处理横向误差
   *
   * @param new_error 新的横向误差
   * @return 滤波后的横向误差
   */
  double filterCrossTrackError(double new_error)
  {
    // 添加到滤波器缓冲区
    cross_track_error_buffer_.push_back(new_error);
    if (cross_track_error_buffer_.size() > filter_window_size_)
    {
      cross_track_error_buffer_.pop_front();
    }

    // 复制到向量中进行排序
    std::vector<double> sorted_errors(cross_track_error_buffer_.begin(), cross_track_error_buffer_.end());
    std::sort(sorted_errors.begin(), sorted_errors.end());

    // 返回中值
    size_t mid_idx = sorted_errors.size() / 2;
    return sorted_errors[mid_idx];
  }

  /**
   * @brief 均值滤波处理航向误差
   *
   * @param new_error 新的航向误差
   * @return 滤波后的航向误差
   */
  double filterHeadingError(double new_error)
  {
    // 添加到滤波器缓冲区
    heading_error_buffer_.push_back(new_error);
    if (heading_error_buffer_.size() > filter_window_size_)
    {
      heading_error_buffer_.pop_front();
    }

    // 计算加权平均值，新值权重更高
    double sum = 0.0;
    double weight_sum = 0.0;
    double weight_factor = 1.2;  // 每个新值的权重增加因子

    for (size_t i = 0; i < heading_error_buffer_.size(); ++i)
    {
      double weight = std::pow(weight_factor, i);
      sum += heading_error_buffer_[heading_error_buffer_.size() - 1 - i] * weight;
      weight_sum += weight;
    }

    return sum / weight_sum;
  }
};

class SecondOrderSmoother
{
public:
  SecondOrderSmoother(double natural_freq = 1.0, double damping_ratio = 0.7)
    : wn_(natural_freq), zeta_(damping_ratio), initialized_(false)
  {
    reset();
  }

  void setParameters(double natural_freq, double damping_ratio)
  {
    wn_ = natural_freq;
    zeta_ = damping_ratio;
  }

  double filter(double input, double dt)
  {
    if (!initialized_)
    {
      x1_ = input;
      x2_ = 0.0;
      initialized_ = true;
      return input;
    }

    dt = std::max(0.001, std::min(0.1, dt));

    double x2_dot = wn_ * wn_ * (input - x1_) - 2.0 * zeta_ * wn_ * x2_;
    double x1_dot = x2_;

    x2_ += x2_dot * dt;
    x1_ += x1_dot * dt;

    return x1_;
  }

  void reset()
  {
    x1_ = 0.0;
    x2_ = 0.0;
    initialized_ = false;
  }

  double getCurrentValue() const
  {
    return x1_;
  }
  double getCurrentDerivative() const
  {
    return x2_;
  }

private:
  double wn_;    // 自然频率
  double zeta_;  // 阻尼比
  double x1_;    // 输出值
  double x2_;    // 输出值的导数
  bool initialized_;
};

class FourthOrderLowpassFilter
{
public:
  bool use_biquad_cascade_;

protected:
  // 直接型实现的系数
  double a0_, a1_, a2_, a3_, a4_;  // 分母系数
  double b0_, b1_, b2_, b3_, b4_;  // 分子系数
  double x_[5];                    // 输入历史
  double y_[5];                    // 输出历史

  // 双二阶级联实现
  struct Biquad
  {
    double b0, b1, b2;  // 分子系数
    double a1, a2;      // 分母系数 (a0 = 1)
    double x1, x2;      // 输入延迟
    double y1, y2;      // 输出延迟

    void reset()
    {
      x1 = x2 = y1 = y2 = 0.0;
    }

    double process(double input)
    {
      double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
      x2 = x1;
      x1 = input;
      y2 = y1;
      y1 = output;
      return output;
    }
  };

  Biquad biquad1_, biquad2_;

  // 稳定性参数
  double output_limit_;     // 输出限幅
  double rate_limit_;       // 变化率限制
  double previous_output_;  // 上一次的输出
  bool use_rate_limit_;     // 是否使用变化率限制
  
  // 偏移量限制参数
  double output_offset_;    // 输出偏移量
  bool use_offset_limit_;   // 是否使用偏移量限制

  // 数值稳定性检查
  static constexpr double EPSILON = 1e-10;
  static constexpr double MAX_COEFF = 1e6;

public:
  FourthOrderLowpassFilter()
    : use_biquad_cascade_(false)
    , output_limit_(1000.0)
    , rate_limit_(100.0)
    , previous_output_(0.0)
    , use_rate_limit_(true)
    , output_offset_(0.0)
    , use_offset_limit_(false)
  {
    reset();
  }

  // 主初始化函数
  void initialize(double cutoff_freq, double sample_rate, double output_limit = 1000.0)
  {
    output_limit_ = output_limit;

    // 参数合理性检查
    if (cutoff_freq <= 0 || sample_rate <= 0)
    {
      cutoff_freq = 2.0;
      sample_rate = 20.0;
    }

    // 确保截止频率不超过奈奎斯特频率的45%
    double max_cutoff = 0.45 * sample_rate / 2.0;
    if (cutoff_freq > max_cutoff)
    {
      cutoff_freq = max_cutoff;
    }

    // 计算归一化频率
    double normalized_freq = cutoff_freq / sample_rate;

    // 根据归一化频率选择实现方式
    if (normalized_freq < 0.15)
    {
      // 低频时使用双二阶级联（数值更稳定）
      initializeRealBiquadCascade(cutoff_freq, sample_rate);
      use_biquad_cascade_ = true;
    }
    else
    {
      // 高频时使用直接型实现
      initializeDirectForm(cutoff_freq, sample_rate);
      use_biquad_cascade_ = false;
    }
  }

  // 直接型实现
  void initializeDirectForm(double cutoff_freq, double sample_rate)
  {
    double omega = 2.0 * M_PI * cutoff_freq / sample_rate;
    double omega_c = tan(omega / 2.0);

    // 防止数值溢出
    omega_c = std::min(omega_c, 10.0);

    double omega_c2 = omega_c * omega_c;
    double omega_c3 = omega_c2 * omega_c;
    double omega_c4 = omega_c2 * omega_c2;

    // 巴特沃斯4阶滤波器系数
    double sqrt2 = sqrt(2.0);
    double k1 = sqrt2;
    double k2 = 1.0;

    // 计算滤波器系数
    double denominator = 1 + 2 * k1 * omega_c + 2 * k2 * omega_c2 + 2 * k1 * omega_c3 + omega_c4;

    if (std::abs(denominator) < EPSILON)
    {
      denominator = EPSILON;
    }

    // 归一化的分子系数
    b0_ = omega_c4 / denominator;
    b1_ = 4 * omega_c4 / denominator;
    b2_ = 6 * omega_c4 / denominator;
    b3_ = 4 * omega_c4 / denominator;
    b4_ = omega_c4 / denominator;

    // 归一化的分母系数
    a0_ = 1.0;
    a1_ = (4 * omega_c4 - 4 * k1 * omega_c3 - 8 * k2 * omega_c2 - 8 * k1 * omega_c - 4) / denominator;
    a2_ = (6 * omega_c4 - 8 * k2 * omega_c2 + 6) / denominator;
    a3_ = (4 * omega_c4 + 4 * k1 * omega_c3 - 8 * k2 * omega_c2 + 8 * k1 * omega_c - 4) / denominator;
    a4_ = (omega_c4 - 2 * k1 * omega_c3 + 2 * k2 * omega_c2 - 2 * k1 * omega_c + 1) / denominator;

    limitCoefficients();
  }

  // 双二阶级联实现
  void initializeRealBiquadCascade(double cutoff_freq, double sample_rate)
  {
    double omega = 2.0 * M_PI * cutoff_freq / sample_rate;
    double sin_omega = sin(omega);
    double cos_omega = cos(omega);

    // 巴特沃斯4阶 = 两个2阶级联
    // 第一级: Q = 0.54119610
    // 第二级: Q = 1.30656296

    // 第一个双二阶滤波器
    double Q1 = 0.54119610;
    double alpha1 = sin_omega / (2.0 * Q1);
    double a0_1 = 1.0 + alpha1;

    biquad1_.b0 = (1.0 - cos_omega) / 2.0 / a0_1;
    biquad1_.b1 = (1.0 - cos_omega) / a0_1;
    biquad1_.b2 = (1.0 - cos_omega) / 2.0 / a0_1;
    biquad1_.a1 = -2.0 * cos_omega / a0_1;
    biquad1_.a2 = (1.0 - alpha1) / a0_1;

    // 第二个双二阶滤波器
    double Q2 = 1.30656296;
    double alpha2 = sin_omega / (2.0 * Q2);
    double a0_2 = 1.0 + alpha2;

    biquad2_.b0 = (1.0 - cos_omega) / 2.0 / a0_2;
    biquad2_.b1 = (1.0 - cos_omega) / a0_2;
    biquad2_.b2 = (1.0 - cos_omega) / 2.0 / a0_2;
    biquad2_.a1 = -2.0 * cos_omega / a0_2;
    biquad2_.a2 = (1.0 - alpha2) / a0_2;

    // 重置状态
    biquad1_.reset();
    biquad2_.reset();
  }

  // 简化的初始化方法（级联一阶）
  void initializeSimple(double gain)
  {
    gain = std::max(0.01, std::min(0.5, gain));

    double alpha = gain;
    double beta = 1 - alpha;
    double beta2 = beta * beta;
    double beta3 = beta2 * beta;
    double beta4 = beta2 * beta2;
    double alpha4 = alpha * alpha * alpha * alpha;

    b0_ = alpha4;
    b1_ = 4 * alpha4;
    b2_ = 6 * alpha4;
    b3_ = 4 * alpha4;
    b4_ = alpha4;

    a0_ = 1.0;
    a1_ = -4 * beta;
    a2_ = 6 * beta2;
    a3_ = -4 * beta3;
    a4_ = beta4;

    limitCoefficients();
    use_biquad_cascade_ = false;
  }

  // 重置滤波器状态
  void reset()
  {
    for (int i = 0; i < 5; ++i)
    {
      x_[i] = 0.0;
      y_[i] = 0.0;
    }
    biquad1_.reset();
    biquad2_.reset();
    previous_output_ = 0.0;
  }

  // 设置限制参数（原有方法）
  void setLimits(double output_limit, double rate_limit, bool use_rate_limit = true)
  {
    output_limit_ = output_limit;
    rate_limit_ = rate_limit;
    use_rate_limit_ = use_rate_limit;
    use_offset_limit_ = false;  // 使用此方法时禁用偏移量限制
  }

  // 设置带偏移量的限制参数
  void setLimitsWithOffset(double output_limit, double output_offset, double rate_limit, bool use_rate_limit = true)
  {
    output_limit_ = output_limit;
    output_offset_ = output_offset;
    rate_limit_ = rate_limit;
    use_rate_limit_ = use_rate_limit;
    use_offset_limit_ = true;
  }

  // 单独设置偏移量限制
  void setOffsetLimit(double offset, bool enable = true)
  {
    output_offset_ = offset;
    use_offset_limit_ = enable;
  }

  // 获取当前的限制范围
  void getCurrentLimits(double& lower_limit, double& upper_limit) const
  {
    if (use_offset_limit_)
    {
      lower_limit = output_limit_ - output_offset_;
      upper_limit = output_limit_ + output_offset_;
    }
    else
    {
      lower_limit = -output_limit_;
      upper_limit = output_limit_;
    }
  }

  // 主滤波函数
  double filter(double input)
  {
    // 输入限幅和检查
    input = saturate(input, output_limit_ * 2.0);
    if (std::isnan(input) || std::isinf(input))
    {
      return previous_output_;
    }

    double output;

    if (use_biquad_cascade_)
    {
      // 使用双二阶级联
      output = filterBiquadCascade(input);
    }
    else
    {
      // 使用直接型实现
      output = filterDirectForm(input);
    }

    // 应用变化率限制
    if (use_rate_limit_)
    {
      double change = output - previous_output_;
      if (std::abs(change) > rate_limit_)
      {
        output = previous_output_ + std::copysign(rate_limit_, change);
      }
    }

    previous_output_ = output;
    return output;
  }

  // 零相位滤波（适用于离线处理）
  std::vector<double> filterZeroPhase(const std::vector<double>& input)
  {
    if (input.empty())
      return {};

    std::vector<double> forward(input.size());
    std::vector<double> result(input.size());

    // 正向滤波
    reset();
    for (size_t i = 0; i < input.size(); ++i)
    {
      forward[i] = filter(input[i]);
    }

    // 反向滤波
    reset();
    for (int i = static_cast<int>(forward.size()) - 1; i >= 0; --i)
    {
      result[i] = filter(forward[i]);
    }

    // 反转结果（因为是反向处理的）
    std::reverse(result.begin(), result.end());

    return result;
  }

  // 获取当前输出
  double getCurrentOutput() const
  {
    return previous_output_;
  }

  // 检查滤波器稳定性
  virtual bool checkStability() const
  {
    if (use_biquad_cascade_)
    {
      // 检查双二阶滤波器的稳定性
      return checkBiquadStability(biquad1_) && checkBiquadStability(biquad2_);
    }
    else
    {
      // 检查直接型的稳定性
      return checkDirectFormStability() && checkPoles();
    }
  }

protected:
  // 直接型滤波实现
  double filterDirectForm(double input)
  {
    // 移动历史数据
    for (int i = 4; i > 0; --i)
    {
      x_[i] = x_[i - 1];
      y_[i] = y_[i - 1];
    }
    x_[0] = input;

    // 计算输出
    double feedforward = b0_ * x_[0] + b1_ * x_[1] + b2_ * x_[2] + b3_ * x_[3] + b4_ * x_[4];
    double feedback = a1_ * y_[1] + a2_ * y_[2] + a3_ * y_[3] + a4_ * y_[4];

    // 使用带偏移量的饱和函数
    feedback = saturateWithOffset(feedback, output_limit_ * 10.0);

    y_[0] = feedforward - feedback;
    y_[0] = saturateWithOffset(y_[0], output_limit_);

    if (std::isnan(y_[0]) || std::isinf(y_[0]))
    {
      reset();
      y_[0] = 0.0;
    }

    return y_[0];
  }

  // 双二阶级联滤波实现
  double filterBiquadCascade(double input)
  {
    // 第一级处理
    double stage1_out = biquad1_.process(input);
    stage1_out = saturateWithOffset(stage1_out, output_limit_);

    // 第二级处理
    double output = biquad2_.process(stage1_out);
    output = saturateWithOffset(output, output_limit_);

    if (std::isnan(output) || std::isinf(output))
    {
      biquad1_.reset();
      biquad2_.reset();
      output = 0.0;
    }

    return output;
  }

  // 原有的饱和函数（对称限制）
  double saturate(double value, double limit) const
  {
    return std::max(-limit, std::min(limit, value));
  }

  // 带偏移量的饱和函数
  double saturateWithOffset(double value, double limit) const
  {
    if (use_offset_limit_)
    {
      // 非对称限制：[limit - offset, limit + offset]
      double lower_limit = limit - output_offset_;
      double upper_limit = limit + output_offset_;
      return std::max(lower_limit, std::min(upper_limit, value));
    }
    else
    {
      // 对称限制：[-limit, limit]
      return saturate(value, limit);
    }
  }

  // 限制系数大小
  void limitCoefficients()
  {
    a1_ = saturate(a1_, MAX_COEFF);
    a2_ = saturate(a2_, MAX_COEFF);
    a3_ = saturate(a3_, MAX_COEFF);
    a4_ = saturate(a4_, MAX_COEFF);

    double max_b = std::max({ std::abs(b0_), std::abs(b1_), std::abs(b2_), std::abs(b3_), std::abs(b4_) });
    if (max_b > MAX_COEFF)
    {
      double scale = MAX_COEFF / max_b;
      b0_ *= scale;
      b1_ *= scale;
      b2_ *= scale;
      b3_ *= scale;
      b4_ *= scale;
    }
  }

  // 检查直接型稳定性
  bool checkDirectFormStability() const
  {
    if (std::abs(a1_) > 10.0 || std::abs(a2_) > 20.0 || std::abs(a3_) > 20.0 || std::abs(a4_) > 10.0)
    {
      return false;
    }

    double dc_gain = (b0_ + b1_ + b2_ + b3_ + b4_) / (1.0 + a1_ + a2_ + a3_ + a4_);
    if (std::abs(dc_gain) > 10.0 || std::abs(dc_gain) < 0.001)
    {
      return false;
    }

    return true;
  }

  // 检查双二阶稳定性
  bool checkBiquadStability(const Biquad& bq) const
  {
    // 检查极点是否在单位圆内
    // 特征方程: z^2 + a1*z + a2 = 0
    double discriminant = bq.a1 * bq.a1 - 4 * bq.a2;

    if (discriminant >= 0)
    {
      // 实极点
      double sqrt_disc = sqrt(discriminant);
      double pole1 = (-bq.a1 + sqrt_disc) / 2.0;
      double pole2 = (-bq.a1 - sqrt_disc) / 2.0;
      return std::abs(pole1) < 1.0 && std::abs(pole2) < 1.0;
    }
    else
    {
      // 复极点
      double real_part = -bq.a1 / 2.0;
      double imag_part = sqrt(-discriminant) / 2.0;
      double magnitude = sqrt(real_part * real_part + imag_part * imag_part);
      return magnitude < 1.0;
    }
  }

  // 检查四阶系统的极点
  bool checkPoles() const
  {
    // 对于四阶系统，完整的极点分析较复杂
    // 这里使用简化的稳定性判据

    // Jury稳定性判据的简化版本
    double sum_a = 1.0 + a1_ + a2_ + a3_ + a4_;
    double alt_sum_a = 1.0 - a1_ + a2_ - a3_ + a4_;

    // 必要条件
    if (sum_a <= 0 || alt_sum_a <= 0)
    {
      return false;
    }

    // 检查|a4| < 1（所有极点在单位圆内的必要条件）
    if (std::abs(a4_) >= 1.0)
    {
      return false;
    }

    return true;
  }
};

}  // namespace follow_controller
}  // namespace xline
