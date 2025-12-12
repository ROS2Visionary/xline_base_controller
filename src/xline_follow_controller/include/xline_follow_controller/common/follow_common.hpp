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

/**
 * @brief 异常值处理策略枚举
 */
enum class OutlierStrategy
{
  REPLACE_MEDIAN,      // 直接用中位数替换（原始方式）
  SOFT_THRESHOLD,      // 软阈值混合，平滑过渡
  LINEAR_PREDICTION,   // 线性预测，保持运动连续性
  EXPONENTIAL_DECAY    // 指数衰减，渐进修正
};

/**
 * @brief 改进的Hampel滤波器
 * 
 * 基于MAD（中位数绝对偏差）的异常值检测与处理滤波器。
 * 相比原始实现，增加了多种异常值处理策略，特别是针对位置/速度跟踪
 * 场景优化的线性预测策略，能够保持运动连续性。
 */
class HampelFilter
{
public:
  /**
   * @brief 构造函数
   * @param size 滑动窗口大小
   * @param k 阈值系数（MAD的倍数）
   * @param relative_mad 是否使用相对MAD
   * @param strategy 异常值处理策略，默认使用线性预测
   */
  HampelFilter(int size = 5, double k = 3.0, bool relative_mad = false,
               OutlierStrategy strategy = OutlierStrategy::LINEAR_PREDICTION)
    : window_size_(size)
    , threshold_(k)
    , use_relative_mad_(relative_mad)
    , strategy_(strategy)
    , prev_output_(0.0)
    , initialized_(false)
    , consecutive_outliers_(0)
    , max_consecutive_outliers_(3)
    , prediction_confidence_(1.0)
    , min_confidence_(0.3)
    , confidence_decay_(0.7)
  {
  }

  /**
   * @brief 核心滤波函数
   * @param new_value 新输入值
   * @return 滤波后的值
   */
  double filter(double new_value)
  {
    // 更新滑动窗口
    window_.push_back(new_value);
    if (static_cast<int>(window_.size()) > window_size_)
    {
      window_.pop_front();
    }

    // 窗口未填满时直接返回原始值
    if (static_cast<int>(window_.size()) < window_size_)
    {
      updateValidHistory(new_value);
      prev_output_ = new_value;
      initialized_ = true;
      return new_value;
    }

    // 计算中位数
    double median = computeMedian(window_);

    // 计算MAD（中位数绝对偏差）
    std::vector<double> abs_deviations;
    abs_deviations.reserve(window_.size());
    for (const auto& v : window_)
    {
      abs_deviations.push_back(std::abs(v - median));
    }
    double mad = computeMedian(abs_deviations);

    // 计算动态阈值（1.4826为高斯分布换算系数）
    double sigma = 1.4826 * mad;
    if (use_relative_mad_ && std::abs(median) > 1e-9)
    {
      sigma /= std::abs(median);
    }
    
    // 防止sigma过小导致误判
    sigma = std::max(sigma, 1e-6);
    double dynamic_threshold = threshold_ * sigma;

    // 异常值检测
    double deviation = std::abs(new_value - median);
    bool is_outlier = deviation > dynamic_threshold;

    double result;
    if (!is_outlier)
    {
      // 正常值处理
      result = new_value;
      updateValidHistory(new_value);
      consecutive_outliers_ = 0;
      prediction_confidence_ = 1.0;
    }
    else
    {
      // 异常值处理
      consecutive_outliers_++;
      result = handleOutlier(new_value, median, dynamic_threshold, deviation);
      
      // 更新预测置信度
      prediction_confidence_ *= confidence_decay_;
      prediction_confidence_ = std::max(prediction_confidence_, min_confidence_);

      RCLCPP_DEBUG(rclcpp::get_logger("HampelFilter"), 
                   "检测到异常值: %.4f, 中位数: %.4f, 阈值: %.4f, 输出: %.4f",
                   new_value, median, dynamic_threshold, result);
    }

    prev_output_ = result;
    initialized_ = true;
    return result;
  }

  /**
   * @brief 重置滤波器状态
   * @param size 新的窗口大小（可选）
   * @param k 新的阈值系数（可选）
   */
  void reset(int size = -1, double k = -1.0)
  {
    if (size > 0)
    {
      window_size_ = size;
    }
    if (k > 0)
    {
      threshold_ = k;
    }
    
    window_.clear();
    valid_history_.clear();
    prev_output_ = 0.0;
    initialized_ = false;
    consecutive_outliers_ = 0;
    prediction_confidence_ = 1.0;
  }

  /**
   * @brief 设置异常值处理策略
   * @param strategy 策略类型
   */
  void setStrategy(OutlierStrategy strategy)
  {
    strategy_ = strategy;
  }

  /**
   * @brief 设置预测相关参数
   * @param max_consecutive 连续异常值最大数量，超过后降级到中位数
   * @param min_conf 最小置信度
   * @param decay 置信度衰减系数
   */
  void setPredictionParams(int max_consecutive, double min_conf, double decay)
  {
    max_consecutive_outliers_ = max_consecutive;
    min_confidence_ = std::clamp(min_conf, 0.0, 1.0);
    confidence_decay_ = std::clamp(decay, 0.0, 1.0);
  }

  /**
   * @brief 获取当前预测置信度
   * @return 置信度值 [0, 1]
   */
  double getPredictionConfidence() const
  {
    return prediction_confidence_;
  }

  /**
   * @brief 获取连续异常值计数
   * @return 连续异常值数量
   */
  int getConsecutiveOutliers() const
  {
    return consecutive_outliers_;
  }

private:
  // 基本参数
  std::deque<double> window_;           // 滑动窗口容器
  std::deque<double> valid_history_;    // 有效值历史，用于预测
  int window_size_;                     // 窗口长度
  double threshold_;                    // 阈值系数k
  bool use_relative_mad_;               // 是否使用相对MAD
  OutlierStrategy strategy_;            // 异常值处理策略

  // 状态变量
  double prev_output_;                  // 上一次输出值
  bool initialized_;                    // 是否已初始化

  // 预测相关参数
  int consecutive_outliers_;            // 连续异常值计数
  int max_consecutive_outliers_;        // 连续异常值最大数量
  double prediction_confidence_;        // 预测置信度
  double min_confidence_;               // 最小置信度
  double confidence_decay_;             // 置信度衰减系数

  // 有效历史窗口大小
  static constexpr int kValidHistorySize = 5;

  /**
   * @brief 计算容器中值的中位数
   */
  template <typename Container>
  double computeMedian(const Container& container) const
  {
    if (container.empty())
    {
      return 0.0;
    }

    std::vector<double> temp(container.begin(), container.end());
    size_t n = temp.size() / 2;
    std::nth_element(temp.begin(), temp.begin() + n, temp.end());

    if (temp.size() % 2 == 1)
    {
      return temp[n];
    }
    else
    {
      double upper = temp[n];
      double lower = *std::max_element(temp.begin(), temp.begin() + n);
      return (lower + upper) / 2.0;
    }
  }

  /**
   * @brief 更新有效值历史
   */
  void updateValidHistory(double value)
  {
    valid_history_.push_back(value);
    if (static_cast<int>(valid_history_.size()) > kValidHistorySize)
    {
      valid_history_.pop_front();
    }
  }

  /**
   * @brief 根据策略处理异常值
   */
  double handleOutlier(double new_value, double median, 
                       double threshold, double deviation)
  {
    // 连续异常值过多时，降级到中位数替换
    if (consecutive_outliers_ >= max_consecutive_outliers_)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("HampelFilter"), 
                   "连续异常值过多，降级到中位数替换");
      return median;
    }

    switch (strategy_)
    {
      case OutlierStrategy::SOFT_THRESHOLD:
        return handleSoftThreshold(new_value, median, threshold, deviation);

      case OutlierStrategy::LINEAR_PREDICTION:
        return handleLinearPrediction(new_value, median, threshold);

      case OutlierStrategy::EXPONENTIAL_DECAY:
        return handleExponentialDecay(median, threshold);

      case OutlierStrategy::REPLACE_MEDIAN:
      default:
        return median;
    }
  }

  /**
   * @brief 软阈值混合处理
   * 根据偏离程度进行加权混合，偏离越大越接近中位数
   */
  double handleSoftThreshold(double new_value, double median,
                             double threshold, double deviation)
  {
    double excess = deviation - threshold;
    // 使用sigmoid函数实现平滑过渡
    double blend_factor = 1.0 / (1.0 + excess / threshold);
    return blend_factor * new_value + (1.0 - blend_factor) * median;
  }

  /**
   * @brief 线性预测处理（推荐用于位置/速度跟踪）
   * 基于有效历史值的趋势进行线性外推预测
   */
  double handleLinearPrediction(double new_value, double median, double threshold)
  {
    size_t history_size = valid_history_.size();

    // 历史数据不足时回退到软阈值
    if (history_size < 2)
    {
      return median;
    }

    // 计算加权线性趋势（近期数据权重更高）
    double weighted_trend = 0.0;
    double weight_sum = 0.0;
    
    for (size_t i = 1; i < history_size; ++i)
    {
      double local_trend = valid_history_[i] - valid_history_[i - 1];
      double weight = static_cast<double>(i);  // 越新的权重越大
      weighted_trend += local_trend * weight;
      weight_sum += weight;
    }
    
    double trend = (weight_sum > 0) ? weighted_trend / weight_sum : 0.0;

    // 基于最后一个有效值和趋势进行预测
    double predicted = valid_history_.back() + trend;

    // 使用置信度混合预测值和中位数
    predicted = prediction_confidence_ * predicted + 
                (1.0 - prediction_confidence_) * median;

    // 限制预测值在合理范围内（中位数附近的阈值范围）
    double max_deviation = threshold * 1.5;
    double lower_bound = median - max_deviation;
    double upper_bound = median + max_deviation;
    predicted = std::clamp(predicted, lower_bound, upper_bound);

    // 额外检查：如果预测值仍然是异常值方向，进行修正
    // 确保预测值不会比原始异常值更偏离
    if (new_value > median && predicted > new_value)
    {
      predicted = std::min(predicted, new_value);
    }
    else if (new_value < median && predicted < new_value)
    {
      predicted = std::max(predicted, new_value);
    }

    return predicted;
  }

  /**
   * @brief 指数衰减处理
   * 从上一个输出值向中位数方向渐进修正
   */
  double handleExponentialDecay(double median, double threshold)
  {
    if (!initialized_)
    {
      return median;
    }

    // 计算修正步长（与阈值相关）
    double max_step = threshold * 0.5;
    double diff = median - prev_output_;

    // 如果差值小于最大步长，直接返回中位数
    if (std::abs(diff) <= max_step)
    {
      return median;
    }

    // 否则向中位数方向移动一步
    return prev_output_ + std::copysign(max_step, diff);
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
