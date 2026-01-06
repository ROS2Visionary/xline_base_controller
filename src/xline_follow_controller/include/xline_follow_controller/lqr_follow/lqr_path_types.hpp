#pragma once

namespace xline
{
namespace follow_controller
{

/**
 * @brief 路径点结构（包含曲率信息）
 */
struct PathPointWithCurvature
{
  double x;          ///< X坐标 (m)
  double y;          ///< Y坐标 (m)
  double theta;      ///< 航向角 (rad)
  double curvature;  ///< 曲率 (1/m)
  double arc_length; ///< 累积弧长 (m)
};

}  // namespace follow_controller
}  // namespace xline

