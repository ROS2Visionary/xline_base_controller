#include "xline_follow_controller/rpp_follow/path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/curve_path_strategy.hpp"
#include "xline_follow_controller/rpp_follow/circle_path_strategy.hpp"

namespace xline
{
namespace follow_controller
{

PathStrategy::UniquePtr createPathStrategy(PathStrategyType type)
{
  switch (type)
  {
    case PathStrategyType::CURVE:
      return std::make_unique<CurvePathStrategy>();

    case PathStrategyType::CIRCLE:
      return std::make_unique<CirclePathStrategy>();

    default:
      return std::make_unique<CurvePathStrategy>();
  }
}

}  // namespace follow_controller
}  // namespace xline
