#include "common/lane/lane_generator.h"
#include "common/basics/config.h"
#include "common/spline/spline_generator.h"

namespace common {

ErrorType LaneGenerator::GetLaneBySampleInterpolation(
    const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
    Lane* lane) {
  // 如果给定参数，直接使用 (插值)
  Spline<LaneDegree, LaneDim> spline;
  SplineGenerator<LaneDegree, LaneDim> spline_generator;

  if (spline_generator.GetCubicSplineBySampleInterpolation(
          samples, para, &spline) != kSuccess) {
    // printf("[LaneBySampleInterpolation]Cannot get lane by interpolation.\n");
    return kWrongStatus;
  }
  lane->set_position_spline(spline);
  if (!lane->IsValid()) return kWrongStatus;
  return kSuccess;
}

ErrorType LaneGenerator::GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                               Lane* lane) {
  // 如果不给定参数，采用采样点欧氏距离代替 (插值)
  std::vector<decimal_t> para;
  double d = 0;
  para.push_back(d);
  for (int i = 1; i < (int)samples.size(); ++i) {
    double dx = samples[i](0) - samples[i - 1](0);
    double dy = samples[i](1) - samples[i - 1](1);
    d += std::hypot(dx, dy);
    para.push_back(d);
  }
  if (common::LaneGenerator::GetLaneBySampleInterpolation(samples, para,
                                                          lane) != kSuccess) {
    // printf("Cannot get lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType LaneGenerator::GetLaneBySampleFitting(
    const vec_Vecf<LaneDim>& samples, const std::vector<decimal_t>& para,
    const Eigen::ArrayXf& breaks, const decimal_t regulator, Lane* lane) {
  Spline<LaneDegree, LaneDim> spline;
  SplineGenerator<LaneDegree, LaneDim> spline_generator;
  // 直接插值
  if (spline_generator.GetQuinticSplineBySampleFitting(
          samples, para, breaks, regulator, &spline) != kSuccess) {
    // printf("[LaneBySampleInterpolation]Cannot get lane by fitting.\n");
    return kWrongStatus;
  }
  lane->set_position_spline(spline);
  if (!lane->IsValid()) return kWrongStatus;
  return kSuccess;
}

}  // namespace common
