/*
 * PointXYZRGBConfidenceRatio.hpp
 *
 *  Created on: Nov 26, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *   This file defines our custom pcl type, ie including a confidence_ratio.
 *   Adapted from https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/impl/point_types.hpp
 */

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ostream>

namespace pcl {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
struct PointXYZRGBConfidenceRatio {
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  float confidence_ratio;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline PointXYZRGBConfidenceRatio(const float& _x, const float& _y, const float& _z, const uint8_t& _r, const uint8_t& _g, const uint8_t& _b, const float& _confidence_ratio = 1.0f)
  {
    x = _x;
    y = _y;
    z = _z;
    r = _r;
    g = _g;
    b = _b;
    confidence_ratio = _confidence_ratio;
  }

  inline PointXYZRGBConfidenceRatio()
  {
    x = y = z = 0.0f;
    r = g = b = 0;
    confidence_ratio = 1.0f;
  }

  friend std::ostream& operator<<(std::ostream& os, const PointXYZRGBConfidenceRatio& p);
};
#pragma GCC diagnostic pop

PCL_EXPORTS std::ostream& operator<<(std::ostream& os, const pcl::PointXYZRGBConfidenceRatio& p);

}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGBConfidenceRatio,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (std::uint32_t, rgba, rgba)
                                  (float, confidence_ratio, confidence_ratio))

namespace elevation_mapping {
  using PointCloudType = pcl::PointCloud<pcl::PointXYZRGBConfidenceRatio>;
}

