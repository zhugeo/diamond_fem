#ifndef _DIAMOND_FEM_MESHING_INTERNAL_BOOST_RTREE_HPP
#define _DIAMOND_FEM_MESHING_INTERNAL_BOOST_RTREE_HPP

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace diamond_fem::meshing {

namespace internal {

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

enum class PointType { kEnd, kStart };

struct BorderInfo {
  int border_idx;
  PointType type;
};

using BoostPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using RTree = bgi::rtree<BoostPoint, bgi::quadratic<16>>;

using BoostPointWithBorderInfo = std::pair<BoostPoint, BorderInfo>;
using RTreeWithBorderInfo =
    bgi::rtree<BoostPointWithBorderInfo, bgi::quadratic<16>>;

using BoostPointWithPointIdx = std::pair<BoostPoint, int>;
using RTreeWithPointIdx =
    bgi::rtree<BoostPointWithPointIdx, bgi::quadratic<16>>;

} // namespace internal

} // namespace diamond_fem::meshing

#endif // _DIAMOND_FEM_MESHING_INTERNAL_BOOST_RTREE_HPP
