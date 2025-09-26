#include <meshing/triangulator.hpp>

#include <iterator>
#include <stdexcept>

#include <fmt/format.h>

#include <geometry/point.hpp>
#include <meshing/internal_boost_rtree.hpp>
#include <meshing/mesh.hpp>

namespace diamond_fem::meshing {

namespace {

geometry::Point ToDiamondFemPoint(const internal::CGALPoint &point) {
  return geometry::Point(point.x(), point.y());
}

internal::BoostPoint ToBoostPoint(const geometry::Point &point) {
  return internal::BoostPoint(point.GetX(), point.GetY());
}

internal::CGALPoint ToCGALPoint(const geometry::Point &point) {
  return internal::CGALPoint(point.GetX(), point.GetY());
}

} // namespace

Triangulator::Triangulator(std::vector<PointWithBorderInfo> points,
                           std::vector<internal::BorderRef> borders,
                           std::vector<Constraint> constraints)
    : points_(points), borders_(borders), constraints_(constraints) {}

void Triangulator::BuildTriangulation() {
  LoadPoints_();
  LoadConstraints_();
}

internal::Triangulation Triangulator::GetTriangulation() const {
  return triangulation_;
}

std::vector<MeshTriangle> Triangulator::ExtractTriangles() {
  FillRTreeIndex_();
  ExtractTrianglesFromTriangulation_();
  IndexTriangles_();
  ApplyConstraintsToTriangles_();

  return triangles_;
}

void Triangulator::LoadConstraints_() {
  for (const auto &constraint : constraints_) {
    const auto p_1 = points_[constraint.start_point_idx].point;
    const auto p_2 = points_[constraint.end_point_idx].point;

    const auto cgal_p_1 = ToCGALPoint(p_1);
    const auto cgal_p_2 = ToCGALPoint(p_2);

    triangulation_.insert_constraint(cgal_p_1, cgal_p_2);
  }
}

void Triangulator::LoadPoints_() {
  for (const auto &point : points_) {
    triangulation_.insert(ToCGALPoint(point.point));
  }
}

void Triangulator::ExtractTrianglesFromTriangulation_() {
  for (auto face = triangulation_.finite_faces_begin();
       face != triangulation_.finite_faces_end(); face++) {
    auto vertex_indices = std::vector<int>{};

    for (int i = 0; i < 3; i++) {
      const auto point = ToDiamondFemPoint(face->vertex(i)->point());
      vertex_indices.push_back(GetPointIndex_(point));
    }

    triangles_.push_back(MeshTriangle{
        .p_1_idx = vertex_indices[0],
        .p_2_idx = vertex_indices[1],
        .p_3_idx = vertex_indices[2],
    });
  }
}

void Triangulator::FillRTreeIndex_() {
  for (int i = 0; i < points_.size(); i++) {
    rtree_.insert(std::make_pair(ToBoostPoint(points_[i].point), i));
  }
}

int Triangulator::GetPointIndex_(const geometry::Point &point) const {
  auto result = std::vector<internal::BoostPointWithPointIdx>{};
  rtree_.query(boost::geometry::index::nearest(ToBoostPoint(point), 1),
               std::back_inserter(result));
  if (result.size() != 1) {
    throw std::runtime_error(fmt::format(
        "Triangulator::GetPointIndex: result.size() = {}", result.size()));
  }
  return result.at(0).second;
}

void Triangulator::IndexTriangles_() {
  triangles_adjacent_to_point_.resize(points_.size());
  for (int triangle_idx = 0; triangle_idx < triangles_.size(); triangle_idx++) {
    const auto &triangle = triangles_[triangle_idx];
    const auto vertex_indices = std::vector{
        triangle.p_1_idx,
        triangle.p_2_idx,
        triangle.p_3_idx,
    };
    std::ranges::for_each(vertex_indices, [&](const auto &point_idx) {
      triangles_adjacent_to_point_[point_idx].push_back(triangle_idx);
    });
  }
}

} // namespace diamond_fem::meshing
