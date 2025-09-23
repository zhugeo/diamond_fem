#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include <analysis_task/border.hpp>
#include <analysis_task/border_condition.hpp>
#include <geometry/arc.hpp>
#include <geometry/line.hpp>
#include <meshing/point_stepper.hpp>


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>
#include <iostream>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Определяем тип точки
typedef bg::model::point<float, 2, bg::cs::cartesian> point;

// Тип значения для хранения в дереве
// Можно хранить только точки или пары <точка, доп.данные>
typedef std::pair<point, unsigned> value;


namespace df = diamond_fem;

// int main() {
//   auto borders = std::vector<std::shared_ptr<df::analysis_task::Border>>();

//   borders.push_back(std::make_shared<df::analysis_task::Border>(
//       std::make_shared<df::analysis_task::BorderConditionConstant>(0.0),
//       std::make_shared<df::geometry::Line>(df::geometry::Point{0.0, 0.0},
//                                            df::geometry::Point{1.0, 1.0})));

//   //auto stepper = df::meshing::PointStepper(std::move(borders));

//   // const auto points = stepper.Step();

//   // auto out = std::ofstream("out.txt");
//   // for (const auto &point : points) {
//   //   out << point.point.GetX() << " " << point.point.GetY() << std::endl;
//   // }
//   // out.close();
// }

int main() {
  // Создаем R-tree
  bgi::rtree<value, bgi::quadratic<16>> rtree;

  // Заполняем данными
  for (unsigned i = 0; i < 10; ++i) {
    point p(i + 0.0f, i + 0.5f);
    rtree.insert(std::make_pair(p, i));
  }

  // Ищем ближайшие точки
  std::vector<value> nearest;
  rtree.query(bgi::nearest(point(0, 0), 3), std::back_inserter(nearest));

  // Выводим результаты
  std::cout << "Ближайшие точки:" << std::endl;
  for (const auto &v : nearest) {
    std::cout << "Точка: " << bg::wkt(v.first) << " ID: " << v.second
              << std::endl;
  }

  return 0;
}
