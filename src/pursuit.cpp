#include "robot.hpp"
#include <algorithm>
#include <cmath>

class Intersects {
    public: double x1,
                   y1,
                   x2,
                   y2;

            bool xy1,
                 xy2;
};

Intersects line_circle_intersect(Position pos, double point_one[], double point_two[], double radius) {
    double x1 = point_one[0] - pos.x;
    double x2 = point_two[0] - pos.x;
    double y1 = point_one[1] - pos.y;
    double y2 = point_two[1] - pos.y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double d_points = sqrt(dx*dx + dy*dy);
    double D = x1*y2 - x2*y1;

    double discriminant = radius*radius * d_points*d_points - D*D;

    Intersects solutions;

    if (discriminant < 0) return solutions;

    solutions.x1 = (D*dy + (dy < 0 ? -1 : 1) * dx * sqrt(discriminant)) / (d_points*d_points);
    solutions.y1 = (-D*dx + fabs(dy) * sqrt(discriminant)) / (d_points*d_points);
    solutions.x1 += pos.x;
    solutions.y1 += pos.y;
    solutions.xy1 = true;

    if ( ( solutions.x1 < std::min(point_one[0], point_two[0]) || solutions.x1 > std::max(point_one[0], point_two[0]) )
      && ( solutions.y1 < std::min(point_one[1], point_two[1]) || solutions.y1 > std::max(point_one[1], point_two[1]) )
    ) solutions.xy1 = false;

    if (discriminant == 0) return solutions;

    solutions.x2 = (D*dy - (dy < 0 ? -1 : 1) * dx * sqrt(discriminant)) / (d_points*d_points);
    solutions.y2 = (-D*dx - fabs(dy) * sqrt(discriminant)) / (d_points*d_points);
    solutions.x2 += pos.x;
    solutions.y2 += pos.y;

    if ( ( solutions.x2 < std::min(point_one[0], point_two[0]) || solutions.x2 > std::max(point_one[0], point_two[0]) )
      && ( solutions.y2 < std::min(point_one[1], point_two[1]) || solutions.y2 > std::max(point_one[1], point_two[1]) )
    ) solutions.xy2 = false;

    return solutions;
}
