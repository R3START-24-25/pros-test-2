#include "robot.hpp"
#include <algorithm>
#include <cmath>

class Point {
    public: double x, y;
            Point(double X, double Y) {
                x = X; y = Y;
            }
            Point();
};

class Intersects {
    public: double x1, y1, x2, y2;
            bool   xy1, xy2;
};

double dist_between_pts(Point point_one, Point point_two) {
    double dx = point_two.x - point_one.x;
    double dy = point_two.y - point_one.y;

    double dist = sqrt(dx*dx + dy*dy);
    return dist;
}

Intersects line_circle_intersect(Position pos, Point point_one, Point point_two, double radius) {
    double x1 = point_one.x - pos.x;
    double x2 = point_two.x - pos.x;
    double y1 = point_one.y - pos.y;
    double y2 = point_two.y - pos.y;

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

    if ( ( solutions.x1 < std::min(point_one.x, point_two.x) || solutions.x1 > std::max(point_one.x, point_two.x) )
      && ( solutions.y1 < std::min(point_one.y, point_two.y) || solutions.y1 > std::max(point_one.y, point_two.y) )
    ) solutions.xy1 = false;

    if (discriminant == 0) return solutions;

    solutions.x2 = (D*dy - (dy < 0 ? -1 : 1) * dx * sqrt(discriminant)) / (d_points*d_points);
    solutions.y2 = (-D*dx - fabs(dy) * sqrt(discriminant)) / (d_points*d_points);
    solutions.x2 += pos.x;
    solutions.y2 += pos.y;
    solutions.xy2 = true;

    if ( ( solutions.x2 < std::min(point_one.x, point_two.x) || solutions.x2 > std::max(point_one.x, point_two.x) )
      && ( solutions.y2 < std::min(point_one.y, point_two.y) || solutions.y2 > std::max(point_one.y, point_two.y) )
    ) solutions.xy2 = false;

    return solutions;
}

Point find_target(Position pos, Point path[], int *last_found_index, double look_ahead) {
    Point target;
    Intersects intersects = line_circle_intersect(pos, path[*last_found_index], path[*last_found_index+1], look_ahead);
    
    if (!(intersects.xy1 || intersects.xy2)) return find_target(pos, path, &++(*last_found_index), look_ahead);

    Point intersects_1 = Point(intersects.x1, intersects.y1);
    Point intersects_2 = Point(intersects.x2, intersects.y2);
    double intersect_1_dist = dist_between_pts(path[*last_found_index+1], intersects_1);
    double intersect_2_dist = dist_between_pts(path[*last_found_index+1], intersects_2);

    if (intersect_1_dist > intersect_2_dist) {
        target.x = intersects.x2;
        target.y = intersects.y2;
    } else {
        target.x = intersects.x1;
        target.y = intersects.y1;
    }
    
    Point robot_pos = Point(pos.x, pos.y);
    double bot_to_path    = dist_between_pts(path[*last_found_index+1], target);
    double target_to_path = dist_between_pts(path[*last_found_index+1], target);

    if (target_to_path > bot_to_path) return find_target(pos, path, &++(*last_found_index), look_ahead);

    return target;
}
