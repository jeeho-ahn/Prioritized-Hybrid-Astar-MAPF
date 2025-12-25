#ifndef POINT_H
#define POINT_H

#include <vector>

struct Point {
    double x, y;
};

using Corners = std::vector<Point>;

std::pair<double, double> project(const Corners& corners, const Point& axis) {
    double min_d = std::numeric_limits<double>::infinity();
    double max_d = -std::numeric_limits<double>::infinity();
    for (const auto& c : corners) {
        double dot = c.x * axis.x + c.y * axis.y;
        min_d = std::min(min_d, dot);
        max_d = std::max(max_d, dot);
    }
    return {min_d, max_d};
}

bool overlap(std::pair<double, double> p1, std::pair<double, double> p2) {
    return p1.second >= p2.first && p2.second >= p1.first;
}

std::vector<Point> get_axes(const Corners& corners) {
    std::vector<Point> axes;
    for (size_t i = 0; i < 4; ++i) {
        Point p1 = corners[i];
        Point p2 = corners[(i + 1) % 4];
        double ex = p2.x - p1.x;
        double ey = p2.y - p1.y;
        Point normal = {-ey, ex};
        double norm = std::hypot(normal.x, normal.y);
        if (norm > 0) {
            normal.x /= norm;
            normal.y /= norm;
            axes.push_back(normal);
        }
    }
    return axes;
}

Corners get_corners(double x, double y, double yaw, double front, double rear, double width) {
    double cos = std::cos(yaw);
    double sin = std::sin(yaw);
    Corners corners = {
        {x - rear * cos - (width / 2) * sin, y - rear * sin + (width / 2) * cos},
        {x + front * cos - (width / 2) * sin, y + front * sin + (width / 2) * cos},
        {x + front * cos + (width / 2) * sin, y + front * sin - (width / 2) * cos},
        {x - rear * cos + (width / 2) * sin, y - rear * sin - (width / 2) * cos}
    };
    return corners;
}

bool rectangles_intersect(const Corners& corners1, const Corners& corners2) {
    auto axes1 = get_axes(corners1);
    auto axes2 = get_axes(corners2);
    std::vector<Point> all_axes;
    all_axes.reserve(axes1.size() + axes2.size());
    all_axes.insert(all_axes.end(), axes1.begin(), axes1.end());
    all_axes.insert(all_axes.end(), axes2.begin(), axes2.end());
    for (const auto& axis : all_axes) {
        auto p1 = project(corners1, axis);
        auto p2 = project(corners2, axis);
        if (!overlap(p1, p2)) return false;
    }
    return true;
}


#endif // POINT_H
