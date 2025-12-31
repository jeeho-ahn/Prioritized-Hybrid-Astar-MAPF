/*************************************
 * Path Visualization using Qt
 *
 * 2025.10.24
 * Jeeho Ahn, jeeho@umich.edu
*************************************/


#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QPainter>
#include <QSlider>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QLabel>
#include <QDialog>
#include <vector>

#include <Node.h>
#include <Point.h>
#include <Entities.h>
#include <Params.h>
#include <TimeTable.h>

std::tuple<double, double, double> interpolate_timed_path(const std::vector<Waypoint> &waypoints, double t)
{
    if (t <= waypoints[0].time)
        return {waypoints[0].x, waypoints[0].y, waypoints[0].yaw};
    if (t >= waypoints.back().time)
        return {waypoints.back().x, waypoints.back().y, waypoints.back().yaw};

    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        auto &wp1 = waypoints[i];
        auto &wp2 = waypoints[i + 1];
        if (wp1.time <= t && t <= wp2.time)
        {
            double frac = (t - wp1.time) / (wp2.time - wp1.time);
            double x = wp1.x + frac * (wp2.x - wp1.x);
            double y = wp1.y + frac * (wp2.y - wp1.y);

            // FIX: Use pi_2_pi to get the shortest angular distance (range -PI to PI)
            double dyaw = pi_2_pi(wp2.yaw - wp1.yaw);

            double yaw = wp1.yaw + frac * dyaw;
            yaw = mod2pi(yaw); // Keep the final result in [0, 2PI)
            return {x, y, yaw};
        }
    }
    return {waypoints.back().x, waypoints.back().y, waypoints.back().yaw};
}

// to be depricated
class VizWidget_old : public QWidget {
private:
    const TimeTable& timetable;
    const std::unordered_map<std::string, EntityMeta*>& entities;
    const std::vector<Trajectory>& trajectories;
    const Params& params;
    double current_t = 0.0;
    double max_t = 0.0;

    void drawFilledPolygon(QPainter& p, const Corners& corners, const QColor& color, const std::function<double(double)>& screen_x, const std::function<double(double)>& screen_y) {
        QPolygonF poly;
        for (const auto& c : corners) poly << QPointF(screen_x(c.x), screen_y(c.y));
        poly << QPointF(screen_x(corners[0].x), screen_y(corners[0].y));
        p.setPen(color);
        p.setBrush(color);
        p.drawPolygon(poly);
    }

public:
    VizWidget_old(const TimeTable& tt, const std::unordered_map<std::string, EntityMeta*>& ents, const std::vector<Trajectory>& trajs, const Params& p)
        : timetable(tt), entities(ents), trajectories(trajs), params(p) {
        for (const auto& traj : trajectories) {
            if (!traj.waypoints.empty()) max_t = timetable.get_max_time();
        }
        setMinimumSize(600, 600);
    }

    void setTime(double t) {
        current_t = t;
        update();
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QPainter p(this);
        p.fillRect(rect(), Qt::white);

        double scale_x = static_cast<double>(width()) / (params.max_x - params.min_x);
        double scale_y = static_cast<double>(height()) / (params.max_y - params.min_y);
        double sc = std::min(scale_x, scale_y);
        auto screen_x = [&](double x) { return (x - params.min_x) * sc; };
        auto screen_y = [&](double y) { return (params.max_y - y) * sc; };

        // Draw grid
        p.setPen(Qt::lightGray);
        for (double x = params.min_x; x <= params.max_x + 1e-6; x += params.xy_resolution*5) {
            double sx = screen_x(x);
            double sy1 = screen_y(params.min_y);
            double sy2 = screen_y(params.max_y);
            p.drawLine(QPointF(sx, sy1), QPointF(sx, sy2));
        }
        for (double y = params.min_y; y <= params.max_y + 1e-6; y += params.xy_resolution*5) {
            double sy = screen_y(y);
            double sx1 = screen_x(params.min_x);
            double sx2 = screen_x(params.max_x);
            p.drawLine(QPointF(sx1, sy), QPointF(sx2, sy));
        }

        // Draw boundary
        p.setPen(QPen(Qt::black,3));
        p.drawRect(QRectF(screen_x(params.min_x) + 1, screen_y(params.max_y) + 1, sc * (params.max_x - params.min_x) - 3, sc * (params.max_y - params.min_y) - 3));


        // Draw paths
        p.setPen(QPen(QColor("#FFD6C2"), 4));
        for (const auto& traj : trajectories) {
            QPolygonF path_poly;
            for (const auto& wp : traj.waypoints) {
                path_poly << QPointF(screen_x(wp.x), screen_y(wp.y));
            }
            p.drawPolyline(path_poly);
        }

        // Draw entities
        auto poses = timetable.get_poses(current_t);
        for (const auto& [ent, pose] : poses) {
            QColor color = (ent->type == EntityType::ROBOT) ? QColor("#555B6E") : QColor("#89B0AE");
            auto corners = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            drawFilledPolygon(p, corners, color, screen_x, screen_y);

            // Indicate front with a red arrow for robots
            if (ent->type == EntityType::ROBOT) {
                double front_x = pose.x + ent->size.front_length * std::cos(pose.yaw);
                double front_y = pose.y + ent->size.front_length * std::sin(pose.yaw);
                p.setPen(QPen(QColor("FFD6BA"), 2));
                p.drawLine(screen_x(pose.x), screen_y(pose.y), screen_x(front_x), screen_y(front_y));
            }

            // Draw Name
            p.setPen(Qt::black);
            p.drawText(QPointF(screen_x(pose.x), screen_y(pose.y)), QString::fromStdString(ent->name));
        }
    }
};


class VizWidget : public QWidget {
private:
    const TimeTable& timetable;
    const std::unordered_map<std::string, EntityMeta*>& entities;
    const Params& params;
    double current_t = 0.0;
    double max_t = 0.0;

    void drawFilledPolygon(QPainter& p, const Corners& corners, const QColor& color, const std::function<double(double)>& screen_x, const std::function<double(double)>& screen_y) {
        QPolygonF poly;
        for (const auto& c : corners) poly << QPointF(screen_x(c.x), screen_y(c.y));
        poly << QPointF(screen_x(corners[0].x), screen_y(corners[0].y));
        p.setPen(color);
        p.setBrush(color);
        p.drawPolygon(poly);
    }

public:
    VizWidget(const TimeTable& tt, const std::unordered_map<std::string, EntityMeta*>& ents, const Params& p)
        : timetable(tt), entities(ents), params(p) {
        max_t = tt.get_max_time();
        setMinimumSize(600, 600);
    }

    void setTime(double t) {
        current_t = t;
        update();
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QPainter p(this);
        p.fillRect(rect(), Qt::white);

        double scale_x = static_cast<double>(width()) / (params.max_x - params.min_x);
        double scale_y = static_cast<double>(height()) / (params.max_y - params.min_y);
        double sc = std::min(scale_x, scale_y);
        auto screen_x = [&](double x) { return (x - params.min_x) * sc; };
        auto screen_y = [&](double y) { return (params.max_y - y) * sc; };

        // Draw grid
        p.setPen(Qt::lightGray);
        for (double x = params.min_x; x <= params.max_x + 1e-6; x += params.xy_resolution*5) {
            double sx = screen_x(x);
            double sy1 = screen_y(params.min_y);
            double sy2 = screen_y(params.max_y);
            p.drawLine(QPointF(sx, sy1), QPointF(sx, sy2));
        }
        for (double y = params.min_y; y <= params.max_y + 1e-6; y += params.xy_resolution*5) {
            double sy = screen_y(y);
            double sx1 = screen_x(params.min_x);
            double sx2 = screen_x(params.max_x);
            p.drawLine(QPointF(sx1, sy), QPointF(sx2, sy));
        }

        // Draw boundary
        p.setPen(QPen(Qt::black,3));
        p.drawRect(QRectF(screen_x(params.min_x) + 1, screen_y(params.max_y) + 1, sc * (params.max_x - params.min_x) - 3, sc * (params.max_y - params.min_y) - 3));

        // Draw entities
        auto poses = timetable.get_poses(current_t);
        for (const auto& [ent, pose] : poses) {
            if(ent==nullptr){break;} // temp fix
            QColor color = (ent->type == EntityType::ROBOT) ? QColor("#555B6E") : QColor("#89B0AE");
            auto corners = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            drawFilledPolygon(p, corners, color, screen_x, screen_y);

            // Indicate front with a red arrow for robots
            if (ent->type == EntityType::ROBOT) {
                double front_x = pose.x + ent->size.front_length * std::cos(pose.yaw);
                double front_y = pose.y + ent->size.front_length * std::sin(pose.yaw);
                p.setPen(QPen(QColor("FFD6BA"), 2));
                p.drawLine(screen_x(pose.x), screen_y(pose.y), screen_x(front_x), screen_y(front_y));
            }

            // Draw Name
            p.setPen(Qt::black);
            p.drawText(QPointF(screen_x(pose.x), screen_y(pose.y)), QString::fromStdString(ent->name));
        }
    }
};

void show_results(int argc, char** argv, const TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities, const std::vector<Trajectory>& all_trajectories, const Params& params) {
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget_old* viz = new VizWidget_old(timetable, entities, all_trajectories, params);
    win.setCentralWidget(viz);

    QWidget* panel = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(panel);
    QSlider* slider = new QSlider(Qt::Horizontal);
    double max_t = 0.0;
    for (const auto& traj : all_trajectories) {
        if (!traj.waypoints.empty()) max_t = timetable.get_max_time();
    }
    slider->setRange(0, static_cast<int>(max_t * 100));
    layout->addWidget(slider);
    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    layout->addWidget(timeLabel);

    QObject::connect(slider, &QSlider::valueChanged, [viz, timeLabel](int val) {
        double t = val / 100.0;
        viz->setTime(t);
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
    });

    QDockWidget* dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    win.show();
    app.exec();
}

// without trajectory input
void show_results(int argc, char** argv, const TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities,
                  const Params& params) {
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget* viz = new VizWidget(timetable, entities, params);
    win.setCentralWidget(viz);

    QWidget* panel = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(panel);
    QSlider* slider = new QSlider(Qt::Horizontal);
    double max_t = timetable.get_max_time();
    slider->setRange(0, static_cast<int>(max_t * 100));
    layout->addWidget(slider);
    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    layout->addWidget(timeLabel);

    QObject::connect(slider, &QSlider::valueChanged, [viz, timeLabel](int val) {
        double t = val / 100.0;
        viz->setTime(t);
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
    });

    QDockWidget* dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    win.show();
    app.exec();
}

// In Visualization.h or a new DebugViz.h (include <QWidget>, <QPainter>, <QApplication>, <QDialog> if not already)
class DebugVisualizer : public QDialog {
public:
    DebugVisualizer(const TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities,
                    const Params& params, double query_time,
                    const Pose& start_pose = {}, const Pose& goal_pose = {}, QWidget* parent = nullptr)
        : QDialog(parent), timetable_(timetable), entities_(entities), params_(params),
        query_time_(query_time), start_pose_(start_pose), goal_pose_(goal_pose) {
        setWindowTitle(QString("Debug State at t=%1").arg(query_time));
        resize(800, 600);  // Adjust as needed

        // Debug print: Check entities
        qDebug() << "Debug Viz: " << entities_.size() << " entities at t=" << query_time;
        for (const auto& [name, ent] : entities_) {
            Pose p = timetable_.get_pose(ent, query_time_);
            qDebug() << " - " << QString::fromStdString(name) << ": (" << p.x << ", " << p.y << ", yaw=" << p.yaw << ")";
        }
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Compute scale (fit workspace to window)
        double w = params_.max_x - params_.min_x;
        double h = params_.max_y - params_.min_y;
        double scale_x = (width() - 100) / w;  // Add margins
        double scale_y = (height() - 100) / h;
        double scale = std::min(scale_x, scale_y);
        qDebug() << "Scale:" << scale << "Workspace:" << w << "x" << h;

        // Save original state
        painter.save();

        // Apply transform for world coords
        painter.translate(50, height() - 50);  // Bottom-left origin with margin
        painter.scale(scale, -scale);  // Flip Y, scale

        // Draw bounds (with fixed device pen)
        painter.restore();  // Draw boundary in device coords for visibility
        painter.setPen(QPen(Qt::black, 2));  // Fixed 2px width
        double dev_min_x = 50;
        double dev_min_y = 50;
        double dev_w = w * scale;
        double dev_h = h * scale;
        painter.drawRect(QRectF(dev_min_x, dev_min_y, dev_w, dev_h));
        painter.save();  // Re-apply transform for world drawing
        painter.translate(50, height() - 50);
        painter.scale(scale, -scale);

        // Pre-compute poses and corners for all entities
        std::unordered_map<std::string, Pose> poses;
        std::unordered_map<std::string, Corners> all_corners;
        for (const auto& [name, ent] : entities_) {
            Pose p = timetable_.get_pose(ent, query_time_);
            poses[name] = p;
            Corners corners = get_corners(p.x, p.y, p.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            all_corners[name] = corners;
        }

        // Detect collisions: Check pairwise intersections
        std::vector<std::pair<std::string, std::string>> collisions;
        auto entity_names = std::vector<std::string>{};
        for (const auto& [name, ent] : entities_) entity_names.push_back(name);
        for (size_t i = 0; i < entity_names.size(); ++i) {
            for (size_t j = i + 1; j < entity_names.size(); ++j) {
                const auto& name1 = entity_names[i];
                const auto& name2 = entity_names[j];
                if (rectangles_intersect(all_corners.at(name1), all_corners.at(name2))) {
                    collisions.emplace_back(name1, name2);
                    qDebug() << "Collision detected between" << QString::fromStdString(name1) << "and" << QString::fromStdString(name2);
                }
            }
        }

        // Draw entities (scaled)
        for (const auto& [name, ent] : entities_) {
            QColor color = (ent->type == EntityType::ROBOT) ? Qt::blue : Qt::green;
            painter.setBrush(color);
            painter.setPen(QPen(Qt::black, 0.02));  // Fixed small world-space width (adjust if needed)

            // Get corners and draw polygon
            const Corners& corners = all_corners.at(name);
            QPolygonF poly;
            for (const auto& c : corners) {
                poly << QPointF(c.x, c.y);
            }
            painter.drawPolygon(poly);
        }

        // Highlight collisions (red overlay or outline)
        painter.setPen(QPen(Qt::red, 0.05, Qt::SolidLine));
        painter.setBrush(QBrush(QColor(255, 0, 0, 50)));  // Semi-transparent red
        for (const auto& [name1, name2] : collisions) {
            // Draw outlines around colliding pairs
            QPolygonF poly1, poly2;
            for (const auto& c : all_corners.at(name1)) poly1 << QPointF(c.x, c.y);
            for (const auto& c : all_corners.at(name2)) poly2 << QPointF(c.x, c.y);
            painter.drawPolygon(poly1);
            painter.drawPolygon(poly2);
            // Optional: Draw intersection area if needed (advanced: compute clip, but skip for simplicity)
        }

        // Draw start/goal (scaled)
        painter.setPen(QPen(Qt::black, 0.02));
        if (start_pose_.x != 0 || start_pose_.y != 0) {
            painter.setBrush(Qt::yellow);
            painter.drawEllipse(QPointF(start_pose_.x, start_pose_.y), 0.1, 0.1);
        }
        if (goal_pose_.x != 0 || goal_pose_.y != 0) {
            painter.setBrush(Qt::red);
            painter.drawEllipse(QPointF(goal_pose_.x, goal_pose_.y), 0.1, 0.1);
            if (start_pose_.x != 0 || start_pose_.y != 0) {
                painter.setPen(QPen(Qt::red, 0.05, Qt::DashLine));
                painter.drawLine(QPointF(start_pose_.x, start_pose_.y), QPointF(goal_pose_.x, goal_pose_.y));
            }
        }

        // Future overlays (scaled, semi-transparent)
        painter.setOpacity(0.3);
        for (double dt = 5.0; dt <= 15.0; dt += 5.0) {
            for (const auto& [name, ent] : entities_) {
                if (ent->type != EntityType::ROBOT) continue;
                Pose p_future = timetable_.get_pose(ent, query_time_ + dt);
                painter.setBrush(Qt::gray);
                painter.setPen(QPen(Qt::black, 0.02));
                Corners corners = get_corners(p_future.x, p_future.y, p_future.yaw,
                                              ent->size.front_length, ent->size.rear_length, ent->size.width);
                QPolygonF poly;
                for (const auto& c : corners) poly << QPointF(c.x, c.y);
                painter.drawPolygon(poly);
            }
        }
        painter.setOpacity(1.0);

        // Restore for unscaled drawing (e.g., labels)
        painter.restore();

        // Draw labels in device coordinates (unscaled)
        QFont font = painter.font();
        font.setPointSize(10);  // Fixed small size
        painter.setFont(font);
        painter.setPen(Qt::black);
        for (const auto& [name, ent] : entities_) {
            Pose p = timetable_.get_pose(ent, query_time_);
            // World to device
            double dev_x = 50 + (p.x - params_.min_x) * scale;
            double dev_y = 50 + (params_.max_y - p.y) * scale;  // Adjust for flipped Y (since bottom is max_y after flip?)
            painter.drawText(QPointF(dev_x, dev_y + 10), QString::fromStdString(name));  // Pixel offset
        }
    }

private:
    const TimeTable& timetable_;
    const std::unordered_map<std::string, EntityMeta*>& entities_;
    const Params& params_;
    double query_time_;
    Pose start_pose_;
    Pose goal_pose_;
};

// The visualize_current_state function remains the same
void visualize_current_state(const TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities,
                             const Params& params, double query_time,
                             const Pose& start_pose = {}, const Pose& goal_pose = {}) {
    QApplication* app = qobject_cast<QApplication*>(QCoreApplication::instance());
    bool own_app = false;
    if (!app) {
        static int local_argc = 1;
        static char* local_argv[] = {const_cast<char*>("debug_viz")};
        app = new QApplication(local_argc, local_argv);
        own_app = true;
    }

    DebugVisualizer viz(timetable, entities, params, query_time, start_pose, goal_pose);
    viz.exec();  // Blocks until closed

    if (own_app) {
        delete app;
    }
}

class SearchTreeViz : public QDialog {
public:
    SearchTreeViz(const std::vector<Node>& nodes, const Params& params, QWidget* parent = nullptr)
        : QDialog(parent), nodes_(nodes), params_(params) {
        setWindowTitle("PHA* Search Tree Debug");
        resize(800, 600);

        // Debug log
        qDebug() << "Search Tree Viz: " << nodes.size() << " nodes";
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Compute scale (same as DebugVisualizer)
        double w = params_.max_x - params_.min_x;
        double h = params_.max_y - params_.min_y;
        double scale_x = (width() - 100) / w;
        double scale_y = (height() - 100) / h;
        double scale = std::min(scale_x, scale_y);

        painter.translate(50, height() - 50);  // Bottom-left origin
        painter.scale(scale, -scale);  // Flip Y

        // Draw bounds
        painter.setPen(QPen(Qt::black, 2.0 / scale));  // Visible width
        painter.drawRect(QRectF(params_.min_x, params_.min_y, w, h));

        // Draw tree: edges first (lines to parent), then nodes (dots)
        painter.setPen(QPen(Qt::black, 0.01));  // Thin lines
        for (const auto& node : nodes_) {
            if (node.parent) {
                painter.drawLine(QPointF(node.x, node.y), QPointF(node.parent->x, node.parent->y));
            }
        }

        // Nodes as small cyan dots
        painter.setBrush(Qt::cyan);
        painter.setPen(Qt::NoPen);  // No outline for dots
        for (const auto& node : nodes_) {
            painter.drawEllipse(QPointF(node.x, node.y), 0.05, 0.05);  // Small radius
        }
    }

private:
    const std::vector<Node>& nodes_;
    const Params& params_;
};

void visualize_search_tree(const std::vector<Node>& nodes, const Params& params) {
    QApplication* app = qobject_cast<QApplication*>(QCoreApplication::instance());
    bool own_app = false;
    if (!app) {
        static int local_argc = 1;
        static char* local_argv[] = {const_cast<char*>("search_viz")};
        app = new QApplication(local_argc, local_argv);
        own_app = true;
    }

    SearchTreeViz viz(nodes, params);
    viz.exec();  // Blocks until closed

    if (own_app) {
        delete app;
    }
}


#endif // VISUALIZATION_H
