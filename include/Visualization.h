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

#include <vector>

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


#endif // VISUALIZATION_H
