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
#include <QVBoxLayout>
#include <QLabel>
#include <QDialog>
#include <QDebug>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QFontMetrics>
#include <QListWidget>
#include <QSplitter>
#include <QCheckBox>

#include <vector>

#include <Node.h>
#include <Point.h>
#include <Entities.h>
#include <Params.h>
#include <TimeTable.h>
#include <PlanningResult.h>

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

void show_results(int argc, char** argv, const TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities,
                  const Params& params) {
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget* viz = new VizWidget(timetable, entities, params);
    win.setCentralWidget(viz);

    double max_t = timetable.get_max_time();
    int max_val = static_cast<int>(max_t * 100 + 0.5);

    QWidget* panel = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(panel);

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, max_val);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Compute minimum width to prevent layout reflow
    QFontMetrics fm(timeLabel->font());
    QString maxTimeStr = QString("Time: %1 s").arg(max_t, 0, 'f', 2);
    int labelWidth = fm.horizontalAdvance(maxTimeStr) + 20;  // +20 for padding
    timeLabel->setMinimumWidth(labelWidth);

    QDoubleSpinBox* stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, qMax(0.01, max_t));
    stepSpin->setSingleStep(0.05);
    stepSpin->setDecimals(2);
    stepSpin->setValue(0.50);  // Changed to 0.50 s as requested
    stepSpin->setSuffix(" s");

    QPushButton* prevBtn = new QPushButton("<<");
    QPushButton* nextBtn = new QPushButton(">>");

    // Enable auto-repeat
    prevBtn->setAutoRepeat(true);
    prevBtn->setAutoRepeatDelay(300);
    prevBtn->setAutoRepeatInterval(100);
    nextBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeatDelay(300);
    nextBtn->setAutoRepeatInterval(100);

    layout->addWidget(new QLabel("Step:"));
    layout->addWidget(stepSpin);
    layout->addWidget(prevBtn);
    layout->addWidget(slider);
    layout->addWidget(nextBtn);
    layout->addWidget(timeLabel);

    // Throttled live update during drag
    QTimer* dragUpdateTimer = new QTimer(&win);
    dragUpdateTimer->setInterval(50);

    QObject::connect(dragUpdateTimer, &QTimer::timeout, [=]() {
        double t = slider->value() / 100.0;
        viz->setTime(t);
    });

    QObject::connect(slider, &QSlider::sliderPressed, [=]() {
        dragUpdateTimer->start();
    });

    QObject::connect(slider, &QSlider::sliderReleased, [=]() {
        dragUpdateTimer->stop();
        double t = slider->value() / 100.0;
        viz->setTime(t);
    });

    QObject::connect(slider, &QSlider::valueChanged, [=](int val) {
        double t = val / 100.0;
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
        if (!slider->isSliderDown()) {
            viz->setTime(t);
        }
    });

    // Step buttons
    QObject::connect(prevBtn, &QPushButton::clicked, [=]() {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMax(0.0, curr_t - step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]() {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMin(max_t, curr_t + step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    slider->setValue(0);

    QDockWidget* dock = new QDockWidget;
    dock->setWidget(panel);
    win.setWindowTitle("Prioritized Hybrid A* Demo - Jeeho Ahn");
    win.addDockWidget(Qt::BottomDockWidgetArea, dock);
    win.resize(600, 600);
    win.show();
    app.exec();
}

class TrajectoryReplayVizWidget : public QWidget {
public:
    TrajectoryReplayVizWidget(const std::unordered_map<std::string, EntityMeta*>& ents,
                              const Params& p,
                              QWidget* parent = nullptr)
        : QWidget(parent), entities(ents), params(p) {
        setMinimumSize(700, 700);
    }

    void setTime(double t) {
        current_t = t;
        update();
    }

    void setSnapshot(const TimeTable* tt,
                     const TimeTable::TrajectoryLabel* label,
                     int idx,
                     int total_count) {
        snapshot_tt = tt;
        selected_label = label;
        selected_index = idx;
        selected_total = total_count;
        update();
    }

    void setFinalTimeTable(const TimeTable* tt) {
        final_tt = tt;
        update();
    }

    void setShowFinalMode(bool enabled) {
        show_final_mode = enabled;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), Qt::white);

        const TimeTable* active_tt = show_final_mode ? final_tt : snapshot_tt;
        if (!active_tt) {
            painter.setPen(Qt::black);
            painter.drawText(rect(), Qt::AlignCenter, "No selected snapshot.");
            return;
        }

        double scale_x = static_cast<double>(width()) / (params.max_x - params.min_x);
        double scale_y = static_cast<double>(height()) / (params.max_y - params.min_y);
        double sc = std::min(scale_x, scale_y);
        auto screen_x = [&](double x) { return (x - params.min_x) * sc; };
        auto screen_y = [&](double y) { return (params.max_y - y) * sc; };

        painter.setPen(Qt::lightGray);
        for (double x = params.min_x; x <= params.max_x + 1e-6; x += params.xy_resolution * 5) {
            double sx = screen_x(x);
            painter.drawLine(QPointF(sx, screen_y(params.min_y)), QPointF(sx, screen_y(params.max_y)));
        }
        for (double y = params.min_y; y <= params.max_y + 1e-6; y += params.xy_resolution * 5) {
            double sy = screen_y(y);
            painter.drawLine(QPointF(screen_x(params.min_x), sy), QPointF(screen_x(params.max_x), sy));
        }

        painter.setPen(QPen(Qt::black, 3));
        painter.drawRect(QRectF(screen_x(params.min_x) + 1, screen_y(params.max_y) + 1,
                                sc * (params.max_x - params.min_x) - 3,
                                sc * (params.max_y - params.min_y) - 3));

        auto poses = active_tt->get_poses(current_t);
        for (const auto& [ent, pose] : poses) {
            if (ent == nullptr) break;
            QColor color = (ent->type == EntityType::ROBOT) ? QColor("#555B6E") : QColor("#89B0AE");
            auto corners = get_corners(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
            QPolygonF poly;
            for (const auto& c : corners) poly << QPointF(screen_x(c.x), screen_y(c.y));
            poly << QPointF(screen_x(corners[0].x), screen_y(corners[0].y));
            painter.setPen(color);
            painter.setBrush(color);
            painter.drawPolygon(poly);

            if (ent->type == EntityType::ROBOT) {
                double front_x = pose.x + ent->size.front_length * std::cos(pose.yaw);
                double front_y = pose.y + ent->size.front_length * std::sin(pose.yaw);
                painter.setPen(QPen(QColor("FFD6BA"), 2));
                painter.drawLine(screen_x(pose.x), screen_y(pose.y), screen_x(front_x), screen_y(front_y));
            }

            painter.setPen(Qt::black);
            painter.drawText(QPointF(screen_x(pose.x), screen_y(pose.y)), QString::fromStdString(ent->name));
        }

        // Draw only the selected trajectory trace (last trajectory)
        if (!show_final_mode && selected_label && selected_label->end_time >= selected_label->start_time) {
            auto ent_it = entities.find(selected_label->entity_name);
            if (ent_it != entities.end() && ent_it->second) {
                EntityMeta* ent = ent_it->second;
                QPolygonF trace;
                double dt = std::max(0.05, params.xy_resolution);
                for (double t = selected_label->start_time; t <= selected_label->end_time + 1e-6; t += dt) {
                    Pose p = active_tt->get_pose(ent, t);
                    trace << QPointF(screen_x(p.x), screen_y(p.y));
                }
                painter.setPen(QPen(QColor(255, 80, 0), 4));
                painter.setBrush(Qt::NoBrush);
                painter.drawPolyline(trace);
            }
        }

        painter.setPen(Qt::black);
        QString mode = show_final_mode ? "FINAL TIMETABLE" : "UP TO SELECTED PATH";
        QString title = QString("[%1] Path %2/%3  Time: %4 s")
                            .arg(mode)
                            .arg(std::max(1, selected_index + 1))
                            .arg(std::max(1, selected_total))
                            .arg(current_t, 0, 'f', 2);
        painter.drawText(QRect(8, 8, width() - 16, 20), Qt::AlignLeft, title);
    }

private:
    const std::unordered_map<std::string, EntityMeta*>& entities;
    const Params& params;
    const TimeTable* snapshot_tt = nullptr;
    const TimeTable* final_tt = nullptr;
    const TimeTable::TrajectoryLabel* selected_label = nullptr;
    int selected_index = -1;
    int selected_total = 0;
    double current_t = 0.0;
    bool show_final_mode = false;
};

inline void show_trajectory_registration_replay(const TimeTable& timetable,
                                                const std::unordered_map<std::string, EntityMeta*>& entities,
                                                const Params& params) {
    if (!QApplication::instance()) {
        static int argc = 1;
        static char arg[] = "traj_replay";
        static char* argv[] = {arg};
        new QApplication(argc, argv);
    }

    const auto& snapshots = timetable.get_registration_snapshots();
    const auto& labels = timetable.get_trajectory_labels();

    QDialog dialog;
    dialog.setWindowTitle("Trajectory Registration Replay");
    dialog.resize(1500, 900);

    std::vector<TimeTable> snapshot_tables;
    snapshot_tables.reserve(snapshots.size());
    for (size_t i = 0; i < snapshots.size(); ++i) {
        snapshot_tables.push_back(timetable.build_snapshot_timetable(i));
    }

    QVBoxLayout* root = new QVBoxLayout(&dialog);
    QHBoxLayout* top = new QHBoxLayout();

    TrajectoryReplayVizWidget* viz = new TrajectoryReplayVizWidget(entities, params);
    QListWidget* list = new QListWidget;
    list->setMinimumWidth(360);
    viz->setFinalTimeTable(&timetable);

    std::unordered_map<int, const TimeTable::TrajectoryLabel*> label_by_id;
    for (const auto& m : labels) {
        label_by_id[m.id] = &m;
    }

    // Keep exact insertion order: iterate snapshots in stored sequence.
    for (size_t i = 0; i < snapshots.size(); ++i) {
        const auto& s = snapshots[i];
        const TimeTable::TrajectoryLabel* m = nullptr;
        if (label_by_id.count(s.traj_id)) {
            m = label_by_id[s.traj_id];
        }

        QString robot_name = QString::fromStdString(s.entity_name);
        if (m && !m->entity_name.empty()) {
            robot_name = QString::fromStdString(m->entity_name);
        }

        QString entry;
        if (m) {
            entry = QString("[%1] traj#%2 | robot=%3 | %4 -> %5")
                        .arg(static_cast<int>(i) + 1)
                        .arg(s.traj_id)
                        .arg(robot_name)
                        .arg(m->start_time, 0, 'f', 2)
                        .arg(m->end_time, 0, 'f', 2);
            if (m->is_transfer && !m->transferred_object_name.empty()) {
                entry += QString(" | push %1").arg(QString::fromStdString(m->transferred_object_name));
            }
        } else {
            entry = QString("[%1] traj#%2 | robot=%3")
                        .arg(static_cast<int>(i) + 1)
                        .arg(s.traj_id)
                        .arg(robot_name);
        }
        list->addItem(entry);
    }

    QWidget* panel = new QWidget;
    QHBoxLayout* ctrl = new QHBoxLayout(panel);
    QCheckBox* modeToggle = new QCheckBox("Show final timetable result");
    modeToggle->setChecked(false);

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 0);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    timeLabel->setMinimumWidth(140);

    QDoubleSpinBox* stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, 1000.0);
    stepSpin->setSingleStep(0.05);
    stepSpin->setDecimals(2);
    stepSpin->setValue(0.50);
    stepSpin->setSuffix(" s");

    QPushButton* prevBtn = new QPushButton("<<");
    QPushButton* nextBtn = new QPushButton(">>");
    prevBtn->setAutoRepeat(true);
    prevBtn->setAutoRepeatDelay(300);
    prevBtn->setAutoRepeatInterval(100);
    nextBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeatDelay(300);
    nextBtn->setAutoRepeatInterval(100);

    ctrl->addWidget(modeToggle);
    ctrl->addSpacing(10);
    ctrl->addWidget(new QLabel("Step:"));
    ctrl->addWidget(stepSpin);
    ctrl->addWidget(prevBtn);
    ctrl->addWidget(slider);
    ctrl->addWidget(nextBtn);
    ctrl->addWidget(timeLabel);

    QTimer* dragUpdateTimer = new QTimer(&dialog);
    dragUpdateTimer->setInterval(50);

    QObject::connect(dragUpdateTimer, &QTimer::timeout, [=]() {
        viz->setTime(slider->value() / 100.0);
    });

    QObject::connect(slider, &QSlider::sliderPressed, [=]() {
        dragUpdateTimer->start();
    });

    QObject::connect(slider, &QSlider::sliderReleased, [=]() {
        dragUpdateTimer->stop();
        viz->setTime(slider->value() / 100.0);
    });

    QObject::connect(slider, &QSlider::valueChanged, [=](int val) {
        double t = val / 100.0;
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
        if (!slider->isSliderDown()) {
            viz->setTime(t);
        }
    });

    QObject::connect(prevBtn, &QPushButton::clicked, [=]() {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMax(0.0, curr_t - step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]() {
        double step = stepSpin->value();
        double max_t = slider->maximum() / 100.0;
        double curr_t = slider->value() / 100.0;
        double new_t = qMin(max_t, curr_t + step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    auto refresh_selection = [=]() {
        int row = list->currentRow();
        if (row < 0 || row >= static_cast<int>(snapshot_tables.size())) return;
        const TimeTable* tt = &snapshot_tables[static_cast<size_t>(row)];
        const TimeTable::TrajectoryLabel* label = nullptr;
        int traj_id = snapshots[static_cast<size_t>(row)].traj_id;
        auto label_it = label_by_id.find(traj_id);
        if (label_it != label_by_id.end()) {
            label = label_it->second;
        }

        viz->setSnapshot(tt, label, row, static_cast<int>(snapshot_tables.size()));
        viz->setShowFinalMode(modeToggle->isChecked());

        double max_t = modeToggle->isChecked() ? timetable.get_max_time() : tt->get_max_time();
        int max_val = static_cast<int>(max_t * 100 + 0.5);
        slider->setRange(0, std::max(0, max_val));
        double t = slider->value() / 100.0;
        if (t > max_t) {
            slider->setValue(max_val);
        } else {
            viz->setTime(t);
        }
    };

    QObject::connect(list, &QListWidget::currentRowChanged, &dialog, [=](int) {
        refresh_selection();
    });

    QObject::connect(modeToggle, &QCheckBox::toggled, &dialog, [=](bool enabled) {
        viz->setShowFinalMode(enabled);
        (void)enabled;
        refresh_selection();
    });

    top->addWidget(viz, 1);
    top->addWidget(list, 0);
    root->addLayout(top, 1);
    root->addWidget(panel, 0);

    if (list->count() > 0) {
        list->setCurrentRow(list->count() - 1);
        refresh_selection();
    }

    dialog.exec();
}

/**
 * Visualizes two timetables side-by-side for comparison.
 */
inline void show_comparison(int argc, char** argv,
                     const TimeTable& tt_initial,
                     const TimeTable& tt_best,
                     const std::unordered_map<std::string, EntityMeta*>& entities,
                     const Params& params) {
    QApplication app(argc, argv);
    QMainWindow win;
    win.setWindowTitle("ALNS Comparison: Initial vs Best Solution");

    QWidget* centralWidget = new QWidget;
    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);
    win.setCentralWidget(centralWidget);

    QHBoxLayout* vizLayout = new QHBoxLayout;
    mainLayout->addLayout(vizLayout);

    // Initial View
    QVBoxLayout* initialLayout = new QVBoxLayout;
    QLabel* initialLabel = new QLabel("<b>Initial Greedy Solution</b>");
    initialLabel->setAlignment(Qt::AlignCenter);
    VizWidget* vizInitial = new VizWidget(tt_initial, entities, params);
    initialLayout->addWidget(initialLabel);
    initialLayout->addWidget(vizInitial);
    vizLayout->addLayout(initialLayout);

    // Separator line
    QFrame* line = new QFrame;
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    vizLayout->addWidget(line);

    // Best View
    QVBoxLayout* bestLayout = new QVBoxLayout;
    double best_makespan = tt_best.get_max_time();
    QLabel* bestLabel = new QLabel(QString("<b>Best ALNS Solution (Makespan: %1s)</b>").arg(best_makespan, 0, 'f', 2));
    bestLabel->setAlignment(Qt::AlignCenter);
    VizWidget* vizBest = new VizWidget(tt_best, entities, params);
    bestLayout->addWidget(bestLabel);
    bestLayout->addWidget(vizBest);
    vizLayout->addLayout(bestLayout);

    // Shared Controls
    double max_t = std::max(tt_initial.get_max_time(), tt_best.get_max_time());
    int max_val = static_cast<int>(max_t * 100 + 0.5);

    QWidget* panel = new QWidget;
    QHBoxLayout* ctrlLayout = new QHBoxLayout(panel);
    mainLayout->addWidget(panel);

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, max_val);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    QFontMetrics fm(timeLabel->font());
    QString maxTimeStr = QString("Time: %1 s").arg(max_t, 0, 'f', 2);
    timeLabel->setMinimumWidth(fm.horizontalAdvance(maxTimeStr) + 20);

    QDoubleSpinBox* stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, qMax(0.01, max_t));
    stepSpin->setSingleStep(0.05);
    stepSpin->setValue(0.50);
    stepSpin->setSuffix(" s");

    QPushButton* prevBtn = new QPushButton("<<");
    QPushButton* nextBtn = new QPushButton(">>");
    prevBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeat(true);

    ctrlLayout->addWidget(new QLabel("Step:"));
    ctrlLayout->addWidget(stepSpin);
    ctrlLayout->addWidget(prevBtn);
    ctrlLayout->addWidget(slider);
    ctrlLayout->addWidget(nextBtn);
    ctrlLayout->addWidget(timeLabel);

    // Coordination
    auto updateTime = [=](double t) {
        vizInitial->setTime(t);
        vizBest->setTime(t);
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
    };

    QObject::connect(slider, &QSlider::valueChanged, [=](int val) {
        updateTime(val / 100.0);
    });

    QObject::connect(prevBtn, &QPushButton::clicked, [=]() {
        double new_t = qMax(0.0, (slider->value() / 100.0) - stepSpin->value());
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]() {
        double new_t = qMin(max_t, (slider->value() / 100.0) + stepSpin->value());
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    win.resize(1200, 700);
    win.show();
    app.exec();
}

void show_results(int argc, char** argv, const TimeTable& timetable, const std::unordered_map<std::string, EntityMeta*>& entities, const std::vector<Trajectory>& all_trajectories, const Params& params) {
    QApplication app(argc, argv);
    QMainWindow win;
    VizWidget_old* viz = new VizWidget_old(timetable, entities, all_trajectories, params);
    win.setCentralWidget(viz);

    double max_t = 0.0;
    for (const auto& traj : all_trajectories) {
        if (!traj.waypoints.empty()) max_t = timetable.get_max_time();
    }
    int max_val = static_cast<int>(max_t * 100 + 0.5);

    QWidget* panel = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(panel);

    QSlider* slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, max_val);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    QLabel* timeLabel = new QLabel("Time: 0.00 s");
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // Compute minimum width to prevent layout reflow
    QFontMetrics fm(timeLabel->font());
    QString maxTimeStr = QString("Time: %1 s").arg(max_t, 0, 'f', 2);
    int labelWidth = fm.horizontalAdvance(maxTimeStr) + 20;
    timeLabel->setMinimumWidth(labelWidth);

    QDoubleSpinBox* stepSpin = new QDoubleSpinBox();
    stepSpin->setRange(0.01, qMax(0.01, max_t));
    stepSpin->setSingleStep(0.05);
    stepSpin->setDecimals(2);
    stepSpin->setValue(0.50);  // Changed to 0.50 s
    stepSpin->setSuffix(" s");

    QPushButton* prevBtn = new QPushButton("<<");
    QPushButton* nextBtn = new QPushButton(">>");

    prevBtn->setAutoRepeat(true);
    prevBtn->setAutoRepeatDelay(300);
    prevBtn->setAutoRepeatInterval(100);
    nextBtn->setAutoRepeat(true);
    nextBtn->setAutoRepeatDelay(300);
    nextBtn->setAutoRepeatInterval(100);

    layout->addWidget(new QLabel("Step:"));
    layout->addWidget(stepSpin);
    layout->addWidget(prevBtn);
    layout->addWidget(slider);
    layout->addWidget(nextBtn);
    layout->addWidget(timeLabel);

    QTimer* dragUpdateTimer = new QTimer(&win);
    dragUpdateTimer->setInterval(50);

    QObject::connect(dragUpdateTimer, &QTimer::timeout, [=]() {
        double t = slider->value() / 100.0;
        viz->setTime(t);
    });

    QObject::connect(slider, &QSlider::sliderPressed, [=]() {
        dragUpdateTimer->start();
    });

    QObject::connect(slider, &QSlider::sliderReleased, [=]() {
        dragUpdateTimer->stop();
        double t = slider->value() / 100.0;
        viz->setTime(t);
    });

    QObject::connect(slider, &QSlider::valueChanged, [=](int val) {
        double t = val / 100.0;
        timeLabel->setText(QString("Time: %1 s").arg(t, 0, 'f', 2));
        if (!slider->isSliderDown()) {
            viz->setTime(t);
        }
    });

    QObject::connect(prevBtn, &QPushButton::clicked, [=]() {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMax(0.0, curr_t - step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    QObject::connect(nextBtn, &QPushButton::clicked, [=]() {
        double step = stepSpin->value();
        double curr_t = slider->value() / 100.0;
        double new_t = qMin(max_t, curr_t + step);
        slider->setValue(static_cast<int>(new_t * 100 + 0.5));
    });

    slider->setValue(0);

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
        qDebug() << " - " << QString::fromStdString("goalPose") << ": (" << goal_pose.x << ", " << goal_pose.y << ", yaw=" << goal_pose.yaw << ")";
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

// ==========================================
// PLANNING DEBUG VISUALIZATION (Visualization.h)
// ==========================================

// Helper to get corners for drawing (Local calculation)
inline std::vector<Point> get_corners_local(double x, double y, double yaw, double fl, double rl, double w) {
    double cos_y = std::cos(yaw);
    double sin_y = std::sin(yaw);
    double dx_fl = fl * cos_y;
    double dy_fl = fl * sin_y;
    double dx_rl = rl * cos_y;
    double dy_rl = rl * sin_y;
    double dx_w = (w/2.0) * sin_y;
    double dy_w = (w/2.0) * cos_y;

    return {
        {x + dx_fl - dx_w, y + dy_fl + dy_w}, // Front Left
        {x + dx_fl + dx_w, y + dy_fl - dy_w}, // Front Right
        {x - dx_rl + dx_w, y - dy_rl - dy_w}, // Rear Right
        {x - dx_rl - dx_w, y - dy_rl + dy_w}  // Rear Left
    };
}

class PlanningDebugWidget : public QDialog {
public:
    PlanningDebugWidget(const TimeTable& tt, 
                        RobotMeta* robot,
                        const PlanningResult& result,
                        double start_time,
                        const Pose& start,
                        const Pose& goal,
                        const Params& p,
                        QWidget* parent = nullptr)
        : QDialog(parent), timetable(tt), 
          active_robot(robot), plan(result), 
          plan_start_time(start_time),
          start_pose(start), goal_pose(goal), params(p) 
    {
        plan_duration = 0.0;
        if (!plan.waypoints.empty()) {
            plan_duration = plan.waypoints.back().time;
        } else {
            // Default window if plan is empty/failed
            plan_duration = 10.0; 
        }
        
        setWindowTitle(QString("Debug: %1 (t=%2 to %3)")
                       .arg(QString::fromStdString(robot->name))
                       .arg(start_time, 0, 'f', 2)       // %2: start time, 2 decimal places
                       .arg(start_time + plan_duration, 0, 'f', 2)); // %3: end time
        
        // Resize to accommodate side panel
        resize(1200, 800);
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        p.fillRect(rect(), Qt::white);

        // --- Layout Configuration ---
        int legend_width = 260; 
        int view_width = width() - legend_width; 
        int view_height = height();

        // --- 1. Transform Setup ---
        double margin = 1.0; 
        double w_real = (params.max_x - params.min_x) + 2*margin;
        double h_real = (params.max_y - params.min_y) + 2*margin;
        
        // Scale to fit ONLY in the view area
        double scale = std::min((double)view_width / w_real, (double)view_height / h_real);

        auto toScreen = [&](double x, double y) {
            return QPointF((x - params.min_x + margin) * scale, 
                           (params.max_y - y + margin) * scale); // Flip Y
        };

        // --- 2. Draw Workspace Boundary ---
        p.setPen(QPen(Qt::black, 3, Qt::SolidLine));
        p.setBrush(Qt::NoBrush);
        p.drawRect(QRectF(toScreen(params.min_x, params.max_y), toScreen(params.max_x, params.min_y)));

        // Vertical separator line
        p.setPen(QPen(Qt::gray, 1));
        p.drawLine(view_width, 0, view_width, height());

        // --- 3. Draw Entities (Context) ---
        for (const auto& [entity, timeline] : timetable.get_database()) {
            if (entity == active_robot) continue; 

            // Interpolate at exactly the start time
            Pose current_pose = interpolate_pose(timeline, plan_start_time);
            
            QColor bodyColor;
            if (entity->type == EntityType::OBJECT) {
                bodyColor = QColor(255, 140, 0, 150); // Orange for Objects
            } else {
                bodyColor = QColor(100, 100, 100, 150); // Dark Grey for Robots
            }

            drawShape(p, current_pose, entity, bodyColor, QPen(Qt::black, 1), toScreen, QString::fromStdString(entity->name));
            
            // Draw trail only if it moves in the window
            drawTrail(p, timeline, plan_start_time, plan_start_time + plan_duration, toScreen);
        }

        // --- 4. Draw Active Robot Plan ---
        if (!plan.waypoints.empty()) {
            QPolygonF path;
            for (const auto& wp : plan.waypoints) {
                path << toScreen(wp.x, wp.y);
            }
            p.setPen(QPen(QColor(0, 102, 255), 3, Qt::SolidLine)); 
            p.drawPolyline(path);
        }

        // --- 5. Draw Active Start / Goal ---
        drawShape(p, start_pose, active_robot, QColor(50, 205, 50, 180), QPen(Qt::black, 2), toScreen, "START");
        drawShape(p, goal_pose, active_robot, QColor(220, 20, 60, 180), QPen(Qt::black, 2, Qt::DashLine), toScreen, "GOAL");

        // --- 6. Draw Legend (Side Panel) ---
        drawLegend(p, view_width + 10, 20, legend_width - 20);
    }

private:
    Pose interpolate_pose(const std::map<double, Pose>& timeline, double t) {
        if (timeline.empty()) return Pose();
        auto it = timeline.lower_bound(t);
        
        if (it == timeline.end()) return timeline.rbegin()->second;
        if (it->first == t || it == timeline.begin()) return it->second;

        auto prev = std::prev(it);
        double t1 = prev->first;
        double t2 = it->first;
        Pose p1 = prev->second;
        Pose p2 = it->second;

        double ratio = (t - t1) / (t2 - t1);
        Pose p;
        p.x = p1.x + ratio * (p2.x - p1.x);
        p.y = p1.y + ratio * (p2.y - p1.y);
        
        double dyaw = p2.yaw - p1.yaw;
        while (dyaw > M_PI) dyaw -= 2 * M_PI;
        while (dyaw < -M_PI) dyaw += 2 * M_PI;
        p.yaw = p1.yaw + ratio * dyaw;
        return p;
    }

    template<typename Func>
    void drawTrail(QPainter& p, const std::map<double, Pose>& timeline, double t_start, double t_end, Func toScreen) {
        QPolygonF path;
        auto it = timeline.lower_bound(t_start);
        
        if (it != timeline.begin()) {
             Pose start_p = interpolate_pose(timeline, t_start);
             path << toScreen(start_p.x, start_p.y);
        }

        for (; it != timeline.end() && it->first <= t_end; ++it) {
            path << toScreen(it->second.x, it->second.y);
        }
        
        if (it != timeline.end()) {
             Pose end_p = interpolate_pose(timeline, t_end);
             path << toScreen(end_p.x, end_p.y);
        }

        if (path.size() > 1) {
            p.setPen(QPen(QColor(150, 150, 150), 2, Qt::DashLine));
            p.setBrush(Qt::NoBrush);
            p.drawPolyline(path);
        }
    }

    template<typename Func>
    void drawShape(QPainter& p, Pose pose, EntityMeta* ent, QColor brush, QPen pen, Func toScreen, QString label = "") {
        auto c = get_corners_local(pose.x, pose.y, pose.yaw, ent->size.front_length, ent->size.rear_length, ent->size.width);
        QPolygonF poly;
        for (auto& pt : c) poly << toScreen(pt.x, pt.y);
        
        p.setBrush(brush);
        p.setPen(pen);
        p.drawPolygon(poly);

        if (!label.isEmpty()) {
            p.setPen(Qt::black);
            p.drawText(toScreen(pose.x, pose.y) + QPointF(0, -5), label);
        }
        
        QPointF center = toScreen(pose.x, pose.y);
        QPointF front = toScreen(pose.x + 0.3 * std::cos(pose.yaw), pose.y + 0.3 * std::sin(pose.yaw));
        p.setPen(Qt::black);
        p.drawLine(center, front);
    }

    void drawLegend(QPainter& p, int x, int y, int w) {
        p.setBrush(QColor(245, 245, 245)); 
        p.setPen(Qt::black);
        p.drawRect(x, y, w, 220); 

        int cy = y + 25;
        auto item = [&](QString t, QColor c, Qt::PenStyle s, bool line) {
            p.setPen(Qt::black); 
            p.drawText(x + 40, cy + 5, t);
            
            if(line) { 
                p.setPen(QPen(c, 2, s)); 
                p.drawLine(x+10, cy, x+30, cy); 
            } else { 
                p.setBrush(c); 
                p.setPen(Qt::black); 
                p.drawRect(x+10, cy-7, 15, 15); 
            }
            cy += 25;
        };

        p.setFont(QFont("Arial", 10, QFont::Bold));
        p.drawText(x+10, y+18, "Context (t=" + QString::number(plan_start_time, 'f', 1) + ")");
        p.setFont(QFont("Arial", 9));
        
        cy += 5;
        item("Active Plan", QColor(0, 102, 255), Qt::SolidLine, true);
        item("Start Pose", QColor(50, 205, 50, 180), Qt::SolidLine, false);
        item("Goal Pose", QColor(220, 20, 60, 180), Qt::DashLine, false);
        item("Object", QColor(255, 140, 0, 150), Qt::SolidLine, false);
        item("Other Robot", QColor(100, 100, 100, 150), Qt::SolidLine, false);
        item("Other Trail", QColor(150, 150, 150), Qt::DashLine, true);
    }

    const TimeTable& timetable;
    RobotMeta* active_robot;
    PlanningResult plan;
    double plan_start_time;
    double plan_duration;
    Pose start_pose;
    Pose goal_pose;
    Params params;
};

// Main entry point for visualization
void visualize_planning_debug(const TimeTable& tt, 
                              RobotMeta* robot,
                              const PlanningResult& result,
                              double start_time,
                              const Pose& start,
                              const Pose& goal,
                              const Params& params) 
{
    if (!QApplication::instance()) {
        static int argc = 1;
        static char arg[] = "viz";
        static char* argv[] = {arg};
        new QApplication(argc, argv);
    }
    PlanningDebugWidget w(tt, robot, result, start_time, start, goal, params);
    w.exec();
}
#endif // VISUALIZATION_H
